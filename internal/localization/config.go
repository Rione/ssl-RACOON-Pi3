package localization

import (
	"encoding/json"
	"fmt"
	"io"
	"math"
	"os"
)

// GeometryConfig は 4 輪オムニの機体パラメータ。
//
// ここの値はすべて未確定である (計画 §3.3 / §12-A)。車輪半径・モーメントアーム・
// 取付角が 4 つのコードベースで食い違っており、符号規約も世代で反転している。
// ハードコードせず必ず設定ファイルから読むのは、P1 の同定結果で差し替えられる
// ようにするため。
//
//	| パラメータ       | 旧世代 STM | 新世代 STM | RAVEN | 本リポジトリ |
//	| 車輪半径         | 27 mm      | 30 mm      | 26 mm | 30 mm        |
//	| モーメントアーム | 85 mm      | 75 mm      | 90 mm | --           |
//	| 取付角           | 55/135/-135/-55 | 同左  | 60/135/-135/-60 | -- |
type GeometryConfig struct {
	// Comment は設定ファイルに出どころを書き残すための欄。読み込み時は無視する。
	//
	// **必ず書くこと。** 「この 74 mm はどこから来たのか」が後から分からなくなると、
	// CAD と同定値の食い違い (研究 §2.3) のような問題を切り分けられなくなる。
	Comment []string `json:"_comment,omitempty"`

	// WheelAnglesDeg は各車輪の取付角 [deg]。ワールドではなくロボット系で、
	// +x (前方) から反時計回り。並びは論理輪番号 (FL, BL, BR, FR) の順。
	WheelAnglesDeg [NumWheels]float64 `json:"wheelAnglesDeg"`

	// WheelRadiusM は車輪半径 [m]。個体差の較正 (§8) のために輪ごとに持つ。
	WheelRadiusM [NumWheels]float64 `json:"wheelRadiusM"`

	// MomentArmM は機体中心から車輪接地点までの距離 [m]。回転の腕の長さ。
	MomentArmM float64 `json:"momentArmM"`

	// WheelSlotOrder は「SPI フレーム上の並び」から「論理輪番号」への写像。
	// WheelSlotOrder[slot] = 論理輪番号。
	//
	// 計画 §12-A4 の通り、世代間で FL と FR が入れ替わっている疑いがある。
	// 恒等写像 [0,1,2,3] が既定だが、P1 の同定で確定させること。
	WheelSlotOrder [NumWheels]int `json:"wheelSlotOrder"`

	// WheelSigns は各車輪の符号 [+1 or -1]。
	//
	// 計画 §3.4 の通り、基準形は旧世代 STM と RAVEN が一致する
	//
	//	v_i = sin(a_i)*vx - cos(a_i)*vy - R*omega
	//
	// で、新世代 STM (omni_drive.c) だけが完全に符号反転している。
	// どちらが実機の「エンコーダが返す値」と整合するかは未検証であり、
	// これが「RAVEN の EKF は効果が測れなかった」の最有力の原因候補である。
	// P1 で指令と実測を突き合わせて確定させること。
	WheelSigns [NumWheels]float64 `json:"wheelSigns"`
}

// DefaultGeometry は既定の機体パラメータを返す。
//
// **これは「幾何」ではなく「有効運動学パラメータ」である。**
// CAD の設計値 (CADGeometry) とはわざと違う値が入っている。理由は下記。
//
// # 幾何の値と、オドメトリに使うべき値は違う
//
// 3D モデル (Robot_V2.step) を解析すると、機械としての値は**厳密**である:
// 取付角 +-60.0000 / +-135.0000 度、接地点半径 78.450 mm、車軸は位置ベクトルと
// 完全に反平行、ローラの傾き 0.0000 度。**全機体が同じ設計である。**
//
// ところが実機のログで「車輪だけの推測航法」(フィルタも vision の補正も通さず、
// 0.3 秒の窓で車輪から積分して vision の変位と比べる) を測ると、
// **CAD の値は最良ではない** (9 本・27 窓):
//
//	前輪角 phi   平均誤差   中央値        モーメントアーム   向きのドリフト
//	  55 度      8.93 mm    4.06 mm         70 mm            0.95 deg
//	  56 度      8.95 mm    3.98 mm         74 mm            0.46 deg
//	  60 度(CAD) 9.75 mm    4.91 mm         78.45 mm(CAD)    1.10 deg
//
// **角度もアームも、幾何より小さい値のほうがよく合う。** 向きが揃っているのが
// 大事で、これはオムニ車輪のローラのコンプライアンスと滑りで説明がつく:
// 自由方向 (ローラが転がる向き) の運動の一部が車輪の回転に漏れるので、
// ある機体運動に対して車輪が幾何の予想ほど回らない。当てはめるとその差が
// 「実効的に小さい取付角・短いアーム」として現れる。5 度は結合係数
// tan(5 deg) = 0.09 に相当する。
//
// オムニ車輪較正の文献でいう effective kinematic parameters (EKP) がこれで、
// **幾何を測るのではなく有効ヤコビアンを較正せよ**というのが定説である。
// 旧世代 STM ファームが +-55 度の輪にだけ cos x 1.05 の経験補正を入れていたのも
// 同じ現象への手当てと思われる。
//
// # だから
//
//   - **フィルタにはこの値 (有効値) を使う。**
//   - **CAD は「幾何の権威」として別に持つ** (CADGeometry)。検算と、
//     較正が暴走していないかの歯止めに使う。
//   - 有効値は**ゴム・摩耗・荷重で変わる**ので、機体ごと・定期的に測り直す。
//     手順は docs/self-localization-research-20260923.md §2.6。
//
// 値の出どころは PoC が vision 基準で 6 本から同定したもの
// (trajpoc-dataset/geometry/geometry-racoon-56011.json)。符号 -1 は実機の実測で、
// 旧既定の +1 は逆であり推定は最初から壊れていた (Trajectory POC Log §5-17)。
func DefaultGeometry() GeometryConfig {
	return GeometryConfig{
		// 有効取付角。幾何は +-60 / +-135 度 (CADGeometry)。
		WheelAnglesDeg: [NumWheels]float64{55.4, 136.1, -136.3, -57.4},
		// 有効転がり半径。4 輪で約 5% 違う。
		WheelRadiusM: [NumWheels]float64{0.02933, 0.02803, 0.02818, 0.02805},
		// 有効モーメントアーム。幾何は 78.45 mm。**78.45 にすると向きの
		// ドリフトが 2 倍になる** (実機のその場回転と旋回で確認)。
		MomentArmM:     0.074,
		WheelSlotOrder: [NumWheels]int{WheelFL, WheelBL, WheelBR, WheelFR},
		WheelSigns:     [NumWheels]float64{-1, -1, -1, -1},
	}
}

// CADGeometry は 3D モデル Robot_V2.step の**幾何としての**設計値を返す。
//
// 取付角は厳密に +-60.0000 / +-135.0000 度、接地点半径 78.450 mm
// (車軸が位置ベクトルと厳密に反平行なので「取付角 = 位置角」、
// モーメントアーム = 接地点半径)。ローラは 15 個 x 2 列・角ピッチ 12 度の千鳥、
// ピッチ円 25.000 mm、ハブ外径 26.705 mm、**ローラの傾き 0.0000 度**。
// **全機体がこの設計である。**
//
// 車輪半径は CAD からは決まらない (ローラのゴム部の形状が STEP に入っていない)。
// 下限 26.705 mm、上限 約 29.5 mm の窓しか出ないので、公称 28.4 mm を置いてある。
//
// **フィルタにこれを使ってはいけない。** オドメトリに要るのは有効値のほうで、
// 実機では CAD の値は最良ではない (DefaultGeometry のコメントに実測がある)。
// この関数は**検算と歯止め**のためにある: 較正した有効値が CAD から
// 大きく離れたら、較正が壊れているか機体が壊れている。
func CADGeometry() GeometryConfig {
	const r = 0.0284
	return GeometryConfig{
		WheelAnglesDeg: [NumWheels]float64{60, 135, -135, -60},
		WheelRadiusM:   [NumWheels]float64{r, r, r, r},
		MomentArmM:     0.07845,
		WheelSlotOrder: [NumWheels]int{WheelFL, WheelBL, WheelBR, WheelFR},
		WheelSigns:     [NumWheels]float64{-1, -1, -1, -1},
	}
}

// Validate は設定が運動学として成立するかを調べる。
func (g *GeometryConfig) Validate() error {
	if g.MomentArmM <= 0 || math.IsNaN(g.MomentArmM) {
		return fmt.Errorf("momentArmM must be positive, got %v", g.MomentArmM)
	}
	var seen [NumWheels]bool
	for i := 0; i < NumWheels; i++ {
		if g.WheelRadiusM[i] <= 0 || math.IsNaN(g.WheelRadiusM[i]) {
			return fmt.Errorf("wheelRadiusM[%d] must be positive, got %v", i, g.WheelRadiusM[i])
		}
		if g.WheelSigns[i] != 1 && g.WheelSigns[i] != -1 {
			return fmt.Errorf("wheelSigns[%d] must be +1 or -1, got %v", i, g.WheelSigns[i])
		}
		if math.IsNaN(g.WheelAnglesDeg[i]) {
			return fmt.Errorf("wheelAnglesDeg[%d] is NaN", i)
		}
		slot := g.WheelSlotOrder[i]
		if slot < 0 || slot >= NumWheels {
			return fmt.Errorf("wheelSlotOrder[%d] = %d is out of range", i, slot)
		}
		if seen[slot] {
			return fmt.Errorf("wheelSlotOrder maps two slots to wheel %d", slot)
		}
		seen[slot] = true
	}
	return nil
}

// LoadGeometry は JSON から機体パラメータを読む。
// 未指定のフィールドは DefaultGeometry の値が残る。
func LoadGeometry(r io.Reader) (GeometryConfig, error) {
	g := DefaultGeometry()
	dec := json.NewDecoder(r)
	dec.DisallowUnknownFields()
	if err := dec.Decode(&g); err != nil {
		return GeometryConfig{}, fmt.Errorf("decode geometry: %w", err)
	}
	if err := g.Validate(); err != nil {
		return GeometryConfig{}, err
	}
	return g, nil
}

// LoadGeometryFile は path の JSON から機体パラメータを読む。
// path が空なら DefaultGeometry を返す。
func LoadGeometryFile(path string) (GeometryConfig, error) {
	if path == "" {
		return DefaultGeometry(), nil
	}
	f, err := os.Open(path)
	if err != nil {
		return GeometryConfig{}, fmt.Errorf("open geometry %s: %w", path, err)
	}
	defer f.Close()
	return LoadGeometry(f)
}

// NoiseConfig はフィルタのプロセス雑音・観測雑音。
//
// **2026-09-23 に、実機で測れたものを実測値へ置き換えた**
// (docs/self-localization-research-20260923.md §4.6)。出典は Trajectory POC Log。
// **まだ実測が無いものには「未計測」と明記してある。**
// 合成データに合わせ込んではいけない。合わせ込むと後で剥がせない嘘になる。
type NoiseConfig struct {
	// AccelNoise は並進の加速度雑音密度 [m/s^2/sqrt(Hz)]。
	//
	// 加速度計を予測に使わないので、実際の加減速はすべてこの雑音で表現される。
	// 定速度モデルに対して、実加速度が rms a、相関時間 tau なら
	// 等価な白色 PSD は q = 2*a^2*tau。a = 4 m/s^2、tau = 0.1 s なら
	// q = 3.2 -> 密度 1.8。**ただし実機はこれより大きいほうが良かった。**
	//
	// **実測のトレードオフ** (実機 2 本、vision を 0.3 s 落としたときの誤差):
	//
	//	AccelNoise   円 0.4 m/s   8 の字 0.4 m/s
	//	   2.0        8.2 mm       8.9 mm
	//	   4.0        7.4 mm        --
	//	   8.0        5.1 mm       7.0 mm
	//
	// 車輪雑音が大きい (0.23 + 0.07|omega| rad/s) ので、欠落中に車輪の変化へ
	// 速く追従するにはフィルタの帯域が要る。一方 vision がある間の精度は
	// AccelNoise にほとんど依らない (「次のフレームとの差」は 1.86-1.87 mm で不変)。
	// **この 2 つを見て 4.0 を採った。**
	//
	// 合成データでは逆に、小さいほうが vision のある間は良く、欠落時に悪い
	// (4.0 -> 1.0 で 0.5 s 欠落の誤差が 3.1 -> 7.1 mm、
	// TestProcessNoiseSensitivityMap)。向きは同じ。
	//
	// **実機で決めるときは NIS を見ること** (Estimator.VisionNIS)。
	// 真値が要らず、平均が観測の次元 3 になるのが正しい。
	// `loc_replay -qsweep` がその表を出す。
	AccelNoise float64 `json:"accelNoise"`

	// AngAccelNoise は角加速度の雑音密度 [rad/s^2/sqrt(Hz)]。
	//
	// 同じ考え方で、角加速度 rms 15 rad/s^2、相関時間 0.05 s なら
	// q = 22.5 -> 密度 4.7。既定 5.0。
	//
	// **以前の 20.0 は大きすぎた。** 1 周期 (8.6 ms) で角速度が 1.9 rad/s
	// 変わりうるという意味になり、向きの予測の不確かさが 0.53 度/周期と
	// vision の雑音 (0.32 度) を超えてしまう。vision の雑音を実測値へ
	// 下げたことで初めて見えた (それまでは vision 側が 0.5 度で隠れていた)。
	AngAccelNoise float64 `json:"angAccelNoise"`

	// SlipNoise はスリップ状態の駆動雑音密度 [m/s^2/sqrt(Hz)]。**未計測**。
	//
	// IMU が無い間、車輪観測には v と s が和としてしか現れない。分離しているのは
	// vision の位置観測と、この雑音および SlipTau が作る過程モデルの違いだけである。
	// つまり**測れない定数がスリップ推定の結果を独占的に決める**。
	SlipNoise float64 `json:"slipNoise"`

	// SlipTau はスリップの一次減衰の時定数 [s]。**未計測**。計画 §4.3 の既定は 0.1 s。
	//
	// **0 にしてはいけない (= ランダムウォークにしてはいけない)。** Yu ほか
	// (IROS 2023, arXiv:2209.15140) の可観測性解析により、スリップが可観測なのは
	// 減衰率 alpha > 0 のときだけで、alpha = 0 では不可観測になる。
	SlipTau float64 `json:"slipTau"`

	// EnableSlip はスリップ状態を使うか。false なら s は常にゼロに固定される。
	EnableSlip bool `json:"enableSlip"`

	// SlipRScaleRef はスリップ量で車輪の R を膨らませるときの基準 [m/s]。
	// SlipRScaleMax は膨らませる倍率の上限。
	//
	//	R_wheel *= min( exp(|s| / SlipRScaleRef), SlipRScaleMax )
	//
	// Yu ほか (IROS 2023) の W_y = W_0 exp(||u||) と同じ考え。滑っているあいだ
	// 車輪観測を自動的に信用しなくなる。同論文は軸ごとに効かせて 1 軸だけ悪化させて
	// おり、著者自身が適応のさせ方が不適切だったと述べているので、ここでは**等方**に
	// 効かせ、上限でクランプする。SlipRScaleRef <= 0 で無効。
	SlipRScaleRef float64 `json:"slipRScaleRef"`
	SlipRScaleMax float64 `json:"slipRScaleMax"`

	// WheelNoise は車輪角速度の観測雑音のうち**速度に依存しない分** [rad/s]。
	//
	// SPI は rad/s x 100 の int16 なので量子化だけで 0.01/sqrt(12) = 0.0029 ある。
	// 実測は internal/localization/wheelcal.go の FitNullVector で取れる
	// (vision も時刻合わせも要らない)。
	WheelNoise float64 `json:"wheelNoise"`

	// WheelNoiseSpeedCoef は車輪角速度に比例する雑音の係数 [無次元]。
	//
	//	sigma_i = sqrt( WheelNoise^2 + (WheelNoiseSpeedCoef * |omega_i|)^2 )
	//
	// **オムニ車輪の有効転がり半径は一定ではない。** 本機はローラ 15 個 x 2 列・
	// 角ピッチ 12 度なので、接触ローラが移るたびに転がり半径と接触点が変わる
	// (Mecanum の文献でいう polygon 効果)。0.4 m/s では約 68 Hz のリップルになり、
	// 125 Hz のサンプリングでは白色雑音として扱ってよい。
	// STM の速度制御のリップルとローラの弾性もここに入る。
	//
	// 出発点 0.02 は PoC の「走行中は 0.3 rad/s 程度が妥当」(車輪 14.3 rad/s 時) から。
	WheelNoiseSpeedCoef float64 `json:"wheelNoiseSpeedCoef"`

	// VisionPosNoise / VisionAngNoise は vision の観測雑音の標準偏差 [m] / [rad]。
	//
	// **実測値** (Trajectory POC Log §5-11): 位置 0.2..0.4 mm RMS、向き 0.25..0.32 度。
	// 150 ms 窓の 2 次式当てはめの残差なので、**短期の再現性であって絶対精度ではない**。
	// カメラ較正の偏りやフィールド位置依存の系統誤差はここに入っていない。
	// 絶対精度は別に測ること (研究 §6.4)。AdaptiveVisionR はこの既定値を
	// 外したときの保険でもある。
	VisionPosNoise float64 `json:"visionPosNoise"`
	VisionAngNoise float64 `json:"visionAngNoise"`

	// VisionTimeSigma は vision の時刻に残る不確かさ [s]。
	//
	// **計画 §5.3 の手当てそのもの**: 「残る時刻の不確かさ sigma_t は、
	// vision の R に (v*sigma_t)^2 と (omega*sigma_t)^2 を足して吸う」。
	//
	// 時刻が sigma_t だけ揺れると、速度 v で動いているロボットの位置は
	// v*sigma_t だけ揺れて見える。**これは速度に比例するので、定数の R では
	// 表せない。** 入れないと、速い走りで共分散が過小になり
	// (= 過信)、位置制御が信じてはいけない値を信じる。
	//
	// 実測: timesync の写像に残る片道遅延は 0..4 ms、vision のジッタが数 ms。
	// 既定 2 ms。3 m/s なら 6 mm ぶん R が膨らむ。
	VisionTimeSigma float64 `json:"visionTimeSigma"`

	// AdaptiveVisionR は vision の R を変分ベイズでオンライン推定するか。
	//
	// Sarkka & Nummenmaa の VB-AKF (IEEE TAC 2009) / Sarkka & Hartikainen
	// (arXiv:1302.0681)。逆ガンマ事前分布の十分統計量 (nu, V) を持ち回り、
	// 予測で忘却係数 rho を掛け、更新で数回の固定点反復を回す。
	// 逆ガンマ事前分布が下限を与えるので、クランプが原理から出る。
	AdaptiveVisionR bool `json:"adaptiveVisionR"`
	// AdaptiveForgetting は忘却係数 rho (0 < rho <= 1)。116 Hz で 0.98 なら時定数 0.4 s。
	//
	// 固定点反復は 1 観測あたり 1 回だけ回す (VB-AKF の N = 1)。116 Hz で観測が
	// 来続けるので、反復の役目は時間方向の再帰が果たす。
	AdaptiveForgetting float64 `json:"adaptiveForgetting"`
	// AdaptiveMinScale / AdaptiveMaxScale は R を公称値の何倍まで動かすか。
	// 発散防止のため必須 (計画 §4.4)。
	AdaptiveMinScale float64 `json:"adaptiveMinScale"`
	AdaptiveMaxScale float64 `json:"adaptiveMaxScale"`

	// EnableParamEstimation は機体パラメータの倍率をオンライン較正するか。
	//
	// Mozzarelli ほか (arXiv:2403.13452) は車輪半径とジャイロバイアスを状態にし、
	// 絶対位置が来ているあいだだけ補正、来ないあいだは凍結する方式で、
	// 130 秒・70 m の位置喪失での累積誤差を 5.3 m -> 0.35 m にしている。
	//
	// 本件では CAD (60 度 / 78.45 mm) と同定値 (55.4 度 / 74 mm) が食い違っており、
	// **どちらかを選ぶ代わりにオンラインで較正する**
	// (docs/self-localization-research-20260923.md §4.1)。
	EnableParamEstimation bool `json:"enableParamEstimation"`
	// ParamScaleNoise は並進・回転倍率のランダムウォーク雑音密度 [1/sqrt(s)]。
	// 電池電圧・温度・摩耗で動く量なので時定数は分オーダー。小さく置く。
	ParamScaleNoise float64 `json:"paramScaleNoise"`
	// ParamAngleNoise は取付角補正のランダムウォーク雑音密度 [rad/sqrt(s)]。
	ParamAngleNoise float64 `json:"paramAngleNoise"`
	// ParamScaleLimit は倍率の可動域 (1 +- この値)。既定 0.25。
	ParamScaleLimit float64 `json:"paramScaleLimit"`
	// ParamAngleLimitDeg は取付角補正の可動域 [deg]。既定 10。
	ParamAngleLimitDeg float64 `json:"paramAngleLimitDeg"`
	// InitParamScaleVar / InitParamAngleVar は倍率・角度補正の初期分散。
	InitParamScaleVar float64 `json:"initParamScaleVar"`
	InitParamAngleVar float64 `json:"initParamAngleVar"`

	// EnableGyro はジャイロが届いたときに使うか。
	//
	// **届かない機体でもそのまま動く。** ImuSample.HasGyro が false なら
	// 観測を単に使わない (計画 §7.5 の「定義されていないフィールドは
	// センサ非搭載として扱う」と同じ方針)。
	//
	// ジャイロは「予測の入力」にするのが TIGERs / PX4 / RAVEN 側の調査の結論だが、
	// ここでは **omega の観測**として入れる。角加速度のプロセス雑音
	// (AngAccelNoise 20 rad/s^2/sqrt(Hz) = 1 周期で 1.8 rad/s) がジャイロ雑音
	// (0.037 rad/s、RoboTeam Twente の実測) より 2 桁大きいので、推定される
	// omega は実質ジャイロそのものになり、入力にした場合とほぼ一致する。
	// **かつ、ジャイロが無い機体・落ちた瞬間にそのまま車輪へ戻れる。**
	EnableGyro bool `json:"enableGyro"`
	// GyroNoise はジャイロの観測雑音の標準偏差 [rad/s]。
	//
	// 実測の出典は RoboTeam Twente ETDP 2020 (Xsens MTi-3): 静止 0.0011、
	// **走行中 0.037**。走行中の値を取る。本機の LSM6DSO32 では未計測。
	GyroNoise float64 `json:"gyroNoise"`
	// GyroBiasNoise はジャイロバイアスのランダムウォーク雑音密度 [rad/s/sqrt(s)]。**未計測**。
	// Allan 分散で測ること (計画 §8)。
	GyroBiasNoise float64 `json:"gyroBiasNoise"`
	// InitGyroBiasVar はジャイロバイアスの初期分散 [rad^2/s^2]。
	InitGyroBiasVar float64 `json:"initGyroBiasVar"`

	// AccelCollisionThreshold は衝突とみなすロボット系加速度の大きさ [m/s^2]。
	//
	// **加速度計は速度の予測に使わない。** RoboTeam Twente の実測で走行中の
	// 標準偏差が 2.5 m/s^2 (静止時の 100 倍) あり、「state estimation で頼れる
	// 精度ではない」と結論されている。衝突検出にだけ使い、閾値を超えたら
	// AccelCollisionBoost の間だけプロセス雑音を膨らませる。
	// 0 以下で無効。
	AccelCollisionThreshold float64 `json:"accelCollisionThreshold"`
	// AccelCollisionBoost は衝突検出時にプロセス雑音へ掛ける倍率。
	AccelCollisionBoost float64 `json:"accelCollisionBoost"`

	// EnableZupt は停止中の疑似観測 (v = 0, omega = 0) を使うか。
	//
	// 判定は 3 条件の AND (研究 §4.5):
	// 4 輪速がすべて閾値以下、直近の vision の動きが閾値以下、冗長残差が正常。
	EnableZupt bool `json:"enableZupt"`
	// ZuptWheelThreshold は停止とみなす車輪角速度の上限 [rad/s]。
	//
	// **実測の静止時の車輪雑音 (0.23 rad/s) より大きく取らないと、永遠に成立しない。**
	// 最初 0.15 に置いたが、実機ログでは ZUPT が 1 度も発動しなかった。
	// ここは粗い足切りと割り切り、**実際の判定は vision の見かけ速度**
	// (100 ms の基線で 20 mm/s) と冗長残差に任せる。
	ZuptWheelThreshold float64 `json:"zuptWheelThreshold"`
	// ZuptVisionSpeedThreshold は停止とみなす vision の見かけの速度の上限 [m/s]。
	//
	// **vision が新しいことを条件に入れるのが肝。** 車輪の読みが壊れて 0 のまま
	// 機体が動く事故 (Trajectory POC Log §5-18) では、車輪だけを見た停止判定が
	// 推定を固めてしまう。
	ZuptVisionSpeedThreshold float64 `json:"zuptVisionSpeedThreshold"`
	// ZuptVelNoise / ZuptOmegaNoise は疑似観測の標準偏差 [m/s] / [rad/s]。
	// 小さいほど強く効く。誤検出が致命的なので、ゼロにはしない。
	ZuptVelNoise   float64 `json:"zuptVelNoise"`
	ZuptOmegaNoise float64 `json:"zuptOmegaNoise"`

	// HuberC は Huber のしきい値 (正規化イノベーション)。既定 2.0。
	//
	// これを超えたら重みを c/||nu|| へ落とす。カイ二乗のハードゲートは使わない。
	// 一度弾き始めると共分散が縮んだまま復帰できない失敗モードがあるため (計画 §4.4)。
	HuberC float64 `json:"huberC"`

	// InitPosVar / InitAngVar / InitVelVar / InitOmegaVar / InitSlipVar は初期共分散。
	InitPosVar   float64 `json:"initPosVar"`
	InitAngVar   float64 `json:"initAngVar"`
	InitVelVar   float64 `json:"initVelVar"`
	InitOmegaVar float64 `json:"initOmegaVar"`
	InitSlipVar  float64 `json:"initSlipVar"`
}

// DefaultNoise は既定の雑音設定を返す。
//
// vision と車輪は**実機の実測値**、それ以外は未計測のまま。
func DefaultNoise() NoiseConfig {
	return NoiseConfig{
		AccelNoise:    4.0,
		AngAccelNoise: 5.0,

		// スリップの既定は合成データの掃引で選んだ (locsim の TestSlipParameterSweep)。
		//
		// tau x Q を 12 通り、スリップ無し / バースト / バースト+欠落 /
		// 定常スリップ+欠落 の 4 シナリオで比べ、**スリップのある全シナリオで
		// 過信 (NEES が上側へ外れるサンプル) がゼロになる最も安価な設定**を採った。
		// 過信を基準にしたのは、ずれているのに自信満々な状態が位置制御にとって
		// 一番危ないため。
		//
		//	tau 0.10 / Q 1.0   スリップ無しで 6.85 mm、全スリップ条件で過信 0
		//	tau 0.05 / Q 1.0   スリップ無しで 5.74 mm だが定常スリップで過信 174/1126
		//	tau 0.30 / Q 1.0   過信 0 だがスリップ無しのコストが 7.87 mm と最大
		//	OFF                スリップ無しで 2.65 mm。ただし定常スリップ時 NEES 175
		//
		// スリップを切ると、スリップ中に 124 mm ずれながら共分散は締まったまま
		// という最悪の振る舞いになる。**実機の雑音で測り直すこと。**
		SlipNoise:     1.0,
		SlipTau:       0.1,
		EnableSlip:    true,
		SlipRScaleRef: 0.1,
		SlipRScaleMax: 10,

		// **実機ログからの実測値** (2026-09-23、有効な 11 本・8749 サンプル)。
		//
		// 冗長残差 n^T w (単位ノルム) を車輪速度で 8 分位に分けて広がりを測った:
		//
		//	車輪速度 rms [rad/s]   残差 rms [rad/s]
		//	         0.31              0.229
		//	         3.95              0.448
		//	         9.19              0.884
		//	        12.52              1.127
		//
		// 切片が 0.23、比例分が 0.07 前後。PoC が経験的に置いた「0.3 rad/s」と
		// 同じ桁で、そこをまたぐ形になっている。
		//
		// **測ったのは車輪 0.3..12.5 rad/s (機体 0.35 m/s まで) の範囲だけである。**
		// 試合の 2-3 m/s は車輪 70-107 rad/s で、この式はそこでは外挿になる。
		// 比例分をそのまま伸ばすと 5-7 rad/s の雑音になり、車輪がほとんど効かなくなる。
		// **速い走りのログを撮って測り直すこと** (研究 §6.5)。
		//
		// **切片の 0.23 rad/s は素の量子化 (0.0029) の 80 倍あり、センサ雑音ではない。**
		// PoC §5-16 が実測した「止まり際に 4 輪そろって 2.2..3.0 rad/s 逆回りする
		// STM の速度制御の行き過ぎ」と、走行中の +-70 mm/s の揺れがここに入っている。
		// つまり**車輪は本当にそう回っている**ので、観測雑音として扱うのが正しい。
		// 比例分の根拠は研究 §3.7 (オムニ車輪の polygon 効果) + 同じ制御のリップル。
		WheelNoise:          0.23,
		WheelNoiseSpeedCoef: 0.07,

		// 実測 (Trajectory POC Log §5-11)。
		VisionPosNoise: 0.0004,
		VisionAngNoise: 0.32 * math.Pi / 180,

		VisionTimeSigma: 0.002,

		AdaptiveVisionR:    true,
		AdaptiveForgetting: 0.98,
		AdaptiveMinScale:   0.25,
		AdaptiveMaxScale:   100,

		EnableParamEstimation: true,
		ParamScaleNoise:       0.002,
		ParamAngleNoise:       0.002,
		ParamScaleLimit:       0.25,
		ParamAngleLimitDeg:    10,
		InitParamScaleVar:     0.01,  // 標準偏差 10%
		InitParamAngleVar:     0.012, // 標準偏差 6.3 度

		// ジャイロは届けば使う。届かない機体では観測が来ないので無効も同然。
		EnableGyro:      true,
		GyroNoise:       0.037,
		GyroBiasNoise:   0.001,
		InitGyroBiasVar: 0.01, // 標準偏差 0.1 rad/s = 5.7 deg/s

		AccelCollisionThreshold: 15.0,
		AccelCollisionBoost:     25.0,

		EnableZupt:               true,
		ZuptWheelThreshold:       0.7, // 実測の静止時雑音 0.23 rad/s の 3 シグマ
		ZuptVisionSpeedThreshold: 0.02,
		ZuptVelNoise:             0.002,
		ZuptOmegaNoise:           0.01,

		HuberC: 2.0,

		InitPosVar:   1.0,
		InitAngVar:   math.Pi * math.Pi,
		InitVelVar:   4.0,
		InitOmegaVar: 100.0,
		InitSlipVar:  0.25,
	}
}

// Validate は雑音設定が使える値かを調べる。
func (n *NoiseConfig) Validate() error {
	checks := []struct {
		name string
		v    float64
	}{
		{"accelNoise", n.AccelNoise},
		{"angAccelNoise", n.AngAccelNoise},
		{"wheelNoise", n.WheelNoise},
		{"visionPosNoise", n.VisionPosNoise},
		{"visionAngNoise", n.VisionAngNoise},
		{"huberC", n.HuberC},
		{"initPosVar", n.InitPosVar},
		{"initAngVar", n.InitAngVar},
		{"initVelVar", n.InitVelVar},
		{"initOmegaVar", n.InitOmegaVar},
	}
	for _, c := range checks {
		if c.v <= 0 || math.IsNaN(c.v) || math.IsInf(c.v, 0) {
			return fmt.Errorf("%s must be positive and finite, got %v", c.name, c.v)
		}
	}
	if n.WheelNoiseSpeedCoef < 0 || math.IsNaN(n.WheelNoiseSpeedCoef) {
		return fmt.Errorf("wheelNoiseSpeedCoef must be non-negative, got %v", n.WheelNoiseSpeedCoef)
	}
	if n.EnableSlip {
		if n.SlipTau <= 0 || math.IsNaN(n.SlipTau) {
			// Yu ほか (arXiv:2209.15140) の可観測性解析: alpha = 1/tau > 0 でないと
			// スリップは不可観測になる。
			return fmt.Errorf("slipTau must be positive when slip is enabled, got %v", n.SlipTau)
		}
		if n.SlipNoise <= 0 || n.InitSlipVar <= 0 {
			return fmt.Errorf("slipNoise and initSlipVar must be positive when slip is enabled")
		}
		if n.SlipRScaleRef > 0 && n.SlipRScaleMax < 1 {
			return fmt.Errorf("slipRScaleMax must be >= 1 when slipRScaleRef is set, got %v", n.SlipRScaleMax)
		}
	}
	if n.VisionTimeSigma < 0 || math.IsNaN(n.VisionTimeSigma) {
		return fmt.Errorf("visionTimeSigma must be non-negative, got %v", n.VisionTimeSigma)
	}
	if n.AdaptiveVisionR {
		if n.AdaptiveForgetting <= 0 || n.AdaptiveForgetting > 1 {
			return fmt.Errorf("adaptiveForgetting must be in (0, 1], got %v", n.AdaptiveForgetting)
		}
		if n.AdaptiveMinScale <= 0 || n.AdaptiveMaxScale < n.AdaptiveMinScale {
			return fmt.Errorf("adaptive R scale bounds are invalid: [%v, %v]", n.AdaptiveMinScale, n.AdaptiveMaxScale)
		}
	}
	if n.EnableParamEstimation {
		if n.ParamScaleNoise < 0 || n.ParamAngleNoise < 0 {
			return fmt.Errorf("param noise densities must be non-negative")
		}
		if n.ParamScaleLimit <= 0 || n.ParamScaleLimit >= 1 {
			return fmt.Errorf("paramScaleLimit must be in (0, 1), got %v", n.ParamScaleLimit)
		}
		if n.ParamAngleLimitDeg <= 0 {
			return fmt.Errorf("paramAngleLimitDeg must be positive, got %v", n.ParamAngleLimitDeg)
		}
		if n.InitParamScaleVar <= 0 || n.InitParamAngleVar <= 0 {
			return fmt.Errorf("initial parameter variances must be positive")
		}
	}
	if n.EnableGyro {
		if n.GyroNoise <= 0 || math.IsNaN(n.GyroNoise) {
			return fmt.Errorf("gyroNoise must be positive, got %v", n.GyroNoise)
		}
		if n.GyroBiasNoise < 0 || n.InitGyroBiasVar <= 0 {
			return fmt.Errorf("gyro bias noise settings are invalid")
		}
	}
	if n.AccelCollisionThreshold > 0 && n.AccelCollisionBoost < 1 {
		return fmt.Errorf("accelCollisionBoost must be >= 1, got %v", n.AccelCollisionBoost)
	}
	if n.EnableZupt {
		if n.ZuptWheelThreshold <= 0 {
			return fmt.Errorf("zuptWheelThreshold must be positive, got %v", n.ZuptWheelThreshold)
		}
		if n.ZuptVisionSpeedThreshold <= 0 {
			return fmt.Errorf("zuptVisionSpeedThreshold must be positive, got %v", n.ZuptVisionSpeedThreshold)
		}
		if n.ZuptVelNoise <= 0 || n.ZuptOmegaNoise <= 0 {
			return fmt.Errorf("zupt pseudo-measurement noise must be positive")
		}
	}
	return nil
}

// Config は推定器の設定一式。
type Config struct {
	Geometry GeometryConfig `json:"geometry"`
	Noise    NoiseConfig    `json:"noise"`
}

// DefaultConfig は暫定の設定を返す。**機体パラメータも雑音も未確定である。**
func DefaultConfig() Config {
	return Config{Geometry: DefaultGeometry(), Noise: DefaultNoise()}
}

// Validate は設定全体を検査する。
func (c *Config) Validate() error {
	if err := c.Geometry.Validate(); err != nil {
		return err
	}
	return c.Noise.Validate()
}
