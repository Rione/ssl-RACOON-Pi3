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

// DefaultGeometry は暫定の機体パラメータを返す。
//
// 実機で動いている新世代ファームウェア (MainBoard_V26_1_1) の parammeter.h に
// 合わせてあるが、あくまで 4 候補のうちの 1 つにすぎない。符号は計画 §4.5 の
// 基準形 (旧世代 STM / RAVEN と同じ) を既定にしてある。
//
// この値のまま §10 の精度目標を評価してはならない。
func DefaultGeometry() GeometryConfig {
	const r = 0.030 // m。候補: 26 / 27 / 30 mm
	return GeometryConfig{
		WheelAnglesDeg: [NumWheels]float64{55, 135, -135, -55},
		WheelRadiusM:   [NumWheels]float64{r, r, r, r},
		MomentArmM:     0.075, // 候補: 75 / 85 / 89 / 90 mm
		WheelSlotOrder: [NumWheels]int{WheelFL, WheelBL, WheelBR, WheelFR},
		WheelSigns:     [NumWheels]float64{1, 1, 1, 1},
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
// **どの値も未計測のプレースホルダである。** 実測は計画 §8 / §12-D で埋まる。
// 合成データに合わせ込んではいけない。合成データの雑音は実機の雑音ではないので、
// ここを合わせ込むと後で剥がせない嘘になる。
type NoiseConfig struct {
	// AccelNoise は並進の加速度雑音密度 [m/s^2/sqrt(Hz)]。**未計測**。
	//
	// 加速度計が無いので予測は定速度モデルで、実際の加速はすべてこの雑音で
	// 表現される。SSL のロボットは 3-6 m/s^2 で加減速するので、その桁を置く。
	AccelNoise float64 `json:"accelNoise"`

	// AngAccelNoise は角加速度の雑音密度 [rad/s^2/sqrt(Hz)]。**未計測**。
	AngAccelNoise float64 `json:"angAccelNoise"`

	// SlipNoise はスリップ状態の駆動雑音密度 [m/s^2/sqrt(Hz)]。**未計測**。
	//
	// IMU が無い間、車輪観測には v と s が和としてしか現れない。分離しているのは
	// vision の位置観測と、この雑音および SlipTau が作る過程モデルの違いだけである。
	// つまり**測れない定数がスリップ推定の結果を独占的に決める**。
	SlipNoise float64 `json:"slipNoise"`

	// SlipTau はスリップの一次減衰の時定数 [s]。**未計測**。計画 §4.3 の既定は 0.1 s。
	//
	// 放置すると恒久バイアスとして誤学習するので、必ず 0 へ引き戻す。
	SlipTau float64 `json:"slipTau"`

	// EnableSlip はスリップ状態を使うか。false なら s は常にゼロに固定される。
	//
	// スリップを入れた場合と入れない場合を同じログで比べるための切り替え。
	// 効果を測れない設定を既定にしないために要る。
	EnableSlip bool `json:"enableSlip"`

	// GyroNoise はジャイロのヨーレートの観測雑音の標準偏差 [rad/s]。**未計測**。
	// LSM6DSO32 の分解能は 1/900 rad/s なので、量子化だけなら 0.0003 程度。実際は振動が乗る。
	GyroNoise float64 `json:"gyroNoise"`

	// GyroBiasNoise はジャイロのバイアスのランダムウォークの強さ [rad/s / sqrt(s)]。**未計測**。
	// 温度でゆっくり動く分を吸収する。大きすぎると omega の誤差をバイアスが食ってしまう。
	GyroBiasNoise float64 `json:"gyroBiasNoise"`

	// InitGyroBiasVar はジャイロのバイアスの初期分散 [(rad/s)^2]。
	// STM 側が起動時に静止で引いているので、残りは小さいはず。
	InitGyroBiasVar float64 `json:"initGyroBiasVar"`

	// WheelNoise は車輪角速度の観測雑音の標準偏差 [rad/s]。**未計測**。
	//
	// SPI は rad/s x 100 の int16 なので量子化だけで 0.01/sqrt(12) = 0.003 ある。
	WheelNoise float64 `json:"wheelNoise"`

	// VisionPosNoise / VisionAngNoise は vision の観測雑音の標準偏差 [m] / [rad]。**未計測** (D-7)。
	VisionPosNoise float64 `json:"visionPosNoise"`
	VisionAngNoise float64 `json:"visionAngNoise"`

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

// DefaultNoise は暫定の雑音設定を返す。**どの値も未計測である。**
func DefaultNoise() NoiseConfig {
	return NoiseConfig{
		AccelNoise:    4.0,
		AngAccelNoise: 20.0,

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
		SlipNoise:  1.0,
		SlipTau:    0.1,
		EnableSlip: true,

		WheelNoise: 0.01,

		GyroNoise:       0.01,
		GyroBiasNoise:   0.001,
		InitGyroBiasVar: 0.01 * 0.01,

		VisionPosNoise: 0.005,
		VisionAngNoise: 0.5 * math.Pi / 180,

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
		{"gyroNoise", n.GyroNoise},
		{"gyroBiasNoise", n.GyroBiasNoise},
		{"initGyroBiasVar", n.InitGyroBiasVar},
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
	if n.EnableSlip {
		if n.SlipTau <= 0 || math.IsNaN(n.SlipTau) {
			return fmt.Errorf("slipTau must be positive when slip is enabled, got %v", n.SlipTau)
		}
		if n.SlipNoise <= 0 || n.InitSlipVar <= 0 {
			return fmt.Errorf("slipNoise and initSlipVar must be positive when slip is enabled")
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
