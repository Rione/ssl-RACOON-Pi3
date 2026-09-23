package locsim

import (
	"fmt"
	"math"
	"math/rand"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// 真値からセンサ列を合成する。
//
// **雑音・遅延・欠落の数値はどれも未計測のプレースホルダである。** 実測は P1 の
// D-1〜D-7 で埋まる。合成データでフィルタを「良く見せる」ためにここを触らないこと。
// 合成データの雑音は実機の雑音ではないので、ここに Q / R を合わせ込むと
// 後で剥がせない嘘になる。

// Outage は vision を人為的に落とす区間。
// 計画 §10-4 (0.5 秒後 <= 30 mm) と §10-5 (2.0 秒後 <= 150 mm) の検証に使う。
type Outage struct {
	Start time.Duration
	End   time.Duration
}

func (o Outage) contains(t time.Duration) bool { return t >= o.Start && t < o.End }

// Config はセンサ合成の設定。
type Config struct {
	// Geometry は**真の**機体パラメータ。
	//
	// フィルタにこれと違う設定を与えれば、符号やホイール順序を取り違えたときに
	// どれだけ悪化するかを測れる。計画 §12-A4 / A-5 の検証はそれで行う。
	Geometry localization.GeometryConfig

	// --- 車輪 ---

	// WheelRate は車輪サンプルの周期。既定 8 ms (SPI の公称周期)。
	WheelRate time.Duration
	// WheelNoiseRadS は車輪角速度の白色雑音のうち、速度に依存しない分の
	// 標準偏差 [rad/s]。
	WheelNoiseRadS float64
	// WheelNoiseSpeedCoef は車輪角速度に比例する雑音の係数 [無次元]。
	//
	// オムニ車輪の有効転がり半径はローラの入れ替わりで変動する (polygon 効果)。
	// 本機はローラ 15 個 x 2 列・角ピッチ 12 度なので、0.4 m/s では約 68 Hz の
	// リップルになり、125 Hz のサンプリングでは白色として扱える
	// (docs/self-localization-research-20260923.md §3.7)。
	WheelNoiseSpeedCoef float64
	// WheelQuantRadS は量子化幅 [rad/s]。SPI は rad/s x 100 の int16 なので 0.01。
	WheelQuantRadS float64
	// WheelTimeOffset は転送時刻に対するサンプル時刻のずれ。通常は負。
	//
	// サンプルの刻印はサンプル時刻そのもので、転送 (= Pi に届く時刻) は
	// stamp - WheelTimeOffset になる。vision との到着順を決めるのに使う。
	WheelTimeOffset time.Duration
	// --- IMU (STM に載った場合) ---

	// HasGyro が true なら車輪と同じ周期でジャイロを合成する。
	//
	// **既定は false。** 現行の STM は IMU を送ってこない (計画 §3.5)。
	// ssl-Circuit の FW/MainBoard_V26_2 に LSM6DSO32 を読む実装があるので、
	// 載ったときに効果を測れるようにしてある。
	HasGyro bool
	// GyroNoiseRadS はジャイロの白色雑音の標準偏差 [rad/s]。
	// RoboTeam Twente の実測 (Xsens MTi-3) は走行中 0.037 rad/s。
	GyroNoiseRadS float64
	// GyroBiasRadS は一定のバイアス [rad/s]。推定器がこれを当てられるかを見る。
	GyroBiasRadS float64

	// Slip はロボット系のスリップ速度 [m/s] を返す。nil ならスリップ無し。
	//
	// 車輪は「実際の運動 + スリップ」を見る。地面に対して滑っているぶん、
	// 車輪の回転は機体の実際の移動より速くなる。
	Slip func(t time.Duration) localization.Vec2

	// --- vision ---

	// VisionRate は vision の周期。既定 1/60 秒。
	VisionRate time.Duration
	// VisionNoiseM / VisionNoiseRad は観測雑音の標準偏差。**未計測** (D-7)。
	VisionNoiseM   float64
	VisionNoiseRad float64
	// VisionMinDelay / VisionJitter は片方向遅延。**未計測** (D-3)。
	VisionMinDelay time.Duration
	VisionJitter   time.Duration
	// VisionLossRate はマルチキャストの欠落率 [0, 1]。**未計測** (D-1)。
	VisionLossRate float64
	// VisionOutages は人為的に vision を落とす区間。
	VisionOutages []Outage
	// VisionTimeBias は timesync が取り除けずに残る時刻の系統誤差。
	//
	// 片方向観測だけでは「クロックオフセット」と「最小片方向遅延」を分離できない
	// ので、この定数分は必ず残る (計画 §5.3)。既定は VisionMinDelay と同じ。
	VisionTimeBias time.Duration
	// Cameras はカメラ台数。1 以上。frame_number はカメラごとに採番する。
	Cameras int
}

func (c *Config) withDefaults() {
	if c.WheelRate <= 0 {
		c.WheelRate = 8 * time.Millisecond
	}
	if c.WheelQuantRadS < 0 {
		c.WheelQuantRadS = 0
	}
	if c.VisionRate <= 0 {
		c.VisionRate = time.Second / 60
	}
	if c.Cameras <= 0 {
		c.Cameras = 1
	}
}

// DefaultConfig は既定のセンサ設定を返す。
//
// **2026-09-23 に実機の実測値へ置き換えた。** 出典は Trajectory POC Log
// (軌道追従 PoC、テスト機 racoon-56011、SSL-Vision カメラ 1 台)。
// 合成データでフィルタを良く見せるためにここを触らないこと。
// **フィルタを合成データへ合わせ込むのではなく、合成データを実機へ合わせる。**
func DefaultConfig() Config {
	return Config{
		Geometry: localization.DefaultGeometry(),

		WheelRate: 8 * time.Millisecond,
		// **実機ログからの実測** (2026-09-23、11 本 8749 サンプル):
		// 冗長残差の広がりは 車輪 0.3 rad/s で 0.23、12.5 rad/s で 1.13。
		// 切片 0.23 + 比例 0.07。切片には STM の速度制御の行き過ぎが入っている
		// (PoC §5-16)。詳しい根拠は localization.DefaultNoise のコメント。
		WheelNoiseRadS:      0.23,
		WheelNoiseSpeedCoef: 0.07,
		WheelQuantRadS:      0.01,
		// 実測: 車輪と vision の速度が最も合うのは車輪を 8..12 ms 古いとしたとき
		// (PoC §5-17)。SPI の 1 周期 = 8 ms と整合する。
		WheelTimeOffset: -10 * time.Millisecond,

		// 実測: 自機の更新は約 116 Hz (PoC §1)。
		VisionRate: time.Second / 116,
		// 実測: 位置の細かい揺れ 0.2..0.4 mm RMS、向き 0.25..0.32 度 (PoC §5-11)。
		// **短期の再現性であって絶対精度ではない** (研究 §4.6 の注)。
		VisionNoiseM:   0.0004,
		VisionNoiseRad: 0.32 * math.Pi / 180,
		VisionMinDelay: 20 * time.Millisecond,
		VisionJitter:   5 * time.Millisecond,
		// 実測: timesync の写像に足りない片道遅延は 0..4 ms (PoC §5-17)。
		// **計画 §5.3 が「原理的に分離できない」とした定数は、車輪という独立な
		// 速度源があれば相関で測れる。** 以前の既定 20 ms は測る前の置き値だった。
		VisionTimeBias: 2 * time.Millisecond,
		// 実測: 最大の途切れ 17..34 ms (= 2..4 フレーム) (PoC §5-11)。
		VisionLossRate: 0.02,
		Cameras:        1,
	}
}

// Sensors は合成したセンサ列一式。
type Sensors struct {
	// Truth は WheelRate 刻みの真値。スリップを適用済み。
	Truth []Truth
	// Wheels は SPI フレーム上の並び (FL, BL, BR, FR) の角速度。
	//
	// 論理輪番号ではなくスロット順なのは、実機で Pi が受け取るのがこれだから。
	// フィルタが wheelSlotOrder を取り違えていれば、ここで食い違う。
	Wheels []localization.WheelSample
	// Imu はジャイロ・加速度。Config.HasGyro が false なら空。
	Imu []localization.ImuSample
	// Vision は自機の観測。欠落ぶんは含まれない。
	Vision []localization.VisionPose
	// Config は**既定値を埋めた後の**設定。
	//
	// Generate は引数の Config をコピーしてから既定値を埋めるので、
	// 呼び出し側が渡した変数には反映されない。解決後の値はここから読むこと。
	Config Config
}

// TruthAt は時刻 t に最も近い真値を返す。評価のときに使う。
func (s *Sensors) TruthAt(stamp localization.Stamp) (Truth, bool) {
	if len(s.Truth) == 0 {
		return Truth{}, false
	}
	dt := localization.Stamp(s.Config.WheelRate)
	i := int(stamp / dt)
	if i < 0 || i >= len(s.Truth) {
		return Truth{}, false
	}
	// 線形補間。真値は dt 刻みでしか持っていないが、評価はその間の時刻でも起きる。
	if i+1 >= len(s.Truth) {
		return s.Truth[i], true
	}
	a, b := s.Truth[i], s.Truth[i+1]
	span := float64(b.Stamp - a.Stamp)
	if span <= 0 {
		return a, true
	}
	f := float64(stamp-a.Stamp) / span
	return Truth{
		Stamp: stamp,
		Pose: localization.Pose2{
			X:     a.Pose.X + f*(b.Pose.X-a.Pose.X),
			Y:     a.Pose.Y + f*(b.Pose.Y-a.Pose.Y),
			Theta: localization.WrapAngle(a.Pose.Theta + f*localization.AngleDiff(b.Pose.Theta, a.Pose.Theta)),
		},
		VelBody:  localization.Vec2{X: a.VelBody.X + f*(b.VelBody.X-a.VelBody.X), Y: a.VelBody.Y + f*(b.VelBody.Y-a.VelBody.Y)},
		YawRate:  a.YawRate + f*(b.YawRate-a.YawRate),
		SlipBody: localization.Vec2{X: a.SlipBody.X + f*(b.SlipBody.X-a.SlipBody.X), Y: a.SlipBody.Y + f*(b.SlipBody.Y-a.SlipBody.Y)},
	}, true
}

// Generate は真値軌道からセンサ列を合成する。
//
// seed を固定すれば必ず同じ列が出る。回帰テストの前提。
func Generate(tr Trajectory, cfg Config, seed int64) (*Sensors, error) {
	cfg.withDefaults()
	if cfg.VisionTimeBias == 0 {
		cfg.VisionTimeBias = cfg.VisionMinDelay
	}
	k, err := localization.NewKinematics(cfg.Geometry)
	if err != nil {
		return nil, fmt.Errorf("locsim: %w", err)
	}

	s := &Sensors{Config: cfg}
	rng := rand.New(rand.NewSource(seed))

	s.Truth, s.Wheels = generateWheels(tr, cfg, k, rng)
	if cfg.HasGyro {
		s.Imu = generateImu(tr, cfg, rng)
	}
	s.Vision = generateVision(tr, cfg, rng)
	return s, nil
}

// generateImu は車輪と同じ周期でジャイロを合成する。
//
// 加速度計は合成しない。走行中の実測雑音が 2.5 m/s^2 と大きく、推定には
// 使わないと決めてあるため (研究 §3.1 / §4.7)。衝突検出の試験は別途。
func generateImu(tr Trajectory, cfg Config, rng *rand.Rand) []localization.ImuSample {
	n := int(tr.Duration()/cfg.WheelRate) + 1
	out := make([]localization.ImuSample, 0, n)
	for i := 0; i < n; i++ {
		t := time.Duration(i) * cfg.WheelRate
		tv := tr.At(t)
		z := tv.YawRate + cfg.GyroBiasRadS
		if cfg.GyroNoiseRadS > 0 {
			z += rng.NormFloat64() * cfg.GyroNoiseRadS
		}
		out = append(out, localization.ImuSample{
			Stamp:   localization.Stamp(t),
			GyroZ:   z,
			HasGyro: true,
		})
	}
	return out
}

func generateWheels(tr Trajectory, cfg Config, k *localization.Kinematics, rng *rand.Rand) ([]Truth, []localization.WheelSample) {
	n := int(tr.Duration()/cfg.WheelRate) + 1
	truth := make([]Truth, 0, n)
	wheels := make([]localization.WheelSample, 0, n)

	for i := 0; i < n; i++ {
		t := time.Duration(i) * cfg.WheelRate
		tv := tr.At(t)
		if cfg.Slip != nil {
			tv.SlipBody = cfg.Slip(t)
		}
		truth = append(truth, tv)

		// 車輪は「機体の運動 + スリップ」を見る。滑っているぶん回転が速くなる。
		logical := k.WheelFromBody(
			tv.VelBody.X+tv.SlipBody.X,
			tv.VelBody.Y+tv.SlipBody.Y,
			tv.YawRate,
		)

		var ws localization.WheelSample
		// **刻印は、その値を取った時刻そのもの。**
		//
		// 以前はここで WheelTimeOffset を足していたが、値は tr.At(t) から
		// 作っているので「t の運動に t+offset の刻印を付ける」ことになり、
		// 合成データ自身に offset ぶんの時刻誤差が入っていた。
		// WheelTimeOffset が表すのは「転送時刻に対してサンプルがどれだけ古いか」で、
		// これは RunFilter が到着時刻を決めるのに使う (stamp - offset が転送時刻)。
		ws.Stamp = localization.Stamp(t)
		for slot := 0; slot < localization.NumWheels; slot++ {
			// 実機の Pi が受け取るのはスロット順なので、真の対応で並べ替える。
			v := logical[cfg.Geometry.WheelSlotOrder[slot]]
			sigma2 := cfg.WheelNoiseRadS * cfg.WheelNoiseRadS
			if cfg.WheelNoiseSpeedCoef > 0 {
				sp := cfg.WheelNoiseSpeedCoef * v
				sigma2 += sp * sp
			}
			if sigma2 > 0 {
				v += rng.NormFloat64() * math.Sqrt(sigma2)
			}
			if cfg.WheelQuantRadS > 0 {
				v = math.Round(v/cfg.WheelQuantRadS) * cfg.WheelQuantRadS
			}
			ws.Omega[slot] = v
		}
		wheels = append(wheels, ws)
	}
	return truth, wheels
}

func generateVision(tr Trajectory, cfg Config, rng *rand.Rand) []localization.VisionPose {
	n := int(tr.Duration()/cfg.VisionRate) + 1
	out := make([]localization.VisionPose, 0, n)

	frameNumbers := make([]uint32, cfg.Cameras)
	for i := 0; i < n; i++ {
		capture := time.Duration(i) * cfg.VisionRate
		cam := uint32(i % cfg.Cameras)
		frameNumbers[cam]++

		if rng.Float64() < cfg.VisionLossRate {
			continue // マルチキャストの欠落
		}
		skip := false
		for _, o := range cfg.VisionOutages {
			if o.contains(capture) {
				skip = true
				break
			}
		}
		if skip {
			continue
		}

		tv := tr.At(capture)
		delay := cfg.VisionMinDelay
		if cfg.VisionJitter > 0 {
			// 片方向遅延は非負で上側に長い裾を持つ (計画 §5.3)。
			delay += time.Duration(rng.ExpFloat64() * float64(cfg.VisionJitter))
		}

		out = append(out, localization.VisionPose{
			// Stamp は timesync 通過後の時刻。分離できない定数分だけずれて届く。
			Stamp:   localization.Stamp(capture).Add(cfg.VisionTimeBias),
			Arrival: localization.Stamp(capture + delay),
			Pose: localization.Pose2{
				X:     tv.Pose.X + rng.NormFloat64()*cfg.VisionNoiseM,
				Y:     tv.Pose.Y + rng.NormFloat64()*cfg.VisionNoiseM,
				Theta: localization.WrapAngle(tv.Pose.Theta + rng.NormFloat64()*cfg.VisionNoiseRad),
			},
			Confidence:  1,
			CameraID:    cam,
			FrameNumber: frameNumbers[cam],
			Mapped:      true,
		})
	}
	return out
}

// ConstantSlip は一定のスリップを返す Slip 関数を作る。
func ConstantSlip(v localization.Vec2) func(time.Duration) localization.Vec2 {
	return func(time.Duration) localization.Vec2 { return v }
}

// BurstSlip は区間 [start, end) の間だけスリップする Slip 関数を作る。
// 急加速や衝突で数十 ms だけ滑る状況を模す。
func BurstSlip(v localization.Vec2, start, end time.Duration) func(time.Duration) localization.Vec2 {
	return func(t time.Duration) localization.Vec2 {
		if t >= start && t < end {
			return v
		}
		return localization.Vec2{}
	}
}
