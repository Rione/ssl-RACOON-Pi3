package locsim

import (
	"fmt"
	"math"
	"math/rand"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/localization"
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
	// WheelNoiseRadS は車輪角速度の白色雑音の標準偏差 [rad/s]。**未計測**。
	WheelNoiseRadS float64
	// WheelQuantRadS は量子化幅 [rad/s]。SPI は rad/s x 100 の int16 なので 0.01。
	WheelQuantRadS float64
	// WheelTimeOffset は転送時刻に対するサンプル時刻のずれ。通常は負。**未計測** (D-5)。
	WheelTimeOffset time.Duration
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

// DefaultConfig は暫定のセンサ設定を返す。
//
// **どの数値も未計測である。** 実測で置き換えるまで、これで出した精度を
// 実機の保証と取り違えないこと。
func DefaultConfig() Config {
	return Config{
		Geometry: localization.DefaultGeometry(),

		WheelRate: 8 * time.Millisecond,
		// 量子化 (0.01 rad/s) の 1/3 程度を白色雑音として置く。根拠は無い。
		WheelNoiseRadS:  0.003,
		WheelQuantRadS:  0.01,
		WheelTimeOffset: -4 * time.Millisecond,

		VisionRate: time.Second / 60,
		// SSL-Vision の静止時分散は実測していない。5 mm / 0.5 度は一般的な桁として置いた値。
		VisionNoiseM:   0.005,
		VisionNoiseRad: 0.5 * math.Pi / 180,
		VisionMinDelay: 20 * time.Millisecond,
		VisionJitter:   5 * time.Millisecond,
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
	s.Vision = generateVision(tr, cfg, rng)
	return s, nil
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
		// 車輪サンプルの時刻は転送時刻ではなく、STM が読んだ推定時刻。
		ws.Stamp = localization.Stamp(t).Add(cfg.WheelTimeOffset)
		for slot := 0; slot < localization.NumWheels; slot++ {
			// 実機の Pi が受け取るのはスロット順なので、真の対応で並べ替える。
			v := logical[cfg.Geometry.WheelSlotOrder[slot]]
			if cfg.WheelNoiseRadS > 0 {
				v += rng.NormFloat64() * cfg.WheelNoiseRadS
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
