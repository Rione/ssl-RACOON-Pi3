package locsim

import (
	"math"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/localization"
)

func testTrajectory() Trajectory {
	// 並進と回転が両方入り、機体系の vy もゼロにならない軌道。
	return FigureEight(localization.Vec2{}, 1.5, 4*time.Second,
		HeadingParams{Mode: HeadingSpin, SpinRate: 1.2}, 8*time.Second)
}

// 真の機体パラメータで読み戻せば、真の body 速度が復元できること。
// ここが合わないとセンサ合成そのものが間違っている。
func TestWheelSamplesRecoverTrueBodyVelocity(t *testing.T) {
	cfg := DefaultConfig()
	cfg.WheelNoiseRadS = 0 // 量子化だけ残す
	s, err := Generate(testTrajectory(), cfg, 1)
	if err != nil {
		t.Fatal(err)
	}
	k, err := localization.NewKinematics(cfg.Geometry)
	if err != nil {
		t.Fatal(err)
	}

	var maxVel, maxOmega float64
	for i, w := range s.Wheels {
		vx, vy, om := k.BodyFromWheel(k.SlotsToLogical(w.Omega))
		truth := s.Truth[i]
		maxVel = math.Max(maxVel, math.Hypot(vx-truth.VelBody.X, vy-truth.VelBody.Y))
		maxOmega = math.Max(maxOmega, math.Abs(om-truth.YawRate))
	}
	// 量子化 0.01 rad/s x 車輪半径 30 mm = 0.3 mm/s 程度が下限。
	if maxVel > 0.002 {
		t.Errorf("worst velocity recovery error = %.4f m/s, want <= 0.002", maxVel)
	}
	if maxOmega > 0.02 {
		t.Errorf("worst yaw rate recovery error = %.4f rad/s, want <= 0.02", maxOmega)
	}
}

// **これが A-4 / A-5 の検出可能性そのもの。**
//
// 符号やホイール順序を取り違えた設定で読むと、復元される body 速度が
// どれだけ壊れるかを数字で出す。壊れないなら、実機で符号が逆でも気づけない。
func TestWrongGeometryBreaksWheelInterpretation(t *testing.T) {
	truthCfg := DefaultConfig()
	truthCfg.WheelNoiseRadS = 0
	s, err := Generate(testTrajectory(), truthCfg, 2)
	if err != nil {
		t.Fatal(err)
	}

	cases := []struct {
		name   string
		mutate func(*localization.GeometryConfig)
	}{
		{"sign flipped (plan A-5)", func(g *localization.GeometryConfig) {
			for i := range g.WheelSigns {
				g.WheelSigns[i] = -1
			}
		}},
		{"FL/FR swapped (plan A-4)", func(g *localization.GeometryConfig) {
			g.WheelSlotOrder = [localization.NumWheels]int{
				localization.WheelFR, localization.WheelBL, localization.WheelBR, localization.WheelFL,
			}
		}},
		{"wheel radius 27mm instead of 30mm (A-1)", func(g *localization.GeometryConfig) {
			for i := range g.WheelRadiusM {
				g.WheelRadiusM[i] = 0.027
			}
		}},
		{"moment arm 90mm instead of 75mm (A-2)", func(g *localization.GeometryConfig) {
			g.MomentArmM = 0.090
		}},
	}

	// 参照: 正しい設定での誤差。
	correct, _ := localization.NewKinematics(truthCfg.Geometry)
	refVel, refOmega := worstRecoveryError(s, correct)
	t.Logf("correct geometry:            vel %.4f m/s, omega %.4f rad/s", refVel, refOmega)

	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			g := localization.DefaultGeometry()
			c.mutate(&g)
			k, err := localization.NewKinematics(g)
			if err != nil {
				t.Fatal(err)
			}
			velErr, omegaErr := worstRecoveryError(s, k)
			t.Logf("%-42s vel %.4f m/s, omega %.4f rad/s", c.name+":", velErr, omegaErr)

			// 誤った設定は、正しい設定より桁違いに悪くなければならない。
			// そうでなければ、実機でその間違いに気づく手がかりが無い。
			if velErr < 10*refVel && omegaErr < 10*refOmega {
				t.Errorf("a wrong geometry produced errors comparable to the correct one "+
					"(vel %.4f vs %.4f, omega %.4f vs %.4f); this mistake would be invisible on the robot",
					velErr, refVel, omegaErr, refOmega)
			}
		})
	}
}

func worstRecoveryError(s *Sensors, k *localization.Kinematics) (vel, omega float64) {
	for i, w := range s.Wheels {
		vx, vy, om := k.BodyFromWheel(k.SlotsToLogical(w.Omega))
		truth := s.Truth[i]
		vel = math.Max(vel, math.Hypot(vx-truth.VelBody.X, vy-truth.VelBody.Y))
		omega = math.Max(omega, math.Abs(om-truth.YawRate))
	}
	return vel, omega
}

// スリップは「車輪だけが速く回る」形で現れること。
func TestSlipAppearsOnlyInWheels(t *testing.T) {
	cfg := DefaultConfig()
	cfg.WheelNoiseRadS = 0
	cfg.WheelQuantRadS = 0
	slip := localization.Vec2{X: 0.4, Y: -0.2}
	cfg.Slip = ConstantSlip(slip)

	s, err := Generate(testTrajectory(), cfg, 3)
	if err != nil {
		t.Fatal(err)
	}
	k, _ := localization.NewKinematics(cfg.Geometry)

	for i, w := range s.Wheels {
		vx, vy, _ := k.BodyFromWheel(k.SlotsToLogical(w.Omega))
		truth := s.Truth[i]
		// 車輪から見える速度 = 真の速度 + スリップ
		if math.Abs(vx-(truth.VelBody.X+slip.X)) > 1e-9 ||
			math.Abs(vy-(truth.VelBody.Y+slip.Y)) > 1e-9 {
			t.Fatalf("sample %d: wheels see (%v, %v), want (%v, %v)",
				i, vx, vy, truth.VelBody.X+slip.X, truth.VelBody.Y+slip.Y)
		}
		// 真値の姿勢はスリップの影響を受けない (滑っても機体は進まない)。
		if s.Truth[i].SlipBody != slip {
			t.Fatalf("sample %d: truth slip = %+v, want %+v", i, s.Truth[i].SlipBody, slip)
		}
	}
}

func TestBurstSlip(t *testing.T) {
	f := BurstSlip(localization.Vec2{X: 1}, 100*time.Millisecond, 200*time.Millisecond)
	if got := f(50 * time.Millisecond); got.X != 0 {
		t.Errorf("before the burst: %+v, want zero", got)
	}
	if got := f(150 * time.Millisecond); got.X != 1 {
		t.Errorf("during the burst: %+v, want 1", got)
	}
	if got := f(250 * time.Millisecond); got.X != 0 {
		t.Errorf("after the burst: %+v, want zero", got)
	}
}

func TestVisionLossRate(t *testing.T) {
	cfg := DefaultConfig()
	cfg.VisionLossRate = 0.2
	tr := Line(localization.Vec2{}, localization.Vec2{X: 1}, localization.Vec2{},
		HeadingParams{}, 60*time.Second)
	s, err := Generate(tr, cfg, 4)
	if err != nil {
		t.Fatal(err)
	}
	expected := int(tr.Duration() / cfg.VisionRate)
	got := float64(expected-len(s.Vision)) / float64(expected)
	if math.Abs(got-0.2) > 0.03 {
		t.Errorf("observed loss rate %.3f, want about 0.20", got)
	}
}

// 人為的な欠落区間を作れること。§10-4 / §10-5 の検証に要る。
func TestVisionOutages(t *testing.T) {
	cfg := DefaultConfig()
	cfg.VisionLossRate = 0
	cfg.VisionOutages = []Outage{{Start: 2 * time.Second, End: 4 * time.Second}}
	tr := Line(localization.Vec2{}, localization.Vec2{X: 2}, localization.Vec2{},
		HeadingParams{}, 8*time.Second)
	s, err := Generate(tr, cfg, 5)
	if err != nil {
		t.Fatal(err)
	}
	// Generate は設定のコピーに既定値を埋めるので、解決後の値は s.Config から読む。
	bias := s.Config.VisionTimeBias
	for _, v := range s.Vision {
		capture := v.Stamp.Sub(0) - bias
		if capture >= 2*time.Second && capture < 4*time.Second {
			t.Fatalf("an observation survived the outage at %v", capture)
		}
	}
	// 欠落の前後には観測があること。
	var before, after int
	for _, v := range s.Vision {
		if v.Stamp < localization.Stamp(2*time.Second) {
			before++
		}
		if v.Stamp > localization.Stamp(4*time.Second+bias) {
			after++
		}
	}
	if before == 0 || after == 0 {
		t.Errorf("observations before/after the outage: %d/%d", before, after)
	}
}

// 遅延は非負で上側に裾を持つこと (計画 §5.3)。
func TestVisionDelayIsNonNegativeWithTail(t *testing.T) {
	cfg := DefaultConfig()
	cfg.VisionLossRate = 0
	tr := Line(localization.Vec2{}, localization.Vec2{X: 1}, localization.Vec2{},
		HeadingParams{}, 60*time.Second)
	s, err := Generate(tr, cfg, 6)
	if err != nil {
		t.Fatal(err)
	}

	// 解決後の設定を使う。cfg 側には既定値が埋まっていない。
	resolved := s.Config
	var minDelay, maxDelay time.Duration = time.Hour, 0
	for _, v := range s.Vision {
		capture := v.Stamp - localization.Stamp(resolved.VisionTimeBias)
		d := time.Duration(v.Arrival - capture)
		if d < 0 {
			t.Fatalf("negative one-way delay %v", d)
		}
		if d < minDelay {
			minDelay = d
		}
		if d > maxDelay {
			maxDelay = d
		}
	}
	if minDelay < resolved.VisionMinDelay {
		t.Errorf("minimum delay %v is below the configured floor %v", minDelay, resolved.VisionMinDelay)
	}
	if maxDelay < resolved.VisionMinDelay+2*resolved.VisionJitter {
		t.Errorf("maximum delay %v shows no upper tail (min %v, jitter %v)",
			maxDelay, resolved.VisionMinDelay, resolved.VisionJitter)
	}
}

func TestVisionNoiseMatchesConfig(t *testing.T) {
	cfg := DefaultConfig()
	cfg.VisionLossRate = 0
	// 静止させれば、観測のばらつきがそのまま雑音になる。
	tr := Stationary(localization.Vec2{X: 1, Y: -1}, HeadingParams{Theta0: 0.3}, 120*time.Second)
	s, err := Generate(tr, cfg, 7)
	if err != nil {
		t.Fatal(err)
	}
	var sx, sy, st float64
	for _, v := range s.Vision {
		sx += (v.Pose.X - 1) * (v.Pose.X - 1)
		sy += (v.Pose.Y + 1) * (v.Pose.Y + 1)
		d := localization.AngleDiff(v.Pose.Theta, 0.3)
		st += d * d
	}
	n := float64(len(s.Vision))
	for name, got := range map[string]float64{
		"x": math.Sqrt(sx / n),
		"y": math.Sqrt(sy / n),
	} {
		if math.Abs(got-cfg.VisionNoiseM)/cfg.VisionNoiseM > 0.1 {
			t.Errorf("%s noise sigma = %.4f m, want %.4f", name, got, cfg.VisionNoiseM)
		}
	}
	if got := math.Sqrt(st / n); math.Abs(got-cfg.VisionNoiseRad)/cfg.VisionNoiseRad > 0.1 {
		t.Errorf("heading noise sigma = %.5f rad, want %.5f", got, cfg.VisionNoiseRad)
	}
}

func TestMultipleCamerasNumberFramesIndependently(t *testing.T) {
	cfg := DefaultConfig()
	cfg.VisionLossRate = 0
	cfg.Cameras = 4
	tr := Line(localization.Vec2{}, localization.Vec2{X: 1}, localization.Vec2{},
		HeadingParams{}, 4*time.Second)
	s, err := Generate(tr, cfg, 8)
	if err != nil {
		t.Fatal(err)
	}
	last := map[uint32]uint32{}
	for _, v := range s.Vision {
		if prev, ok := last[v.CameraID]; ok && v.FrameNumber != prev+1 {
			t.Fatalf("camera %d jumped from frame %d to %d", v.CameraID, prev, v.FrameNumber)
		}
		last[v.CameraID] = v.FrameNumber
	}
	if len(last) != 4 {
		t.Errorf("saw %d cameras, want 4", len(last))
	}
}

// 同じ seed なら必ず同じ列が出ること。回帰テストの前提。
func TestGenerateIsDeterministic(t *testing.T) {
	cfg := DefaultConfig()
	a, err := Generate(testTrajectory(), cfg, 42)
	if err != nil {
		t.Fatal(err)
	}
	b, err := Generate(testTrajectory(), cfg, 42)
	if err != nil {
		t.Fatal(err)
	}
	if len(a.Wheels) != len(b.Wheels) || len(a.Vision) != len(b.Vision) {
		t.Fatalf("lengths differ: wheels %d/%d, vision %d/%d",
			len(a.Wheels), len(b.Wheels), len(a.Vision), len(b.Vision))
	}
	for i := range a.Wheels {
		if a.Wheels[i] != b.Wheels[i] {
			t.Fatalf("wheel sample %d differs", i)
		}
	}
	for i := range a.Vision {
		if a.Vision[i] != b.Vision[i] {
			t.Fatalf("vision sample %d differs", i)
		}
	}
}

func TestTruthAtInterpolates(t *testing.T) {
	cfg := DefaultConfig()
	tr := Line(localization.Vec2{}, localization.Vec2{X: 2}, localization.Vec2{},
		HeadingParams{}, 2*time.Second)
	s, err := Generate(tr, cfg, 9)
	if err != nil {
		t.Fatal(err)
	}
	// サンプル間の時刻でも、直線軌道なら厳密に一致するはず。
	at := localization.Stamp(12 * time.Millisecond) // 8ms と 16ms の間
	got, ok := s.TruthAt(at)
	if !ok {
		t.Fatal("TruthAt failed")
	}
	want := 2.0 * 0.012
	if math.Abs(got.Pose.X-want) > 1e-9 {
		t.Errorf("interpolated x = %v, want %v", got.Pose.X, want)
	}

	if _, ok := s.TruthAt(localization.Stamp(10 * time.Second)); ok {
		t.Error("TruthAt returned a value past the end of the trajectory")
	}
}

func TestGenerateRejectsBadGeometry(t *testing.T) {
	cfg := DefaultConfig()
	cfg.Geometry.WheelSigns[0] = 0
	if _, err := Generate(testTrajectory(), cfg, 1); err == nil {
		t.Error("expected an error for an invalid geometry")
	}
}
