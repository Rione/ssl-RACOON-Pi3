package control

import (
	"math"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/localization"
)

func feedbackConfig() VelocityFeedbackConfig {
	return VelocityFeedbackConfig{LinearGain: .5, AngularGain: .5, MaxLinearCorrection: .3, MaxAngularCorrection: .4,
		MaxSpeed: 2, MaxYawRate: 3, MaxEstimateAge: 40 * time.Millisecond}
}

func feedback(t *testing.T, cfg VelocityFeedbackConfig) *VelocityFeedback {
	t.Helper()
	f, err := NewVelocityFeedback(cfg)
	if err != nil {
		t.Fatal(err)
	}
	return f
}

func TestVelocityFeedbackTracksError(t *testing.T) {
	f := feedback(t, feedbackConfig())
	target := Command{VelBody: localization.Vec2{X: 1, Y: -.4}, YawRate: .6}
	for _, tc := range []struct {
		name           string
		measured, want Command
	}{
		{"on target", target, target},
		{"too slow", Command{VelBody: localization.Vec2{X: .8, Y: -.2}, YawRate: .4}, Command{VelBody: localization.Vec2{X: 1.1, Y: -.5}, YawRate: .7}},
		{"too fast", Command{VelBody: localization.Vec2{X: 1.2, Y: -.6}, YawRate: .8}, Command{VelBody: localization.Vec2{X: .9, Y: -.3}, YawRate: .5}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got, err := f.Correct(target, localization.Estimate{VelBody: tc.measured.VelBody, YawRate: tc.measured.YawRate}, 0)
			if err != nil {
				t.Fatal(err)
			}
			near(t, got.VelBody.X, tc.want.VelBody.X)
			near(t, got.VelBody.Y, tc.want.VelBody.Y)
			near(t, got.YawRate, tc.want.YawRate)
		})
	}
}

func TestVelocityFeedbackStallDoesNotAccumulate(t *testing.T) {
	f := feedback(t, feedbackConfig())
	// 停止したままの計測を繰り返しても、補正を上限以上に積み上げない。
	for i := 0; i < 1000; i++ {
		now := localization.Stamp(i) * localization.Stamp(8*time.Millisecond)
		got, err := f.Correct(Command{VelBody: localization.Vec2{X: .6, Y: .8}, YawRate: -1}, localization.Estimate{Stamp: now}, now)
		if err != nil {
			t.Fatal(err)
		}
		near(t, got.VelBody.X, .78)
		near(t, got.VelBody.Y, 1.04)
		near(t, got.YawRate, -1.4)
	}
}

func TestVelocityFeedbackOutputLimitsAndZeroGains(t *testing.T) {
	cfg := feedbackConfig()
	cfg.MaxSpeed = 1
	cfg.MaxYawRate = 1
	f := feedback(t, cfg)
	got, err := f.Correct(Command{VelBody: localization.Vec2{X: 3, Y: 4}, YawRate: 5}, localization.Estimate{}, 0)
	if err != nil {
		t.Fatal(err)
	}
	near(t, got.VelBody.X, .6)
	near(t, got.VelBody.Y, .8)
	near(t, got.YawRate, 1)
	cfg.LinearGain = 0
	cfg.AngularGain = 0
	f = feedback(t, cfg)
	target := Command{VelBody: localization.Vec2{X: .5}, YawRate: -.5}
	got, err = f.Correct(target, localization.Estimate{VelBody: localization.Vec2{X: -1}, YawRate: 1}, 0)
	if err != nil || got != target {
		t.Fatalf("disabled correction: %+v %v", got, err)
	}
}

func TestVelocityFeedbackRejectsBadEstimates(t *testing.T) {
	f := feedback(t, feedbackConfig())
	now := stamp(1)
	target := Command{VelBody: localization.Vec2{X: 1}}
	for _, e := range []localization.Estimate{
		{Stamp: now, Health: localization.HealthInvalid}, {Stamp: now, Health: 255},
		{Stamp: now.Add(-41 * time.Millisecond)}, {Stamp: now.Add(time.Nanosecond)}, {Stamp: -1},
		{Stamp: now, VelBody: localization.Vec2{X: math.NaN()}}, {Stamp: now, YawRate: math.Inf(1)},
	} {
		got, err := f.Correct(target, e, now)
		if err == nil || got != (Command{}) {
			t.Fatalf("bad estimate accepted: %+v -> %+v %v", e, got, err)
		}
	}
	for _, h := range []localization.Health{localization.HealthOK, localization.HealthDegraded} {
		if _, err := f.Correct(target, localization.Estimate{Stamp: now.Add(-40 * time.Millisecond), Health: h}, now); err != nil {
			t.Fatal(err)
		}
	}
	// 停止は推定器が壊れていてもゼロ。実測速度から逆方向指令を生成しない。
	got, err := f.Correct(Command{}, localization.Estimate{Health: localization.HealthInvalid, VelBody: localization.Vec2{X: 1}}, now)
	if err != nil || got != (Command{}) {
		t.Fatalf("stop: %+v %v", got, err)
	}
}

func TestVelocityFeedbackInvalidConfigurationAndOverflow(t *testing.T) {
	for _, mutate := range []func(*VelocityFeedbackConfig){
		func(c *VelocityFeedbackConfig) { c.LinearGain = -1 },
		func(c *VelocityFeedbackConfig) { c.AngularGain = math.NaN() },
		func(c *VelocityFeedbackConfig) { c.MaxLinearCorrection = -1 },
		func(c *VelocityFeedbackConfig) { c.MaxAngularCorrection = math.Inf(1) },
		func(c *VelocityFeedbackConfig) { c.MaxSpeed = 0 },
		func(c *VelocityFeedbackConfig) { c.MaxYawRate = -1 },
		func(c *VelocityFeedbackConfig) { c.MaxEstimateAge = 0 },
	} {
		cfg := feedbackConfig()
		mutate(&cfg)
		if _, err := NewVelocityFeedback(cfg); err == nil {
			t.Fatalf("accepted %+v", cfg)
		}
	}
	cfg := feedbackConfig()
	cfg.LinearGain = math.MaxFloat64
	f := feedback(t, cfg)
	got, err := f.Correct(Command{VelBody: localization.Vec2{X: 1}}, localization.Estimate{VelBody: localization.Vec2{X: -2}}, 0)
	if err == nil || got != (Command{}) {
		t.Fatalf("overflow: %+v %v", got, err)
	}
	var zero VelocityFeedback
	if _, err := zero.Correct(Command{}, localization.Estimate{}, 0); err == nil {
		t.Fatal("accepted zero feedback")
	}
	if _, err := f.Correct(Command{YawRate: math.NaN()}, localization.Estimate{}, 0); err == nil {
		t.Fatal("accepted NaN target")
	}
}

func TestTrajectoryWithVelocityFeedback(t *testing.T) {
	cfg := config()
	cfg.MaxSpeed = 1.1
	c := controller(t, []Node{{VelBody: localization.Vec2{X: 1}}, {Stamp: stamp(2), Pose: localization.Pose2{X: 2}, VelBody: localization.Vec2{X: 1}}}, cfg)
	f := feedback(t, feedbackConfig())
	e := localization.Estimate{Stamp: stamp(1), Pose: localization.Pose2{X: 1}, VelBody: localization.Vec2{X: .5}}
	got, phase, err := c.CalculateWithFeedback(e, e.Stamp, f)
	if err != nil || phase != Tracking {
		t.Fatalf("%v %v", phase, err)
	}
	near(t, got.VelBody.X, 1.1) // 補正後も経路側の上限を守る
	if got, _, err := c.CalculateWithFeedback(e, stamp(1.1), f); err == nil || got != (Command{}) {
		t.Fatal("accepted stale estimate")
	}
	e.Stamp = stamp(2)
	got, phase, err = c.CalculateWithFeedback(e, e.Stamp, f)
	if err != nil || phase != Finished || got != (Command{}) {
		t.Fatalf("terminal: %+v %v %v", got, phase, err)
	}
}

func TestVelocityFeedbackReducesUnderdriveError(t *testing.T) {
	// 速度応答が一次遅れ(80 ms)、定常ゲイン0.7の仮想機体。
	// P補正で速度不足が減ることを検証。完全な定常偏差除去は主張しない。
	f := feedback(t, feedbackConfig())
	simulate := func(enabled bool) float64 {
		v := 0.0
		for i := 0; i < 500; i++ {
			now := localization.Stamp(i) * localization.Stamp(8*time.Millisecond)
			cmd := Command{VelBody: localization.Vec2{X: 1}}
			if enabled {
				var err error
				cmd, err = f.Correct(cmd, localization.Estimate{Stamp: now, VelBody: localization.Vec2{X: v}}, now)
				if err != nil {
					t.Fatal(err)
				}
			}
			v += .008 / .08 * (.7*cmd.VelBody.X - v)
		}
		return math.Abs(1 - v)
	}
	baseline, corrected := simulate(false), simulate(true)
	if corrected >= baseline*.8 {
		t.Fatalf("insufficient improvement: before=%g after=%g", baseline, corrected)
	}
}
