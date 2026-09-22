package control

import (
	"math"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

func config() Config                     { return Config{PositionGain: 2, HeadingGain: 3, MaxSpeed: 10, MaxYawRate: 10} }
func stamp(s float64) localization.Stamp { return localization.Stamp(s * float64(time.Second)) }
func near(t *testing.T, got, want float64) {
	t.Helper()
	if math.IsNaN(got) || math.Abs(got-want) > 1e-9 {
		t.Fatalf("got %g, want %g", got, want)
	}
}
func controller(t *testing.T, nodes []Node, cfg Config) *Controller {
	t.Helper()
	c, err := New(nodes, cfg)
	if err != nil {
		t.Fatal(err)
	}
	return c
}

func TestInterpolationAndBodyFrame(t *testing.T) {
	// 両端の body 速度は異なるが、world ではどちらも +x に 1 m/s。
	nodes := []Node{
		{Stamp: stamp(1), VelBody: localization.Vec2{X: 1}},
		{Stamp: stamp(3), Pose: localization.Pose2{X: 2, Theta: math.Pi / 2}, VelBody: localization.Vec2{Y: -1}, YawRate: 2},
	}
	c := controller(t, nodes, config())
	cmd, phase, err := c.Calculate(localization.Estimate{Stamp: stamp(2), Pose: localization.Pose2{X: .5, Theta: math.Pi / 2}})
	if err != nil || phase != Tracking {
		t.Fatalf("phase=%v err=%v", phase, err)
	}
	near(t, cmd.VelBody.X, 0)
	near(t, cmd.VelBody.Y, -2) // FF=1 + 2*(1-.5)=2 を現在のbodyへ変換
	near(t, cmd.YawRate, 1-3*math.Pi/4)
}

func TestAngleWrap(t *testing.T) {
	c := controller(t, []Node{
		{Pose: localization.Pose2{Theta: 170 * math.Pi / 180}},
		{Stamp: stamp(2), Pose: localization.Pose2{Theta: -170 * math.Pi / 180}},
	}, config())
	cmd, _, err := c.Calculate(localization.Estimate{Stamp: stamp(1), Pose: localization.Pose2{Theta: 179 * math.Pi / 180}})
	if err != nil {
		t.Fatal(err)
	}
	near(t, cmd.YawRate, 3*math.Pi/180)
}

func TestBoundariesAndPathOwnership(t *testing.T) {
	nodes := []Node{
		{Stamp: stamp(1), VelBody: localization.Vec2{X: 1}},
		{Stamp: stamp(2), VelBody: localization.Vec2{X: 2}},
		{Stamp: stamp(3), VelBody: localization.Vec2{X: 3}},
	}
	c := controller(t, nodes, config())
	nodes[1].VelBody.X = 99
	for _, tc := range []struct {
		seconds, vx float64
		phase       Phase
	}{
		{0, 0, Waiting}, {1, 1, Tracking}, {2, 2, Tracking}, {2.5, 2.5, Tracking}, {3, 0, Finished}, {4, 0, Finished},
	} {
		cmd, phase, err := c.Calculate(localization.Estimate{Stamp: stamp(tc.seconds)})
		if err != nil || phase != tc.phase {
			t.Fatalf("t=%v phase=%v err=%v", tc.seconds, phase, err)
		}
		near(t, cmd.VelBody.X, tc.vx)
	}
}

func TestVelocityLimits(t *testing.T) {
	cfg := config()
	cfg.MaxSpeed = 2
	cfg.MaxYawRate = 1
	for _, sign := range []float64{-1, 1} {
		n := Node{VelBody: localization.Vec2{X: 3, Y: 4}, YawRate: sign * 5}
		b := n
		b.Stamp = stamp(1)
		c := controller(t, []Node{n, b}, cfg)
		cmd, _, err := c.Calculate(localization.Estimate{})
		if err != nil {
			t.Fatal(err)
		}
		near(t, cmd.VelBody.X, 1.2)
		near(t, cmd.VelBody.Y, 1.6)
		near(t, cmd.YawRate, sign)
	}
}

func TestRejectInvalidInputs(t *testing.T) {
	valid := []Node{{}, {Stamp: stamp(1)}}
	for _, nodes := range [][]Node{nil, {{}}, {{}, {}}, {{Stamp: stamp(2)}, {Stamp: stamp(1)}}, {{Stamp: -1}, {Stamp: 1}}, {{Pose: localization.Pose2{X: math.NaN()}}, {Stamp: 1}}, {{YawRate: math.Inf(1)}, {Stamp: 1}}} {
		if _, err := New(nodes, config()); err == nil {
			t.Fatalf("accepted invalid nodes: %+v", nodes)
		}
	}
	for _, cfg := range []Config{{}, {PositionGain: -1, MaxSpeed: 1, MaxYawRate: 1}, {MaxSpeed: math.Inf(1), MaxYawRate: 1}} {
		if _, err := New(valid, cfg); err == nil {
			t.Fatalf("accepted invalid config: %+v", cfg)
		}
	}
	c := controller(t, valid, config())
	for _, estimate := range []localization.Estimate{{Health: localization.HealthInvalid}, {Health: 255}, {Stamp: -1}, {Pose: localization.Pose2{Theta: math.NaN()}}, {Pose: localization.Pose2{X: math.MaxFloat64}}} {
		cmd, _, err := c.Calculate(estimate)
		if err == nil || cmd != (Command{}) {
			t.Fatalf("invalid estimate: cmd=%+v err=%v", cmd, err)
		}
	}
	var zero Controller
	if _, _, err := zero.Calculate(localization.Estimate{}); err == nil {
		t.Fatal("accepted uninitialized controller")
	}
}

func TestStraightTrackingConvergesAt125Hz(t *testing.T) {
	// 理想的な速度応答のロボットで、初期の位置・姿勢誤差が減ることを確認。
	c := controller(t, []Node{
		{VelBody: localization.Vec2{X: 1}},
		{Stamp: stamp(3), Pose: localization.Pose2{X: 3}, VelBody: localization.Vec2{X: 1}},
	}, config())
	estimate := localization.Estimate{Pose: localization.Pose2{X: -.3, Y: .2, Theta: .2}, Health: localization.HealthDegraded}
	const dt = .008
	for i := 0; i < 250; i++ {
		estimate.Stamp = localization.Stamp(i) * localization.Stamp(8*time.Millisecond)
		cmd, phase, err := c.Calculate(estimate)
		if err != nil || phase != Tracking {
			t.Fatalf("phase=%v err=%v", phase, err)
		}
		v := localization.Rotate(estimate.Pose.Theta, cmd.VelBody)
		estimate.Pose.X += v.X * dt
		estimate.Pose.Y += v.Y * dt
		estimate.Pose.Theta += cmd.YawRate * dt
	}
	if math.Hypot(estimate.Pose.X-2, estimate.Pose.Y) > .01 || math.Abs(estimate.Pose.Theta) > .001 {
		t.Fatalf("tracking did not converge: %+v", estimate.Pose)
	}
}
