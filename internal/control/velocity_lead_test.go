package control

import (
	"math"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

func TestVelocityLeadTakesTheVelocityFromAhead(t *testing.T) {
	// 速度の向きが時間とともに回る経路 (円)。1 rad/s で回る。
	const omega = 1.0
	var nodes []Node
	for i := 0; i <= 200; i++ {
		t := float64(i) * 0.01
		a := omega * t
		nodes = append(nodes, Node{
			Stamp:   localization.Stamp(float64(time.Second) * t),
			Pose:    localization.Pose2{X: math.Sin(a), Y: 1 - math.Cos(a)},
			VelBody: localization.Vec2{X: math.Cos(a), Y: math.Sin(a)},
		})
	}
	at := localization.Stamp(float64(time.Second) * 1.0)
	est := localization.Estimate{Stamp: at, Health: localization.HealthOK,
		Pose: localization.Pose2{X: math.Sin(omega), Y: 1 - math.Cos(omega)}}

	base, err := New(nodes, Config{PositionGain: 3, HeadingGain: 4, MaxSpeed: 5, MaxYawRate: 5})
	if err != nil {
		t.Fatal(err)
	}
	lead, err := New(nodes, Config{PositionGain: 3, HeadingGain: 4, MaxSpeed: 5, MaxYawRate: 5,
		VelocityLead: 100 * time.Millisecond})
	if err != nil {
		t.Fatal(err)
	}
	cb, _, err := base.Calculate(est)
	if err != nil {
		t.Fatal(err)
	}
	cl, _, err := lead.Calculate(est)
	if err != nil {
		t.Fatal(err)
	}
	// 機体の姿勢は 0 なので body = world。速度の向きの差が先読みの時間ぶんの回転 (0.1 rad) になる。
	ang := func(v localization.Vec2) float64 { return math.Atan2(v.Y, v.X) }
	if d := localization.AngleDiff(ang(cl.VelBody), ang(cb.VelBody)); math.Abs(d-omega*0.1) > 0.01 {
		t.Errorf("lead rotated the velocity by %.3f rad, want %.3f", d, omega*0.1)
	}
	if math.Abs(math.Hypot(cl.VelBody.X, cl.VelBody.Y)-math.Hypot(cb.VelBody.X, cb.VelBody.Y)) > 1e-6 {
		t.Error("lead must not change the speed, only its direction")
	}
}

func TestVelocityLeadStopsAtTheEnd(t *testing.T) {
	nodes := []Node{
		{Stamp: 0, Pose: localization.Pose2{}, VelBody: localization.Vec2{X: 1}},
		{Stamp: localization.Stamp(time.Second), Pose: localization.Pose2{X: 1}, VelBody: localization.Vec2{X: 1}},
	}
	c, err := New(nodes, Config{PositionGain: 0, HeadingGain: 0, MaxSpeed: 5, MaxYawRate: 5,
		VelocityLead: 200 * time.Millisecond})
	if err != nil {
		t.Fatal(err)
	}
	// 終わりの 0.1 s 前: 先読みは軌道の外なので、先回しの速度は 0 になる (P だけが残る)
	at := localization.Stamp(float64(time.Second) * 0.9)
	est := localization.Estimate{Stamp: at, Health: localization.HealthOK, Pose: localization.Pose2{X: 0.9}}
	cmd, phase, err := c.Calculate(est)
	if err != nil || phase != Tracking {
		t.Fatalf("phase %v err %v", phase, err)
	}
	if cmd.VelBody.X != 0 || cmd.VelBody.Y != 0 {
		t.Errorf("velocity beyond the last node must be zero, got %+v", cmd.VelBody)
	}
}
