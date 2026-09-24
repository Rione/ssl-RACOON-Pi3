package localization

import (
	"math"
	"testing"
	"time"
)

// 一定速度で走らせたあと、PredictAhead が位置を v*d だけ進め、
// 共分散を広げ、**フィルタ本体を汚さない**ことを固定する。
func TestPredictAheadAdvancesPoseWithoutDisturbingFilter(t *testing.T) {
	cfg := DefaultConfig()
	e, err := NewEstimator(cfg, EstimatorOptions{})
	if err != nil {
		t.Fatal(err)
	}
	k, err := NewKinematics(cfg.Geometry)
	if err != nil {
		t.Fatal(err)
	}

	const vx = 1.5
	step := 8 * time.Millisecond
	stamp := Stamp(0)
	wheels := k.WheelFromBody(vx, 0, 0)
	var slots [NumWheels]float64
	for slot := 0; slot < NumWheels; slot++ {
		slots[slot] = wheels[cfg.Geometry.WheelSlotOrder[slot]]
	}

	for i := 0; i < 400; i++ {
		// vision は真の位置をそのまま与える (雑音なし)。
		tt := float64(i) * step.Seconds()
		e.AddVision(VisionPose{Stamp: stamp, Arrival: stamp, Pose: Pose2{X: vx * tt}})
		e.AddWheel(WheelSample{Stamp: stamp, Omega: slots})
		stamp += Stamp(step)
	}

	before := e.Current()
	const horizon = 90 * time.Millisecond
	ahead := e.PredictAhead(horizon)
	after := e.Current()

	// フィルタ本体は動いていない。
	if after.Pose != before.Pose || after.VelBody != before.VelBody {
		t.Fatalf("PredictAhead disturbed the filter: %+v -> %+v", before.Pose, after.Pose)
	}
	if after.Stamp != before.Stamp {
		t.Fatalf("PredictAhead changed the filter stamp: %v -> %v", before.Stamp, after.Stamp)
	}

	// 位置は v*d だけ進む。
	want := before.Pose.X + before.VelBody.X*horizon.Seconds()
	if math.Abs(ahead.Pose.X-want) > 1e-6 {
		t.Fatalf("predicted X = %.6f, want %.6f", ahead.Pose.X, want)
	}
	if ahead.Stamp != before.Stamp.Add(horizon) {
		t.Fatalf("predicted stamp = %v, want %v", ahead.Stamp, before.Stamp.Add(horizon))
	}
	// 共分散は広がる。先読みが長いほど不確かになるのが正しい。
	if ahead.CovPose[0][0] <= before.CovPose[0][0] {
		t.Fatalf("covariance did not grow: %.3g -> %.3g", before.CovPose[0][0], ahead.CovPose[0][0])
	}
	t.Logf("pos sigma %.2f mm -> %.2f mm over %v",
		1000*math.Sqrt(before.CovPose[0][0]), 1000*math.Sqrt(ahead.CovPose[0][0]), horizon)
}

// ホライズンの上限を超えたら打ち切り、Health を落として知らせる。
func TestPredictAheadCapsHorizon(t *testing.T) {
	cfg := DefaultConfig()
	e, err := NewEstimator(cfg, EstimatorOptions{MaxPredictAhead: 100 * time.Millisecond})
	if err != nil {
		t.Fatal(err)
	}
	stamp := Stamp(0)
	for i := 0; i < 200; i++ {
		e.AddVision(VisionPose{Stamp: stamp, Arrival: stamp, Pose: Pose2{}})
		e.AddWheel(WheelSample{Stamp: stamp})
		stamp += Stamp(8 * time.Millisecond)
	}
	ahead := e.PredictAhead(500 * time.Millisecond)
	if ahead.Stamp != e.Current().Stamp.Add(100*time.Millisecond) {
		t.Fatalf("horizon was not capped: %v", ahead.Stamp.Sub(e.Current().Stamp))
	}
	if ahead.Health == HealthOK {
		t.Fatal("capped PredictAhead should not report HealthOK")
	}
}

func TestPredictAheadDoesNotAllocate(t *testing.T) {
	e := newTestEstimator(t)
	stamp := Stamp(0)
	for i := 0; i < 100; i++ {
		stamp += Stamp(8 * time.Millisecond)
		e.AddWheel(WheelSample{Stamp: stamp, Omega: [NumWheels]float64{5, -5, 5, -5}})
	}
	if n := testing.AllocsPerRun(1000, func() {
		_ = e.PredictAhead(90 * time.Millisecond)
	}); n != 0 {
		t.Fatalf("PredictAhead allocated %v times per run", n)
	}
}
