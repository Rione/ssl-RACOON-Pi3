package app

import (
	"sync"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/control"
	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi3/internal/mw"
	"github.com/Rione/ssl-RACOON-Pi3/internal/receive"
	"github.com/Rione/ssl-RACOON-Pi3/internal/supervisor"
)

func TestControlCycleObservationAndStops(t *testing.T) {
	e, err := localization.NewEstimator(localization.DefaultConfig(), localization.EstimatorOptions{})
	if err != nil {
		t.Fatal(err)
	}
	nodes := []control.Node{{Stamp: 0, VelBody: localization.Vec2{X: 1}}, {Stamp: localization.Stamp(time.Second), VelBody: localization.Vec2{X: 1}}}
	plan, err := receive.PreparePlan(7, localization.Stamp(time.Second), nodes, control.Config{PositionGain: 2, HeadingGain: 2, MaxSpeed: 3, MaxYawRate: 3})
	if err != nil {
		t.Fatal(err)
	}
	nodes[0].VelBody.X = 999 // 受信バッファを再利用しても計画が変わらない。
	feedback, err := control.NewVelocityFeedback(control.VelocityFeedbackConfig{
		LinearGain: .5, AngularGain: .5, MaxLinearCorrection: 1, MaxAngularCorrection: 1,
		MaxSpeed: 3, MaxYawRate: 3, MaxEstimateAge: 50 * time.Millisecond,
	})
	if err != nil {
		t.Fatal(err)
	}
	c := ControlCycle{Estimator: e, Feedback: feedback, MaxEstimateAge: 50 * time.Millisecond}
	in := CycleInput{Now: localization.Stamp(10 * time.Millisecond), Mode: supervisor.LocalTracking, Plan: plan}
	if got := c.Step(in); got.Reason != supervisor.EstimateUnavailable || got.Command != (control.Command{}) {
		t.Fatalf("uninitialized: %+v", got)
	}
	in.Wheels = []localization.WheelSample{{Stamp: 0}, {Stamp: in.Now}}
	in.Vision = []localization.VisionPose{{Stamp: in.Now, Confidence: 1, Mapped: true}}
	got := c.Step(in)
	if got.Reason != supervisor.Ready || got.Phase != control.Tracking || got.Command.VelBody.X != 1.5 || got.Error.Velocity.X != 1 || got.PlanID != 7 {
		t.Fatalf("tracking: %+v", got)
	}
	in.Wheels, in.Vision = nil, nil
	// 観測更新を含まない周期の経路評価・受渡しで動的確保を増やさない。
	var store SnapshotStore
	if n := testing.AllocsPerRun(100, func() { store.Store(c.Step(in)); store.Load() }); n != 0 {
		t.Fatalf("cycle/snapshot allocations = %v", n)
	}
	for _, tc := range []struct {
		name   string
		modify func(*CycleInput)
		reason supervisor.Reason
	}{
		{"emergency", func(i *CycleInput) { i.Emergency = true }, supervisor.EmergencyStop},
		{"expired", func(i *CycleInput) { i.Now = localization.Stamp(time.Second) }, supervisor.Expired},
		{"stale", func(i *CycleInput) { i.Now = localization.Stamp(100 * time.Millisecond) }, supervisor.EstimateUnavailable},
		{"no plan", func(i *CycleInput) { i.Plan = nil }, supervisor.Expired},
		{"disarmed", func(i *CycleInput) { i.Mode = supervisor.Disarmed }, supervisor.NotArmed},
	} {
		t.Run(tc.name, func(t *testing.T) {
			input := in
			tc.modify(&input)
			result := c.Step(input)
			if result.Reason != tc.reason || result.Command != (control.Command{}) {
				t.Fatalf("got %+v", result)
			}
		})
	}
}

func TestSnapshotConcurrentValueCopies(t *testing.T) {
	var s SnapshotStore
	if _, ok := s.Load(); ok {
		t.Fatal("empty snapshot marked valid")
	}
	var wg sync.WaitGroup
	for i := 0; i < 4; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := 0; j < 100; j++ {
				v := mw.EstimateReport{PlanID: 42}
				s.Store(v)
				v.PlanID = 99
				out, ok := s.Load()
				if !ok || out.PlanID != 42 {
					t.Error("snapshot not an independent copy")
				}
			}
		}()
	}
	wg.Wait()
}
