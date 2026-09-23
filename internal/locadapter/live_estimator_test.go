package locadapter

import (
	"math"
	"strings"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// 合成した SPI フレームと vision を流して、機上の推定器が実際に動くこと。
//
// **これが無いと、実機に載せてから「車輪の更新が 0 回」のような事故に気づく。**
// 実際に実機ログでそれが起きたので、結線をテストで固定しておく。
func TestLiveEstimatorProducesEstimates(t *testing.T) {
	rec, clock := newTestSPIRecorder(t, "rock5a-v1")
	vision := make(chan localization.VisionPose, 8)

	cfg := localization.DefaultConfig()
	live, err := NewLiveEstimator(rec, vision, nil, LiveEstimatorConfig{Config: cfg})
	if err != nil {
		t.Fatalf("NewLiveEstimator: %v", err)
	}

	kin, err := localization.NewKinematics(cfg.Geometry)
	if err != nil {
		t.Fatal(err)
	}
	const vx = 0.4
	logical := kin.WheelFromBody(vx, 0, 0)
	var slots [localization.NumWheels]int16
	for slot := 0; slot < localization.NumWheels; slot++ {
		slots[slot] = int16(math.Round(logical[cfg.Geometry.WheelSlotOrder[slot]] * 100))
	}

	base := time.Now()
	step := 8 * time.Millisecond
	for i := 0; i < 400; i++ {
		at := base.Add(time.Duration(i) * step)
		// vision は 116 Hz 相当で、真の位置を入れる。
		if i%1 == 0 {
			tt := float64(i) * step.Seconds()
			stamp := clock.StampOf(at)
			select {
			case vision <- localization.VisionPose{
				Stamp: stamp, Arrival: stamp, Mapped: true,
				Pose: localization.Pose2{X: vx * tt},
			}:
			default:
			}
		}
		live.ObserveSPI(
			txFrame(int16(vx*1000), 0, 0, 0),
			rxFrame(230, 0, 0, [4]int16{slots[0], slots[1], slots[2], slots[3]}),
			at, at.Add(200*time.Microsecond))
	}

	cur, ok := live.Latest()
	if !ok {
		t.Fatal("no estimate was produced")
	}
	st := live.Stats()
	t.Logf("%s", live.Status())
	if st.WheelUpdates < 300 {
		t.Errorf("only %d wheel updates out of 400 cycles", st.WheelUpdates)
	}
	if st.VisionUpdates < 300 {
		t.Errorf("only %d vision updates", st.VisionUpdates)
	}
	if cur.Health != localization.HealthOK {
		t.Errorf("health is %v, want OK", cur.Health)
	}
	// 前進 0.4 m/s を 3.2 秒。位置がそれらしく進んでいること。
	if cur.Pose.X < 1.0 || cur.Pose.X > 1.5 {
		t.Errorf("x = %.3f m, expected about 1.28", cur.Pose.X)
	}
	if math.Abs(cur.VelBody.X-vx) > 0.05 {
		t.Errorf("forward speed %.3f m/s, want %.2f", cur.VelBody.X, vx)
	}
	if w := live.Warnings(); len(w) != 0 {
		t.Errorf("unexpected warnings: %v", w)
	}
}

// 推定が始まる前でも Status / Warnings は落ちない。
func TestLiveEstimatorStatusBeforeAnyData(t *testing.T) {
	rec, _ := newTestSPIRecorder(t, "rock5a-v1")
	live, err := NewLiveEstimator(rec, make(chan localization.VisionPose), nil,
		LiveEstimatorConfig{Config: localization.DefaultConfig()})
	if err != nil {
		t.Fatal(err)
	}
	if s := live.Status(); !strings.Contains(s, "no estimate") {
		t.Errorf("unexpected status before any data: %q", s)
	}
	if w := live.Warnings(); len(w) != 1 {
		t.Errorf("expected exactly one warning before any data, got %v", w)
	}
}

// **推定が壊れた設定でも、作る段階で断る** (走行中に落ちない)。
func TestLiveEstimatorRejectsBadConfig(t *testing.T) {
	rec, _ := newTestSPIRecorder(t, "rock5a-v1")
	cfg := localization.DefaultConfig()
	cfg.Geometry.WheelRadiusM[0] = 0
	if _, err := NewLiveEstimator(rec, nil, nil, LiveEstimatorConfig{Config: cfg}); err == nil {
		t.Fatal("a zero wheel radius should be rejected")
	}
	if _, err := NewLiveEstimator(nil, nil, nil, LiveEstimatorConfig{Config: localization.DefaultConfig()}); err == nil {
		t.Fatal("a nil recorder should be rejected")
	}
}
