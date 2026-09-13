package localization

import (
	"math"
	"testing"
	"time"
)

func newTestEstimator(t testing.TB) *Estimator {
	t.Helper()
	e, err := NewEstimator(DefaultConfig(), EstimatorOptions{})
	if err != nil {
		t.Fatalf("NewEstimator: %v", err)
	}
	return e
}

// 計画 §7.3 / §10-9: 1 周期 0 アロケーション。
//
// ここが破れると GC の発生頻度が上がり、§5.2 の dt ジッタが増える。
// interface へのボックス化ひとつでも落ちる。
func TestEstimatorHotPathDoesNotAllocate(t *testing.T) {
	e := newTestEstimator(t)

	// バッファと内部状態を立ち上げてから測る。
	stamp := Stamp(0)
	step := Stamp(8 * time.Millisecond)
	for i := 0; i < 200; i++ {
		stamp += step
		e.AddWheel(WheelSample{Stamp: stamp, Omega: [NumWheels]float64{10, -20, 30, -40}})
	}

	got := testing.AllocsPerRun(2000, func() {
		stamp += step
		e.AddWheel(WheelSample{Stamp: stamp, Omega: [NumWheels]float64{10, -20, 30, -40}})
	})
	if got != 0 {
		t.Errorf("AddWheel allocated %v times per run, want 0", got)
	}
}

// 遅延 vision の取り込み (巻き戻し + 再フィルタ) もアロケートしないこと。
// ここが一番重い経路なので、ホットパスから外れていても押さえておく。
func TestVisionUpdateDoesNotAllocate(t *testing.T) {
	e := newTestEstimator(t)
	stamp := Stamp(0)
	step := Stamp(8 * time.Millisecond)
	for i := 0; i < 200; i++ {
		stamp += step
		e.AddWheel(WheelSample{Stamp: stamp, Omega: [NumWheels]float64{10, -20, 30, -40}})
	}

	got := testing.AllocsPerRun(500, func() {
		stamp += step
		e.AddWheel(WheelSample{Stamp: stamp, Omega: [NumWheels]float64{10, -20, 30, -40}})
		// 100 ms 前の観測。バッファの中ほどまで巻き戻して再フィルタが走る。
		e.AddVision(VisionPose{
			Stamp: stamp - Stamp(100*time.Millisecond),
			Pose:  Pose2{X: 1, Y: 2, Theta: 0.3},
		})
	})
	if got != 0 {
		t.Errorf("AddWheel+AddVision allocated %v times per run, want 0", got)
	}
}

// 計画 §10-9: 1 周期 <= 1 ms。
func BenchmarkEstimatorCycle(b *testing.B) {
	e := newTestEstimator(b)
	stamp := Stamp(0)
	step := Stamp(8 * time.Millisecond)
	for i := 0; i < 200; i++ {
		stamp += step
		e.AddWheel(WheelSample{Stamp: stamp, Omega: [NumWheels]float64{10, -20, 30, -40}})
	}

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		stamp += step
		e.AddWheel(WheelSample{Stamp: stamp, Omega: [NumWheels]float64{10, -20, 30, -40}})
		// 実機では車輪 125 Hz に対し vision 60 Hz なので、2 周期に 1 回。
		if i%2 == 0 {
			e.AddVision(VisionPose{
				Stamp: stamp - Stamp(30*time.Millisecond),
				Pose:  Pose2{X: 1, Y: 2, Theta: 0.3},
			})
		}
	}
}

// バッファより古い観測は捨てて数えること。
// 無理に取り込むと共分散が壊れる (計画 §4.6)。
func TestVisionTooOldIsDiscarded(t *testing.T) {
	e := newTestEstimator(t)
	stamp := Stamp(0)
	step := Stamp(8 * time.Millisecond)
	for i := 0; i < 100; i++ {
		stamp += step
		e.AddWheel(WheelSample{Stamp: stamp, Omega: [NumWheels]float64{}})
	}
	e.AddVision(VisionPose{Stamp: stamp - Stamp(10*time.Millisecond), Pose: Pose2{}})

	before := e.Stats().VisionUpdates
	// バッファは 25 周期 = 200 ms。それより古い観測。
	e.AddVision(VisionPose{Stamp: stamp - Stamp(2*time.Second), Pose: Pose2{X: 99}})

	s := e.Stats()
	if s.VisionTooOld != 1 {
		t.Errorf("VisionTooOld = %d, want 1", s.VisionTooOld)
	}
	if s.VisionUpdates != before {
		t.Errorf("a too-old observation was applied anyway")
	}
	if math.Abs(e.Current().Pose.X-99) < 1 {
		t.Error("the discarded observation still moved the estimate")
	}
}

// 車輪 1 周期ぶん先行する vision は正常として扱うこと。
// vision 60 Hz / 車輪 125 Hz なので普通に起きる。
func TestVisionSlightlyAheadIsNormal(t *testing.T) {
	e := newTestEstimator(t)
	stamp := Stamp(0)
	step := Stamp(8 * time.Millisecond)
	for i := 0; i < 50; i++ {
		stamp += step
		e.AddWheel(WheelSample{Stamp: stamp, Omega: [NumWheels]float64{}})
		e.AddVision(VisionPose{Stamp: stamp + Stamp(4*time.Millisecond), Pose: Pose2{X: 1}})
	}
	if got := e.Stats().VisionFuture; got != 0 {
		t.Errorf("VisionFuture = %d; a 4 ms lead is normal at 60 Hz vision and must not be counted as an anomaly", got)
	}
	if got := e.Stats().VisionUpdates; got < 40 {
		t.Errorf("only %d vision updates were applied out of 50", got)
	}
}

// 明らかに未来すぎる観測は時刻同期の異常として捨てること。
func TestVisionFarInFutureIsRejected(t *testing.T) {
	e := newTestEstimator(t)
	stamp := Stamp(0)
	step := Stamp(8 * time.Millisecond)
	for i := 0; i < 50; i++ {
		stamp += step
		e.AddWheel(WheelSample{Stamp: stamp, Omega: [NumWheels]float64{}})
	}
	e.AddVision(VisionPose{Stamp: stamp, Pose: Pose2{}})
	e.AddVision(VisionPose{Stamp: stamp + Stamp(time.Second), Pose: Pose2{X: 99}})

	if got := e.Stats().VisionFuture; got != 1 {
		t.Errorf("VisionFuture = %d, want 1 for an observation 1 second in the future", got)
	}
}

// NaN が出たら自動で作り直し、走行機能を巻き込まないこと
// (計画 §9 の検証項目 8)。
func TestEstimatorRecoversFromNaN(t *testing.T) {
	e := newTestEstimator(t)
	stamp := Stamp(0)
	step := Stamp(8 * time.Millisecond)
	for i := 0; i < 50; i++ {
		stamp += step
		e.AddWheel(WheelSample{Stamp: stamp, Omega: [NumWheels]float64{}})
		e.AddVision(VisionPose{Stamp: stamp, Pose: Pose2{X: 1, Y: 1}})
	}

	// 状態を直接壊す。
	e.f.x.V.X = math.NaN()

	stamp += step
	e.AddWheel(WheelSample{Stamp: stamp, Omega: [NumWheels]float64{}})

	if got := e.Stats().Resets; got != 1 {
		t.Fatalf("Resets = %d, want 1 after a NaN", got)
	}
	// 作り直した後も出力が有限であること。
	cur := e.Current()
	for _, v := range []float64{cur.Pose.X, cur.Pose.Y, cur.Pose.Theta, cur.VelBody.X, cur.VelBody.Y} {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			t.Fatalf("the estimate is still non-finite after the reset: %+v", cur)
		}
	}
	// 直前の位置は引き継がれていること。ゼロへ飛ぶと制御が跳ねる。
	if math.Abs(cur.Pose.X-1) > 0.5 {
		t.Errorf("the reset threw away the last known pose: x = %v, want about 1", cur.Pose.X)
	}
}

// vision が来る前は DEGRADED、来たら OK、途切れたら DEGRADED へ戻ること。
// RAVEN はこの値を見て vision 生値へフォールバックする。
func TestHealthTransitions(t *testing.T) {
	e := newTestEstimator(t)
	stamp := Stamp(0)
	step := Stamp(8 * time.Millisecond)

	e.AddWheel(WheelSample{Stamp: stamp, Omega: [NumWheels]float64{}})
	if got := e.Current().Health; got != HealthDegraded {
		t.Errorf("before any vision: health = %v, want DEGRADED", got)
	}

	for i := 0; i < 20; i++ {
		stamp += step
		e.AddWheel(WheelSample{Stamp: stamp, Omega: [NumWheels]float64{}})
		e.AddVision(VisionPose{Stamp: stamp, Pose: Pose2{}})
	}
	if got := e.Current().Health; got != HealthOK {
		t.Errorf("with vision flowing: health = %v, want OK", got)
	}

	// vision を止める。既定のタイムアウトは 200 ms。
	for i := 0; i < 40; i++ {
		stamp += step
		e.AddWheel(WheelSample{Stamp: stamp, Omega: [NumWheels]float64{}})
	}
	cur := e.Current()
	if cur.Health != HealthDegraded {
		t.Errorf("after a 320 ms vision gap: health = %v, want DEGRADED", cur.Health)
	}
	if cur.SinceVision < 300*time.Millisecond {
		t.Errorf("SinceVision = %v, want about 320 ms", cur.SinceVision)
	}
}

func TestNewEstimatorRejectsBadConfig(t *testing.T) {
	cfg := DefaultConfig()
	cfg.Noise.WheelNoise = 0
	if _, err := NewEstimator(cfg, EstimatorOptions{}); err == nil {
		t.Error("expected an error for a zero wheel noise")
	}
	cfg = DefaultConfig()
	cfg.Geometry.MomentArmM = -1
	if _, err := NewEstimator(cfg, EstimatorOptions{}); err == nil {
		t.Error("expected an error for a negative moment arm")
	}
}
