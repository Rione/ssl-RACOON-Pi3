package localization

import (
	"math"
	"testing"
)

func newTestKinematics(t *testing.T) *Kinematics {
	t.Helper()
	k, err := NewKinematics(DefaultGeometry())
	if err != nil {
		t.Fatalf("NewKinematics: %v", err)
	}
	return k
}

// 4 輪は冗長なので、body -> wheel -> body は厳密に元へ戻らなければならない。
func TestKinematicsRoundTrip(t *testing.T) {
	k := newTestKinematics(t)
	cases := [][3]float64{
		{0, 0, 0},
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
		{2.5, -1.2, 8.0},
		{-0.3, 0.75, -3.3},
	}
	for _, c := range cases {
		w := k.WheelFromBody(c[0], c[1], c[2])
		vx, vy, om := k.BodyFromWheel(w)
		if math.Abs(vx-c[0]) > 1e-9 || math.Abs(vy-c[1]) > 1e-9 || math.Abs(om-c[2]) > 1e-9 {
			t.Errorf("body %v -> wheel %v -> body (%v, %v, %v)", c, w, vx, vy, om)
		}
		res := k.Residual(w, c[0], c[1], c[2])
		for i, r := range res {
			if math.Abs(r) > 1e-12 {
				t.Errorf("body %v: residual[%d] = %v, want 0", c, i, r)
			}
		}
	}
}

// 前進だけさせたとき、左右対称な車輪の符号が対称になることを確かめる。
// 取付角 55/135/-135/-55 に対し、sin(55) = sin(135)... ではないので
// 素朴な対称性ではなく「前輪ペアと後輪ペアが逆回り」を確認する。
func TestKinematicsForwardMotionSigns(t *testing.T) {
	k := newTestKinematics(t)
	w := k.WheelFromBody(1.0, 0, 0)
	// v_i = sin(a_i)*vx。sin(55) > 0, sin(135) > 0, sin(-135) < 0, sin(-55) < 0。
	if w[WheelFL] <= 0 || w[WheelBL] <= 0 {
		t.Errorf("forward motion: left wheels should spin positive, got FL=%v BL=%v", w[WheelFL], w[WheelBL])
	}
	if w[WheelBR] >= 0 || w[WheelFR] >= 0 {
		t.Errorf("forward motion: right wheels should spin negative, got BR=%v FR=%v", w[WheelBR], w[WheelFR])
	}
}

// 純回転では 4 輪すべてが同じ向き・同じ大きさで回る。
func TestKinematicsPureRotation(t *testing.T) {
	k := newTestKinematics(t)
	const omega = 2.0
	w := k.WheelFromBody(0, 0, omega)
	g := DefaultGeometry()
	want := -g.MomentArmM * omega / g.WheelRadiusM[0]
	for i, got := range w {
		if math.Abs(got-want) > 1e-12 {
			t.Errorf("pure rotation: wheel[%d] = %v, want %v", i, got, want)
		}
	}
}

// 符号規約を反転させたら、全輪の値がちょうど反転すること。
// 計画 §3.4 の「新世代 STM だけ符号が反転している」を設定だけで吸収できる担保。
func TestKinematicsSignFlip(t *testing.T) {
	base := newTestKinematics(t)
	cfg := DefaultGeometry()
	for i := range cfg.WheelSigns {
		cfg.WheelSigns[i] = -1
	}
	flipped, err := NewKinematics(cfg)
	if err != nil {
		t.Fatalf("NewKinematics: %v", err)
	}
	a := base.WheelFromBody(1.1, -0.4, 2.2)
	b := flipped.WheelFromBody(1.1, -0.4, 2.2)
	for i := range a {
		if math.Abs(a[i]+b[i]) > 1e-12 {
			t.Errorf("wheel[%d]: %v vs flipped %v", i, a[i], b[i])
		}
	}
}

func TestSlotsToLogical(t *testing.T) {
	cfg := DefaultGeometry()
	// FL と FR が入れ替わっている疑い (§12-A4) を表現する。
	cfg.WheelSlotOrder = [NumWheels]int{WheelFR, WheelBL, WheelBR, WheelFL}
	k, err := NewKinematics(cfg)
	if err != nil {
		t.Fatalf("NewKinematics: %v", err)
	}
	got := k.SlotsToLogical([NumWheels]float64{10, 20, 30, 40})
	want := [NumWheels]float64{40, 20, 30, 10} // FL には slot3 の値が入る
	if got != want {
		t.Errorf("SlotsToLogical = %v, want %v", got, want)
	}
}

func TestKinematicsRejectsBadConfig(t *testing.T) {
	cfg := DefaultGeometry()
	cfg.WheelSigns[0] = 0
	if _, err := NewKinematics(cfg); err == nil {
		t.Error("expected error for wheelSigns = 0")
	}

	cfg = DefaultGeometry()
	cfg.WheelSlotOrder = [NumWheels]int{0, 0, 1, 2}
	if _, err := NewKinematics(cfg); err == nil {
		t.Error("expected error for duplicated wheelSlotOrder")
	}

	// 全輪を平行にすると m^T m が特異になる。
	cfg = DefaultGeometry()
	cfg.WheelAnglesDeg = [NumWheels]float64{0, 0, 0, 0}
	cfg.MomentArmM = 1e-12
	if _, err := NewKinematics(cfg); err == nil {
		t.Error("expected error for degenerate wheel configuration")
	}
}

// 計画 §7.3: ホットパスで 1 周期あたり 0 アロケーションを固定する。
func BenchmarkKinematicsHotPath(b *testing.B) {
	k, err := NewKinematics(DefaultGeometry())
	if err != nil {
		b.Fatal(err)
	}
	var sink float64
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		w := k.WheelFromBody(1.0, 0.5, 2.0)
		l := k.SlotsToLogical(w)
		vx, vy, om := k.BodyFromWheel(l)
		r := k.Residual(l, vx, vy, om)
		sink += r[0] + vx + vy + om
	}
	_ = sink
}

func TestKinematicsHotPathDoesNotAllocate(t *testing.T) {
	k := newTestKinematics(t)
	got := testing.AllocsPerRun(1000, func() {
		w := k.WheelFromBody(1.0, 0.5, 2.0)
		l := k.SlotsToLogical(w)
		vx, vy, om := k.BodyFromWheel(l)
		_ = k.Residual(l, vx, vy, om)
	})
	if got != 0 {
		t.Errorf("hot path allocated %v times per run, want 0", got)
	}
}
