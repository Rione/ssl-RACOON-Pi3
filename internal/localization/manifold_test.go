package localization

import (
	"math"
	"testing"
)

func TestWrapAngle(t *testing.T) {
	cases := []struct{ in, want float64 }{
		{0, 0},
		{math.Pi, math.Pi},
		{-math.Pi, math.Pi},
		{math.Pi + 0.1, -math.Pi + 0.1},
		{3 * math.Pi, math.Pi},
		{-3*math.Pi - 0.1, math.Pi - 0.1},
		{100 * math.Pi, 0},
	}
	for _, c := range cases {
		got := WrapAngle(c.in)
		if math.Abs(AngleDiff(got, c.want)) > 1e-9 {
			t.Errorf("WrapAngle(%v) = %v, want %v", c.in, got, c.want)
		}
		if got <= -math.Pi || got > math.Pi {
			t.Errorf("WrapAngle(%v) = %v is outside (-pi, pi]", c.in, got)
		}
	}
}

func TestRotateRoundTrip(t *testing.T) {
	v := Vec2{X: 1.5, Y: -0.25}
	for _, th := range []float64{0, 0.3, -2.0, math.Pi, 5.5} {
		back := RotateInv(th, Rotate(th, v))
		if math.Abs(back.X-v.X) > 1e-12 || math.Abs(back.Y-v.Y) > 1e-12 {
			t.Errorf("theta=%v: round trip gave %+v, want %+v", th, back, v)
		}
	}
}

func TestExpLogSE2RoundTrip(t *testing.T) {
	// 通常域と、テイラー展開へ落ちる微小域の両方を通す。
	for _, w := range []float64{0, 1e-12, 1e-9, 1e-6, 0.01, 0.5, 3.0, -2.5} {
		for _, v := range [][2]float64{{0, 0}, {0.1, 0}, {0, 0.2}, {-0.3, 0.4}} {
			p := ExpSE2(w, v[0], v[1])
			gw, gx, gy := LogSE2(p)
			if math.Abs(AngleDiff(gw, w)) > 1e-9 {
				t.Errorf("w=%v v=%v: omega round trip %v", w, v, gw)
			}
			if math.Abs(gx-v[0]) > 1e-9 || math.Abs(gy-v[1]) > 1e-9 {
				t.Errorf("w=%v v=%v: trans round trip (%v, %v)", w, v, gx, gy)
			}
		}
	}
}

// ExpSE2 が定速旋回の厳密解になっていることを、細かく刻んだ前進積分と突き合わせる。
func TestExpSE2MatchesFineIntegration(t *testing.T) {
	const (
		omega = 2.0 // rad/s
		vx    = 1.0 // m/s
		vy    = 0.3
		dt    = 0.008
		steps = 20000
	)
	// 中点法で刻む。単純な前進 Euler だと積分側の誤差が 1e-9 残り、
	// ExpSE2 が厳密解かどうかを判定できない。
	var fine Pose2
	h := dt / steps
	for i := 0; i < steps; i++ {
		d := Rotate(fine.Theta+omega*h/2, Vec2{X: vx * h, Y: vy * h})
		fine.X += d.X
		fine.Y += d.Y
		fine.Theta = WrapAngle(fine.Theta + omega*h)
	}
	closed := ExpSE2(omega*dt, vx*dt, vy*dt)
	if math.Abs(closed.X-fine.X) > 1e-12 || math.Abs(closed.Y-fine.Y) > 1e-12 {
		t.Errorf("ExpSE2 = %+v, fine integration = %+v", closed, fine)
	}
}

func TestComposeBetween(t *testing.T) {
	a := Pose2{X: 1, Y: 2, Theta: 0.7}
	b := Pose2{X: -0.5, Y: 3, Theta: -2.9}
	rel := Between(a, b)
	got := Compose(a, rel)
	if math.Abs(got.X-b.X) > 1e-12 || math.Abs(got.Y-b.Y) > 1e-12 ||
		math.Abs(AngleDiff(got.Theta, b.Theta)) > 1e-12 {
		t.Errorf("Compose(a, Between(a, b)) = %+v, want %+v", got, b)
	}
}
