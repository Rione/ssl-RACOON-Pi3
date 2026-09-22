package locsim

import (
	"math"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// 真値が間違っていたら下流の検証がすべて無意味になる。
// 「速度は位置の微分か」「ヨーレートは角度の微分か」を数値微分で直接確かめる。

func allTrajectories() []Trajectory {
	const d = 8 * time.Second
	fixed := HeadingParams{Mode: HeadingFixed, Theta0: 0.7}
	spin := HeadingParams{Mode: HeadingSpin, Theta0: -0.3, SpinRate: 1.8}
	tangent := HeadingParams{Mode: HeadingTangent}

	return []Trajectory{
		Stationary(localization.Vec2{X: 1, Y: -2}, fixed, d),
		Line(localization.Vec2{}, localization.Vec2{X: 2, Y: 0}, localization.Vec2{}, fixed, d),
		Line(localization.Vec2{X: -1, Y: 1}, localization.Vec2{X: 1, Y: 0.5},
			localization.Vec2{X: 0.3, Y: -0.2}, spin, d),
		Circle(localization.Vec2{}, 1.5, 2.0, fixed, d),
		Circle(localization.Vec2{X: 0.5, Y: 0.5}, 1.0, 1.5, tangent, d),
		Circle(localization.Vec2{}, 1.2, 1.8, spin, d),
		FigureEight(localization.Vec2{}, 1.5, 4*time.Second, fixed, d),
		FigureEight(localization.Vec2{}, 1.5, 4*time.Second, spin, d),
		StepAccel(localization.Vec2{}, 0.4, 3.0, 2.0, 2*time.Second, fixed, d),
	}
}

func TestTrajectoryVelocityIsDerivativeOfPosition(t *testing.T) {
	const h = 1 * time.Microsecond
	for _, tr := range allTrajectories() {
		t.Run(tr.Name()+"/"+headingName(tr), func(t *testing.T) {
			for tt := 500 * time.Millisecond; tt < tr.Duration()-500*time.Millisecond; tt += 137 * time.Millisecond {
				// StepAccel は加速度が不連続に変わる点があるので、その近傍は飛ばす。
				if nearDiscontinuity(tr, tt) {
					continue
				}
				a := tr.At(tt - h)
				b := tr.At(tt + h)
				mid := tr.At(tt)

				dt := 2 * h.Seconds()
				gotX := (b.Pose.X - a.Pose.X) / dt
				gotY := (b.Pose.Y - a.Pose.Y) / dt
				if math.Abs(gotX-mid.VelWorld.X) > 1e-4 || math.Abs(gotY-mid.VelWorld.Y) > 1e-4 {
					t.Errorf("t=%v: numeric velocity (%.6f, %.6f), analytic (%.6f, %.6f)",
						tt, gotX, gotY, mid.VelWorld.X, mid.VelWorld.Y)
				}
			}
		})
	}
}

func TestTrajectoryYawRateIsDerivativeOfTheta(t *testing.T) {
	const h = 1 * time.Microsecond
	for _, tr := range allTrajectories() {
		t.Run(tr.Name()+"/"+headingName(tr), func(t *testing.T) {
			for tt := 500 * time.Millisecond; tt < tr.Duration()-500*time.Millisecond; tt += 137 * time.Millisecond {
				if nearDiscontinuity(tr, tt) {
					continue
				}
				a := tr.At(tt - h)
				b := tr.At(tt + h)
				mid := tr.At(tt)

				// ±pi をまたぐので差は正規化してから割る。
				got := localization.AngleDiff(b.Pose.Theta, a.Pose.Theta) / (2 * h.Seconds())
				if math.Abs(got-mid.YawRate) > 1e-3 {
					t.Errorf("t=%v: numeric yaw rate %.6f, analytic %.6f", tt, got, mid.YawRate)
				}
			}
		})
	}
}

// VelBody は VelWorld をロボット系へ回したもの。
// ここが狂うと運動学の検証が丸ごと嘘になる。
func TestTrajectoryBodyVelocityMatchesWorld(t *testing.T) {
	for _, tr := range allTrajectories() {
		t.Run(tr.Name()+"/"+headingName(tr), func(t *testing.T) {
			for tt := time.Duration(0); tt < tr.Duration(); tt += 97 * time.Millisecond {
				s := tr.At(tt)
				back := localization.Rotate(s.Pose.Theta, s.VelBody)
				if math.Abs(back.X-s.VelWorld.X) > 1e-12 || math.Abs(back.Y-s.VelWorld.Y) > 1e-12 {
					t.Fatalf("t=%v: R(theta)*VelBody = (%v, %v), VelWorld = (%v, %v)",
						tt, back.X, back.Y, s.VelWorld.X, s.VelWorld.Y)
				}
			}
		})
	}
}

// 向きを固定した横移動では、機体系の vy がゼロにならないこと。
// 接線追従だけで検証すると vy が常に 0 になり、横方向の可観測性を試さないまま
// 「動いた」ことになってしまう。
func TestFixedHeadingProducesLateralMotion(t *testing.T) {
	tr := Line(localization.Vec2{}, localization.Vec2{X: 0, Y: 2}, localization.Vec2{},
		HeadingParams{Mode: HeadingFixed, Theta0: 0}, 2*time.Second)
	s := tr.At(time.Second)
	if math.Abs(s.VelBody.Y-2) > 1e-9 {
		t.Errorf("body vy = %v, want 2 (pure lateral motion)", s.VelBody.Y)
	}
	if math.Abs(s.VelBody.X) > 1e-9 {
		t.Errorf("body vx = %v, want 0", s.VelBody.X)
	}
	if s.YawRate != 0 {
		t.Errorf("yaw rate = %v, want 0", s.YawRate)
	}
}

// 接線追従は逆に vy をゼロに保つ。両方使えることを固定する。
func TestTangentHeadingKeepsLateralVelocityZero(t *testing.T) {
	tr := Circle(localization.Vec2{}, 1.5, 2.0, HeadingParams{Mode: HeadingTangent}, 4*time.Second)
	for tt := time.Duration(0); tt < tr.Duration(); tt += 50 * time.Millisecond {
		s := tr.At(tt)
		if math.Abs(s.VelBody.Y) > 1e-9 {
			t.Fatalf("t=%v: body vy = %v, want 0 for tangent heading", tt, s.VelBody.Y)
		}
		if math.Abs(s.VelBody.X-2.0) > 1e-9 {
			t.Fatalf("t=%v: body vx = %v, want the 2 m/s speed", tt, s.VelBody.X)
		}
	}
}

// 円軌道の接線追従では、ヨーレートが speed/radius に一致すること。
func TestCircleTangentYawRate(t *testing.T) {
	const radius, speed = 1.5, 2.0
	tr := Circle(localization.Vec2{}, radius, speed, HeadingParams{Mode: HeadingTangent}, 4*time.Second)
	want := speed / radius
	for tt := time.Duration(0); tt < tr.Duration(); tt += 50 * time.Millisecond {
		if got := tr.At(tt).YawRate; math.Abs(got-want) > 1e-9 {
			t.Fatalf("t=%v: yaw rate %v, want %v", tt, got, want)
		}
	}
}

// 8 の字は曲率の符号が反転する。円では隠れる誤差を出すための軌道なので、
// 実際に反転していることを確かめる。
func TestFigureEightReversesCurvature(t *testing.T) {
	tr := FigureEight(localization.Vec2{}, 1.5, 4*time.Second,
		HeadingParams{Mode: HeadingTangent}, 4*time.Second)
	var sawPositive, sawNegative bool
	for tt := time.Duration(0); tt < tr.Duration(); tt += 20 * time.Millisecond {
		switch w := tr.At(tt).YawRate; {
		case w > 0.2:
			sawPositive = true
		case w < -0.2:
			sawNegative = true
		}
	}
	if !sawPositive || !sawNegative {
		t.Error("the figure-eight never reversed its turn direction")
	}
}

// 急加減速は最後に止まること。止まらないとドリフト検証に使えない。
func TestStepAccelComesToRest(t *testing.T) {
	tr := StepAccel(localization.Vec2{}, 0, 3.0, 2.0, 2*time.Second,
		HeadingParams{Mode: HeadingFixed}, 8*time.Second)
	end := tr.At(tr.Duration())
	if math.Abs(end.VelBody.X) > 1e-9 || math.Abs(end.VelBody.Y) > 1e-9 {
		t.Errorf("final velocity %+v, want rest", end.VelBody)
	}
	// 最高速に達していること。
	peak := tr.At(2 * time.Second)
	if math.Abs(peak.VelBody.X-2.0) > 1e-9 {
		t.Errorf("cruise speed = %v, want 2.0", peak.VelBody.X)
	}
}

func TestStationaryStaysPut(t *testing.T) {
	at := localization.Vec2{X: 1, Y: -2}
	tr := Stationary(at, HeadingParams{Mode: HeadingFixed, Theta0: 0.5}, 10*time.Second)
	for tt := time.Duration(0); tt <= tr.Duration(); tt += time.Second {
		s := tr.At(tt)
		if s.Pose.X != at.X || s.Pose.Y != at.Y {
			t.Fatalf("t=%v: moved to %+v", tt, s.Pose)
		}
		if s.VelBody != (localization.Vec2{}) || s.YawRate != 0 {
			t.Fatalf("t=%v: non-zero motion %+v %v", tt, s.VelBody, s.YawRate)
		}
	}
}

func TestSample(t *testing.T) {
	tr := Line(localization.Vec2{}, localization.Vec2{X: 1}, localization.Vec2{},
		HeadingParams{}, time.Second)
	got := Sample(tr, 8*time.Millisecond)
	if len(got) != 126 { // 1000/8 = 125 区間 + 始点
		t.Errorf("got %d samples, want 126", len(got))
	}
	if got[0].Stamp != 0 {
		t.Errorf("first stamp = %v, want 0", got[0].Stamp)
	}
	if want := localization.Stamp(125 * 8 * time.Millisecond); got[len(got)-1].Stamp != want {
		t.Errorf("last stamp = %v, want %v", got[len(got)-1].Stamp, want)
	}
}

func headingName(tr Trajectory) string {
	a, ok := tr.(*analytic)
	if !ok {
		return "?"
	}
	switch a.heading.Mode {
	case HeadingSpin:
		return "spin"
	case HeadingTangent:
		return "tangent"
	default:
		return "fixed"
	}
}

// nearDiscontinuity は StepAccel の加速度が切り替わる時刻の近傍かを返す。
func nearDiscontinuity(tr Trajectory, t time.Duration) bool {
	a, ok := tr.(*analytic)
	if !ok {
		return false
	}
	p, ok := a.p.(stepAccelPath)
	if !ok {
		return false
	}
	const guard = 10 * time.Millisecond
	for _, edge := range []float64{0, p.tAccel, p.tAccel + p.tCruise, p.tAccel + p.tCruise + p.tDecel} {
		e := time.Duration(edge * float64(time.Second))
		if t > e-guard && t < e+guard {
			return true
		}
	}
	return false
}
