package trajpoc

import (
	"fmt"
	"math"
	"sort"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// Interp は点列の間を埋める方法。TimedPoint は位置と時刻しか持たないので、
// 速度は受け手が作るしかない。その作り方が追従の質を左右する (比べたい軸の 1 つ)。
type Interp string

const (
	// InterpLinear は隣り合う点を直線で結ぶ。速度は区間ごとに一定の階段になり、
	// 点ごとに参照加速度が跳ねる (RAVEN の OC が差分で速度を作るのと同じ)。
	InterpLinear Interp = "linear"
	// InterpHermite は各点の速度を前後の点の中心差分で決め、3 次 Hermite で結ぶ
	// (Catmull-Rom)。位置も速度も連続になる。
	InterpHermite Interp = "hermite"
)

// ParseInterp は文字列を Interp にする。
func ParseInterp(s string) (Interp, error) {
	switch Interp(s) {
	case InterpLinear, InterpHermite:
		return Interp(s), nil
	}
	return "", fmt.Errorf("unknown interpolation %q (linear|hermite)", s)
}

// Phase は参照の時刻が軌道のどこにあるか。
type Phase uint8

const (
	Before Phase = iota // 先頭より前: 先頭の点で静止
	During
	After // 末尾より後: 末尾の点で静止 (位置保持)
)

// RefSample はある時刻の参照 (ワールド系、SI)。
type RefSample struct {
	Pos     localization.Vec2
	Vel     localization.Vec2
	Theta   float64
	YawRate float64
	Phase   Phase
}

// Reference はワールド座標の点列を時刻で引けるようにしたもの。不変。
type Reference struct {
	knots  []Knot
	interp Interp
	// 角度は巻き戻して連続にしてから補間する (±π を跨いでも回り込まない)。
	theta []float64
	// Hermite 用の各点の速度 (中心差分)。
	vel   []localization.Vec2
	omega []float64
}

// NewReference は点列から Reference を作る。knots はワールド座標。
func NewReference(knots []Knot, interp Interp) (*Reference, error) {
	if err := validateKnots(knots); err != nil {
		return nil, err
	}
	n := len(knots)
	r := &Reference{knots: append([]Knot(nil), knots...), interp: interp,
		theta: make([]float64, n), vel: make([]localization.Vec2, n), omega: make([]float64, n)}
	r.theta[0] = knots[0].Pose.Theta
	for i := 1; i < n; i++ {
		r.theta[i] = r.theta[i-1] + localization.AngleDiff(knots[i].Pose.Theta, knots[i-1].Pose.Theta)
	}
	for i := 0; i < n; i++ {
		// 端は片側差分。軌道の端は静止しているのが普通なので、生成側が
		// 同じ点を並べていれば 0 になる。
		a, b := i-1, i+1
		if a < 0 {
			a = 0
		}
		if b >= n {
			b = n - 1
		}
		dt := knots[b].T - knots[a].T
		r.vel[i] = localization.Vec2{
			X: (knots[b].Pose.X - knots[a].Pose.X) / dt,
			Y: (knots[b].Pose.Y - knots[a].Pose.Y) / dt,
		}
		r.omega[i] = (r.theta[b] - r.theta[a]) / dt
	}
	return r, nil
}

// Duration は先頭から末尾までの秒。
func (r *Reference) Duration() float64 { return r.knots[len(r.knots)-1].T - r.knots[0].T }

// End は末尾の時刻 [s]。
func (r *Reference) End() float64 { return r.knots[len(r.knots)-1].T }

// Knots は点列のコピーを返す (指標の計算で経路との距離を測るのに使う)。
func (r *Reference) Knots() []Knot { return append([]Knot(nil), r.knots...) }

// At は軌道の時刻 t [s] の参照を返す。範囲外は端の点で静止した参照。
func (r *Reference) At(t float64) RefSample {
	n := len(r.knots)
	if t <= r.knots[0].T {
		k := r.knots[0]
		return RefSample{Pos: localization.Vec2{X: k.Pose.X, Y: k.Pose.Y}, Theta: k.Pose.Theta, Phase: Before}
	}
	if t >= r.knots[n-1].T {
		k := r.knots[n-1]
		return RefSample{Pos: localization.Vec2{X: k.Pose.X, Y: k.Pose.Y}, Theta: k.Pose.Theta, Phase: After}
	}
	i := sort.Search(n, func(i int) bool { return r.knots[i].T > t }) - 1
	a, b := r.knots[i], r.knots[i+1]
	h := b.T - a.T
	u := (t - a.T) / h
	var s RefSample
	s.Phase = During
	switch r.interp {
	case InterpHermite:
		s.Pos.X, s.Vel.X = hermite(a.Pose.X, b.Pose.X, r.vel[i].X, r.vel[i+1].X, h, u)
		s.Pos.Y, s.Vel.Y = hermite(a.Pose.Y, b.Pose.Y, r.vel[i].Y, r.vel[i+1].Y, h, u)
		var th float64
		th, s.YawRate = hermite(r.theta[i], r.theta[i+1], r.omega[i], r.omega[i+1], h, u)
		s.Theta = localization.WrapAngle(th)
	default:
		s.Pos = localization.Vec2{X: a.Pose.X + u*(b.Pose.X-a.Pose.X), Y: a.Pose.Y + u*(b.Pose.Y-a.Pose.Y)}
		s.Vel = localization.Vec2{X: (b.Pose.X - a.Pose.X) / h, Y: (b.Pose.Y - a.Pose.Y) / h}
		s.Theta = localization.WrapAngle(r.theta[i] + u*(r.theta[i+1]-r.theta[i]))
		s.YawRate = (r.theta[i+1] - r.theta[i]) / h
	}
	return s
}

// hermite は 3 次 Hermite の値と時間微分。p0,p1 は端の値、m0,m1 は端の時間微分、
// h は区間の秒、u は区間内の位置 [0,1]。
func hermite(p0, p1, m0, m1, h, u float64) (value, deriv float64) {
	u2, u3 := u*u, u*u*u
	h00 := 2*u3 - 3*u2 + 1
	h10 := u3 - 2*u2 + u
	h01 := -2*u3 + 3*u2
	h11 := u3 - u2
	value = h00*p0 + h10*h*m0 + h01*p1 + h11*h*m1
	d00 := 6*u2 - 6*u
	d10 := 3*u2 - 4*u + 1
	d01 := -6*u2 + 6*u
	d11 := 3*u2 - 2*u
	deriv = (d00*p0+d01*p1)/h + d10*m0 + d11*m1
	return value, deriv
}

// DistanceToPath は点 p から、点列を結んだ折れ線までの最短距離 [m]。
// 輪郭誤差 (時刻を無視して、経路からどれだけ外れたか) に使う。
func DistanceToPath(knots []Knot, p localization.Vec2) float64 {
	best := math.Inf(1)
	for i := 1; i < len(knots); i++ {
		a := localization.Vec2{X: knots[i-1].Pose.X, Y: knots[i-1].Pose.Y}
		b := localization.Vec2{X: knots[i].Pose.X, Y: knots[i].Pose.Y}
		best = math.Min(best, pointSegment(p, a, b))
	}
	if len(knots) == 1 {
		best = math.Hypot(p.X-knots[0].Pose.X, p.Y-knots[0].Pose.Y)
	}
	return best
}

func pointSegment(p, a, b localization.Vec2) float64 {
	dx, dy := b.X-a.X, b.Y-a.Y
	l2 := dx*dx + dy*dy
	if l2 < 1e-18 {
		return math.Hypot(p.X-a.X, p.Y-a.Y)
	}
	u := ((p.X-a.X)*dx + (p.Y-a.Y)*dy) / l2
	u = math.Max(0, math.Min(1, u))
	return math.Hypot(p.X-(a.X+u*dx), p.Y-(a.Y+u*dy))
}
