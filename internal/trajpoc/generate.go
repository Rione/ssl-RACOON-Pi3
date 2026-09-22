package trajpoc

import (
	"fmt"
	"io"
	"math"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// GenConfig は仮想軌道の作り方。座標は開始位置が原点・ロボットの前方が +x の相対。
type GenConfig struct {
	Shape   string  // line | square | circle | fig8 | turn
	Size    float64 // 形の大きさ [m] (line は往復の片道、square は一辺、circle は直径、fig8 は輪 1 つの直径、turn は回す角度 [rad])
	Speed   float64 // 巡航速度 [m/s] (turn は角速度 [rad/s])
	Accel   float64 // 加減速度と、曲がるときの横加速度の上限 [m/s^2] (turn は角加速度 [rad/s^2])
	Dt      float64 // 点の間隔 [s] (RAVEN は 60 Hz = 約 16 ms)
	Heading string  // fixed (向きを変えない) | tangent (進行方向を向く。circle/fig8 のみ)
	Laps    int
}

// DefaultGenConfig は最初の実機試験用。
func DefaultGenConfig() GenConfig {
	return GenConfig{Shape: "line", Size: 0.6, Speed: 0.4, Accel: 1.0, Dt: 0.016, Heading: "fixed", Laps: 1}
}

// leg は静止から静止までの 1 区間。pts は細かく刻んだ折れ線、radius は最小曲率半径 (直線は +Inf)。
type leg struct {
	pts    []localization.Vec2
	radius float64
}

// Generate は仮想軌道を作る。先頭は原点・t=0。
func Generate(c GenConfig) ([]Knot, error) {
	if !(c.Size > 0 && c.Speed > 0 && c.Accel > 0 && c.Dt > 0) || c.Laps < 1 {
		return nil, fmt.Errorf("size, speed, accel, dt must be positive and laps >= 1")
	}
	if c.Heading != "fixed" && c.Heading != "tangent" {
		return nil, fmt.Errorf("unknown heading %q (fixed|tangent)", c.Heading)
	}
	if c.Shape == "turn" {
		return generateTurn(c), nil
	}
	var one []leg
	switch c.Shape {
	case "line":
		a, b := localization.Vec2{}, localization.Vec2{X: c.Size}
		one = []leg{straight(a, b), straight(b, a)}
	case "square":
		s := c.Size
		p := []localization.Vec2{{}, {X: s}, {X: s, Y: s}, {Y: s}, {}}
		for i := 1; i < len(p); i++ {
			one = append(one, straight(p[i-1], p[i]))
		}
	case "circle":
		one = []leg{arcLeg(c.Size/2, 1)}
	case "fig8":
		// 原点で接する 2 つの円。左回りで上の輪、続けて右回りで下の輪。原点では向きも速度も連続。
		up, down := arcLeg(c.Size/2, 1), arcLeg(c.Size/2, -1)
		one = []leg{{pts: append(up.pts, down.pts[1:]...), radius: c.Size / 2}}
	default:
		return nil, fmt.Errorf("unknown shape %q (line|square|circle|fig8|turn)", c.Shape)
	}
	if c.Heading == "tangent" && (c.Shape == "line" || c.Shape == "square") {
		return nil, fmt.Errorf("heading=tangent needs a smooth path (circle|fig8): %s turns in place at its corners", c.Shape)
	}
	var legs []leg
	for i := 0; i < c.Laps; i++ {
		legs = append(legs, one...)
	}

	knots := []Knot{{T: 0}}
	t0 := 0.0
	theta := 0.0
	for _, l := range legs {
		s, cum := arcLengths(l.pts)
		vmax := math.Min(c.Speed, math.Sqrt(c.Accel*l.radius))
		dur := profileDuration(s, vmax, c.Accel)
		n := int(math.Ceil(dur / c.Dt))
		for k := 1; k <= n; k++ {
			tl := math.Min(float64(k)*c.Dt, dur)
			p, dir := pointAt(l.pts, cum, profileDistance(tl, s, vmax, c.Accel))
			if c.Heading == "tangent" {
				theta += localization.AngleDiff(math.Atan2(dir.Y, dir.X), theta)
			}
			knots = append(knots, Knot{T: t0 + tl, Pose: localization.Pose2{X: p.X, Y: p.Y, Theta: theta}})
		}
		t0 += dur
	}
	return knots, nil
}

// generateTurn はその場で +Size [rad] 回って 0 に戻る (位置は原点のまま)。角速度の指令と実際の
// 回り方を比べる開ループの試験に使う (-trajkth 0 で向きの P を切り、参照の角速度だけで回す)。
func generateTurn(c GenConfig) []Knot {
	knots := []Knot{{T: 0}}
	t0 := 0.0
	for _, dir := range []float64{1, -1} {
		dur := profileDuration(c.Size, c.Speed, c.Accel)
		n := int(math.Ceil(dur / c.Dt))
		start := knots[len(knots)-1].Pose.Theta
		for k := 1; k <= n; k++ {
			tl := math.Min(float64(k)*c.Dt, dur)
			knots = append(knots, Knot{T: t0 + tl, Pose: localization.Pose2{Theta: start + dir*profileDistance(tl, c.Size, c.Speed, c.Accel)}})
		}
		t0 += dur
	}
	return knots
}

// WriteJSONL は軌道を標準入力へ流す形式で書く (最後に EndMarker)。
func WriteJSONL(w io.Writer, knots []Knot, comment string) error {
	if comment != "" {
		if _, err := fmt.Fprintf(w, "# %s\n", comment); err != nil {
			return err
		}
	}
	for _, k := range knots {
		if _, err := fmt.Fprintf(w, "{\"x\":%.2f,\"y\":%.2f,\"theta\":%.5f,\"t\":%.0f}\n",
			k.Pose.X*1000, k.Pose.Y*1000, k.Pose.Theta, k.T*1e9); err != nil {
			return err
		}
	}
	_, err := fmt.Fprintln(w, EndMarker)
	return err
}

func straight(a, b localization.Vec2) leg {
	n := int(math.Max(2, math.Ceil(math.Hypot(b.X-a.X, b.Y-a.Y)/0.005)))
	pts := make([]localization.Vec2, n+1)
	for i := range pts {
		u := float64(i) / float64(n)
		pts[i] = localization.Vec2{X: a.X + u*(b.X-a.X), Y: a.Y + u*(b.Y-a.Y)}
	}
	return leg{pts: pts, radius: math.Inf(1)}
}

// arcLeg は原点から +x 方向へ出て 1 周する半径 r の円。dir=+1 で左回り (中心 (0,r))、-1 で右回り。
func arcLeg(r float64, dir float64) leg {
	n := int(math.Max(16, math.Ceil(2*math.Pi*r/0.005)))
	pts := make([]localization.Vec2, n+1)
	for i := range pts {
		a := 2 * math.Pi * float64(i) / float64(n)
		pts[i] = localization.Vec2{X: r * math.Sin(a), Y: dir * r * (1 - math.Cos(a))}
	}
	return leg{pts: pts, radius: r}
}

func arcLengths(pts []localization.Vec2) (float64, []float64) {
	cum := make([]float64, len(pts))
	for i := 1; i < len(pts); i++ {
		cum[i] = cum[i-1] + math.Hypot(pts[i].X-pts[i-1].X, pts[i].Y-pts[i-1].Y)
	}
	return cum[len(cum)-1], cum
}

// pointAt は弧長 d の位置と進行方向 (単位ベクトル)。
func pointAt(pts []localization.Vec2, cum []float64, d float64) (localization.Vec2, localization.Vec2) {
	i := 1
	for i < len(cum)-1 && cum[i] < d {
		i++
	}
	a, b := pts[i-1], pts[i]
	seg := cum[i] - cum[i-1]
	u := 0.0
	if seg > 0 {
		u = math.Max(0, math.Min(1, (d-cum[i-1])/seg))
	}
	dir := localization.Vec2{X: (b.X - a.X) / seg, Y: (b.Y - a.Y) / seg}
	return localization.Vec2{X: a.X + u*(b.X-a.X), Y: a.Y + u*(b.Y-a.Y)}, dir
}

// 静止から静止への台形 (距離が短ければ三角) の速度プロファイル。
func profileDuration(s, vmax, a float64) float64 {
	if s <= vmax*vmax/a {
		return 2 * math.Sqrt(s/a)
	}
	return s/vmax + vmax/a
}

func profileDistance(t, s, vmax, a float64) float64 {
	if s <= vmax*vmax/a {
		vp := math.Sqrt(s * a)
		th := vp / a
		if t <= th {
			return 0.5 * a * t * t
		}
		td := t - th
		return math.Min(s, s/2+vp*td-0.5*a*td*td)
	}
	ta := vmax / a
	tc := s/vmax - vmax/a
	switch {
	case t <= ta:
		return 0.5 * a * t * t
	case t <= ta+tc:
		return 0.5*vmax*ta + vmax*(t-ta)
	default:
		td := t - ta - tc
		return math.Min(s, 0.5*vmax*ta+vmax*tc+vmax*td-0.5*a*td*td)
	}
}
