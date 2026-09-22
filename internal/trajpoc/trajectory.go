// Package trajpoc は時刻つき軌道 (TimedPoint の列) をロボット上で追従する手法を
// 実機で比べるための PoC。RAVEN から降りてくる予定の軌道を、SSH の標準入力で
// 代わりに流し込み、手法ごとに速度を作って誤差を測る。
//
// このパッケージにはソケット・ハードウェア依存・time.Now() を持ち込まない
// (時刻と vision は呼び出し側が注入する)。内部単位は SI。
package trajpoc

import (
	"bufio"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"strings"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// WirePoint は標準入力で受け取る 1 点。中身は RAVEN の TimedTrajectoryPoint
// (x, y [mm], theta [rad], t [ns]) と同じだが、t だけは PC の絶対時刻ではなく
// 軌道の先頭からの相対時刻 [ns] にする (PC とロボットの時計は揃っていないため)。
type WirePoint struct {
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Theta float64 `json:"theta"`
	T     float64 `json:"t"`
}

// Knot は内部表現の 1 点 (SI)。T は軌道の先頭からの秒。
type Knot struct {
	T    float64
	Pose localization.Pose2
}

// EndMarker は点列の終わりを示す行。これを受け取ってから走り出す。
// EOF と区別するのは、SSH が切れたこと (= 止めるべき) と取り違えないため。
const EndMarker = "END"

// MaxPoints は 1 本の軌道の上限 (異常な入力で固まらないための安全弁)。
const MaxPoints = 20000

// ReadTrajectory は JSON Lines の点列を EndMarker の行まで読む。
// 空行と # で始まる行は無視する。EndMarker より前に EOF が来たらエラー。
func ReadTrajectory(r io.Reader) ([]Knot, error) {
	sc := bufio.NewScanner(r)
	sc.Buffer(make([]byte, 64*1024), 1024*1024)
	var knots []Knot
	line := 0
	for sc.Scan() {
		line++
		s := strings.TrimSpace(sc.Text())
		if s == "" || strings.HasPrefix(s, "#") {
			continue
		}
		if s == EndMarker {
			if err := validateKnots(knots); err != nil {
				return nil, err
			}
			return knots, nil
		}
		var p WirePoint
		if err := json.Unmarshal([]byte(s), &p); err != nil {
			return nil, fmt.Errorf("line %d: %w", line, err)
		}
		if len(knots) >= MaxPoints {
			return nil, fmt.Errorf("too many points (> %d)", MaxPoints)
		}
		knots = append(knots, Knot{
			T:    p.T * 1e-9,
			Pose: localization.Pose2{X: p.X / 1000, Y: p.Y / 1000, Theta: p.Theta},
		})
	}
	if err := sc.Err(); err != nil {
		return nil, err
	}
	return nil, fmt.Errorf("input ended before the %q line (%d points read)", EndMarker, len(knots))
}

func validateKnots(knots []Knot) error {
	if len(knots) < 2 {
		return fmt.Errorf("trajectory needs at least 2 points, got %d", len(knots))
	}
	if knots[0].T < 0 {
		return fmt.Errorf("first point has negative t")
	}
	for i, k := range knots {
		if !finite(k.T, k.Pose.X, k.Pose.Y, k.Pose.Theta) {
			return fmt.Errorf("point %d is not finite", i)
		}
		if i > 0 && k.T <= knots[i-1].T {
			return fmt.Errorf("point %d: t must strictly increase", i)
		}
	}
	return nil
}

// ToWorld は「開始時のロボット姿勢を原点とした相対座標」の軌道をワールド座標へ写す。
// PoC では軌道を相対で作り、走り出す瞬間の vision の姿勢に貼り付ける
// (どこに置いたロボットでも、その場から同じ形を走れるように)。
func ToWorld(rel []Knot, start localization.Pose2) []Knot {
	out := make([]Knot, len(rel))
	for i, k := range rel {
		v := localization.Rotate(start.Theta, localization.Vec2{X: k.Pose.X, Y: k.Pose.Y})
		out[i] = Knot{T: k.T, Pose: localization.Pose2{
			X:     start.X + v.X,
			Y:     start.Y + v.Y,
			Theta: localization.WrapAngle(start.Theta + k.Pose.Theta),
		}}
	}
	return out
}

// Bounds は相対軌道が安全の枠に収まっているかを走る前に調べるための量。
type Bounds struct {
	MaxRadius   float64 // 原点 (開始位置) からの最大距離 [m]
	MaxSpeed    float64 // 隣り合う点の差分から出る最大速度 [m/s]
	MaxYawRate  float64 // 同じく最大角速度 [rad/s]
	StartOffset float64 // 先頭の点と原点の距離 [m] (0 でないと走り出しで跳ぶ)
	Duration    float64 // [s]
}

// MeasureBounds は相対軌道の Bounds を返す。
func MeasureBounds(rel []Knot) Bounds {
	var b Bounds
	if len(rel) == 0 {
		return b
	}
	b.StartOffset = math.Hypot(rel[0].Pose.X, rel[0].Pose.Y)
	b.Duration = rel[len(rel)-1].T - rel[0].T
	for i, k := range rel {
		b.MaxRadius = math.Max(b.MaxRadius, math.Hypot(k.Pose.X, k.Pose.Y))
		if i == 0 {
			continue
		}
		p := rel[i-1]
		dt := k.T - p.T
		b.MaxSpeed = math.Max(b.MaxSpeed, math.Hypot(k.Pose.X-p.Pose.X, k.Pose.Y-p.Pose.Y)/dt)
		b.MaxYawRate = math.Max(b.MaxYawRate, math.Abs(localization.AngleDiff(k.Pose.Theta, p.Pose.Theta))/dt)
	}
	return b
}

func finite(values ...float64) bool {
	for _, v := range values {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return false
		}
	}
	return true
}
