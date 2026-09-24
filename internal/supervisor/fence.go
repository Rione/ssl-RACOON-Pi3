package supervisor

import (
	"math"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// Fence は「開始した場所からこの半径を出たら止める」枠。
//
// 試験のときに機体が場を横切って走り去らないための最後の歯止め。vision の位置で判定するので、
// vision が別の物を見ていると効かない。そのため WheelVisionCheck と組にして使う (§5-18)。
type Fence struct {
	Start  localization.Vec2
	Radius float64 // [m]
}

// Outside は今の位置が枠の外かを返す。半径が 0 以下なら枠を使わない。
func (f Fence) Outside(p localization.Vec2) (float64, bool) {
	if f.Radius <= 0 {
		return 0, false
	}
	d := math.Hypot(p.X-f.Start.X, p.Y-f.Start.Y)
	return d, d > f.Radius
}
