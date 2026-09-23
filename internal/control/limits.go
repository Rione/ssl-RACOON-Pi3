package control

import (
	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
	"math"
)

// 速度制限を共有する。加速度制限は実測dtと機体設定の契約確定後に追加する。
func limitScalar(value, limit float64) float64 { return math.Max(-limit, math.Min(limit, value)) }

func limitVector(v localization.Vec2, limit float64) (localization.Vec2, bool) {
	speed := math.Hypot(v.X, v.Y)
	if !finite(v.X, v.Y, speed) {
		return localization.Vec2{}, false
	}
	if speed > limit {
		scale := limit / speed
		v.X *= scale
		v.Y *= scale
	}
	return v, true
}
