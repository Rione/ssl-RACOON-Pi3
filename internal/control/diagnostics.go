package control

import "github.com/Rione/ssl-RACOON-Pi3/internal/localization"

// TrackingError は同じ時刻の目標と推定の差。world系 [m, m/s, rad, rad/s]。
// 自己位置推定を目標へ引き寄せず、比較結果を独立して報告する。
type TrackingError struct {
	Position localization.Vec2
	Velocity localization.Vec2
	Heading  float64
	YawRate  float64
}

func Compare(ref Reference, estimate localization.Estimate) TrackingError {
	v := localization.Rotate(estimate.Pose.Theta, estimate.VelBody)
	return TrackingError{
		Position: localization.Vec2{X: ref.Pose.X - estimate.Pose.X, Y: ref.Pose.Y - estimate.Pose.Y},
		Velocity: localization.Vec2{X: ref.VelWorld.X - v.X, Y: ref.VelWorld.Y - v.Y},
		Heading:  localization.AngleDiff(ref.Pose.Theta, estimate.Pose.Theta),
		YawRate:  ref.YawRate - estimate.YawRate,
	}
}
