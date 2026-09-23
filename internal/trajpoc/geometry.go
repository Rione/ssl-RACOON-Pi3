package trajpoc

import "github.com/Rione/ssl-RACOON-Pi3/internal/localization"

// PoCGeometry はテスト機 (racoon-56011) の機体パラメータ。config/geometry-racoon-56011.json と同じ値
// (docs/traj-poc-log.md §5-20: 4 輪とも符号 -1、取付角・半径は vision の速度を基準にした同定、
// 腕の長さは vision を抜いたリプレイで向きのずれが最小になる値)。
// 他の機体では測り直すこと (同じ手順で 6 本走らせて traj_locreplay -ident)。
func PoCGeometry() localization.GeometryConfig {
	g := localization.DefaultGeometry()
	g.WheelAnglesDeg = [localization.NumWheels]float64{55.4, 136.1, -136.3, -57.4}
	g.WheelRadiusM = [localization.NumWheels]float64{0.02933, 0.02803, 0.02818, 0.02805}
	g.MomentArmM = 0.074
	g.WheelSigns = [localization.NumWheels]float64{-1, -1, -1, -1}
	return g
}
