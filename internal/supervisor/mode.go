// Package supervisor は制御モードと有効期限の判定を計算・通信から分離する。
// 現在の実機Run経路は未接続。停止判定を既存リンクへ接続するまでは旧処理が動く。
package supervisor

type Mode uint8

const (
	Disarmed Mode = iota
	Legacy
	LocalTracking
	Stopped
	Fault
)

type Reason string

const (
	Ready               Reason = "ready"
	NotArmed            Reason = "not_armed"
	EmergencyStop       Reason = "emergency_stop"
	Expired             Reason = "command_expired"
	EstimateUnavailable Reason = "estimate_unavailable"
	CalculationFailed   Reason = "calculation_failed"
	TrajectoryInactive  Reason = "trajectory_inactive"
	// 以下は実機の走行で見つかった止めどころ (docs/traj-poc-log.md §5-18, 19)。
	OutOfFence          Reason = "out_of_fence"          // 開始位置から決めた半径を出た
	WheelVisionMismatch Reason = "wheel_vision_mismatch" // 車輪は動いたのに vision が動かない (模様の取り違え)
	LinkStale           Reason = "link_stale"            // STM から正しいフレームが届かない
	VisionLost          Reason = "vision_lost"           // vision が古い / 来ない
)
