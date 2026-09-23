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
)
