package app

import (
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/control"
	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi3/internal/mw"
	"github.com/Rione/ssl-RACOON-Pi3/internal/receive"
	"github.com/Rione/ssl-RACOON-Pi3/internal/supervisor"
)

// ControlCycle は周期1回の接続点。時刻・観測・計画は外から渡す。
// ハードウェアなしで再生できる。Runからはまだ呼ばない。
// Estimatorとこの構造体は同じ1本のgoroutineが所有すること。
// UARTの受信待ちやログ書込みで停止監視を止めない周期駆動は今後接続する。
type ControlCycle struct {
	Estimator      *localization.Estimator
	Feedback       *control.VelocityFeedback
	MaxEstimateAge time.Duration
}

// CycleInputの観測は時刻変換済み。到着した新標本だけを渡し、同じ標本を再投入しない。
// 各スライスは呼び出し中に変更しない。入力キューの容量・欠落監視はI/O側の責務。
type CycleInput struct {
	Now       localization.Stamp
	Mode      supervisor.Mode
	Emergency bool
	Plan      *receive.Plan
	Wheels    []localization.WheelSample
	IMU       []localization.ImuSample
	Vision    []localization.VisionPose
}

// Stepは自己位置推定→参照比較→追従を行い、送信とは独立した結果を返す。
// 旧速度モードの中継は既存receive/linkが担当し、このStepから指令を送らない。
// 現在AddImuは未実装。将来の融合では種類を跨いだ観測順序の契約も更新する。
func (c *ControlCycle) Step(in CycleInput) mw.EstimateReport {
	out := mw.EstimateReport{CycleStamp: in.Now, Mode: in.Mode, PlanID: in.Plan.ID(),
		Reason: supervisor.NotArmed, Estimate: localization.Estimate{Health: localization.HealthInvalid}}
	if c == nil || c.Estimator == nil {
		out.Reason = supervisor.EstimateUnavailable
		return out
	}
	for _, s := range in.Wheels {
		c.Estimator.AddWheel(s)
	}
	for _, s := range in.IMU {
		c.Estimator.AddImu(s)
	}
	for _, s := range in.Vision {
		c.Estimator.AddVision(s)
	}
	out.Estimate = c.Estimator.Current()
	if in.Mode != supervisor.LocalTracking {
		return out
	}
	out.Reason = supervisor.Check(in.Mode, in.Emergency, in.Now, in.Plan.ValidUntil(), out.Estimate, c.MaxEstimateAge)
	if out.Reason != supervisor.Ready {
		return out
	}
	// 原点の初期値を実測位置として使わない。車輪と絶対位置の初期観測を待つ。
	stats := c.Estimator.Stats()
	if stats.WheelUpdates == 0 || stats.VisionUpdates == 0 {
		out.Reason = supervisor.EstimateUnavailable
		return out
	}
	controller := in.Plan.Controller()
	if controller == nil {
		out.Reason = supervisor.Expired
		return out
	}
	var err error
	out.Command, out.Phase, err = controller.CalculateWithFeedback(out.Estimate, in.Now, c.Feedback)
	if err != nil {
		out.Reason = supervisor.CalculationFailed
		out.Command = control.Command{}
		return out
	}
	if out.Phase != control.Tracking {
		out.Reason = supervisor.TrajectoryInactive
		return out
	}
	ref, _ := controller.ReferenceAt(out.Estimate.Stamp)
	out.Error = control.Compare(ref, out.Estimate)
	return out
}
