package supervisor

import (
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// Check は同じ単調時計で指令期限と推定状態の鮮度を判定する。
// 停止への減速則、復帰条件、センサー個別の期限は未確定。自動復帰や減速を装わない。
// EstimatorのHealthは観測欠測も反映する必要があり、Stampだけでは判断できない。
func Check(mode Mode, emergency bool, now, validUntil localization.Stamp, estimate localization.Estimate, maxAge time.Duration) Reason {
	if emergency {
		return EmergencyStop
	}
	if mode != Legacy && mode != LocalTracking {
		return NotArmed
	}
	if now < 0 || validUntil <= now {
		return Expired
	}
	if mode == LocalTracking && (maxAge <= 0 || estimate.Stamp < 0 || estimate.Stamp > now || now.Sub(estimate.Stamp) > maxAge ||
		(estimate.Health != localization.HealthOK && estimate.Health != localization.HealthDegraded)) {
		return EstimateUnavailable
	}
	return Ready
}
