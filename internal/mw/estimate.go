package mw

import (
	"github.com/Rione/ssl-RACOON-Pi3/internal/control"
	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi3/internal/supervisor"
)

// EstimateReport は計算側から送信側へ渡す値コピー可能なスナップショット。
// 既存PiToMwの送信は変更していない。protobufの追加欄・RAVEN側の受信が
// 合意できたら、この値を送信goroutineでエンコードする（計算側では行わない）。
// Commandは計算出力であり、実際にSTMへ送信済みという意味ではない。
type EstimateReport struct {
	CycleStamp localization.Stamp
	Estimate   localization.Estimate
	PlanID     uint64
	Mode       supervisor.Mode
	Reason     supervisor.Reason
	Phase      control.Phase
	Error      control.TrackingError
	Command    control.Command
}
