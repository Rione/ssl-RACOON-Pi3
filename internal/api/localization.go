package api

import (
	"encoding/json"
	"net"
	"sync/atomic"
)

// 自己位置推定の状態を HTTP から見るための口。
//
// **実験中に「今どうなっているか」を機体の外から見えるようにするため**にある。
// 推定器は internal/locadapter にあり、そこはビルドタグ付きの層 (internal/app) から
// 起動される。api がそれを直接 import すると向きが逆になるので、
// **提供側 (app) が関数を差し込む**形にしてある。
//
// 差し込まれていなければ「動いていない」と返すだけで、既存の動作は変わらない。

// LocalizationSnapshot は /localization が返す中身。
//
// 迷ったら Status の 1 行を読めばよい。数字は後から追うため。
type LocalizationSnapshot struct {
	// Running は推定器が回っているか。
	Running bool `json:"running"`
	// Status は人が読む 1 行。
	Status string `json:"status"`
	// Warnings は「見たらすぐ手を打つべき」こと。空なら異常なし。
	Warnings []string `json:"warnings"`
	// Estimate は推定の中身 (loclog.EstimateRecord と同じ形)。
	Estimate any `json:"estimate,omitempty"`
	// Stats は累積統計 (loclog.EstimatorStatsRecord と同じ形)。
	Stats any `json:"stats,omitempty"`
	// ActuationDelayMs は測った「指令が効くまでの遅れ」。0 ならまだ決まっていない。
	ActuationDelayMs float64 `json:"actuationDelay_ms"`
}

var localizationProvider atomic.Pointer[func() LocalizationSnapshot]

// SetLocalizationProvider は /localization が呼ぶ関数を差し込む。nil で解除。
func SetLocalizationProvider(fn func() LocalizationSnapshot) {
	if fn == nil {
		localizationProvider.Store(nil)
		return
	}
	localizationProvider.Store(&fn)
}

func handleLocalization(conn net.Conn) {
	snap := LocalizationSnapshot{
		Status: "the estimator is not running (start with -locestimate)",
	}
	if p := localizationProvider.Load(); p != nil {
		snap = (*p)()
	}
	body, err := json.MarshalIndent(snap, "", "  ")
	if err != nil {
		sendErrorResponse(conn, 500)
		return
	}
	sendHTTPResponse(conn, 200, "application/json", string(body)+"\n")
}
