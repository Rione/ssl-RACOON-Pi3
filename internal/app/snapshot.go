package app

import (
	"sync"

	"github.com/Rione/ssl-RACOON-Pi3/internal/mw"
)

// SnapshotStore は値を短いロック区間でコピーする。UDP・ログI/Oはロック外。
// 配列/値だけのEstimateReportを使い、周期ごとのポインタ確保を不要にする。
type SnapshotStore struct {
	mu    sync.RWMutex
	value mw.EstimateReport
	valid bool
}

func (s *SnapshotStore) Store(value mw.EstimateReport) {
	s.mu.Lock()
	s.value, s.valid = value, true
	s.mu.Unlock()
}

func (s *SnapshotStore) Load() (mw.EstimateReport, bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.value, s.valid
}
