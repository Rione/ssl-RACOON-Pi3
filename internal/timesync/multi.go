package timesync

import "sync"

// Provider はカメラごとのクロック推定器を貸し出す。
//
// SSL-Vision は camera_id ごとに別の取り込みパイプラインを回しており、
// 露光から送信までの処理遅延がカメラごとに違う。1 つの推定器に混ぜると
// 凸包の下端が一番速いカメラに引きずられ、他のカメラの観測に系統誤差が乗る。
// そのため camera_id ごとに独立した状態を持つ (計画 §5.3)。
type Provider interface {
	// For は camera_id に対応する推定器を返す。
	For(cameraID uint32) Sync
}

// ArrivalProvider はどのカメラにも Arrival を返す。P1 (計測フェーズ) の既定。
type ArrivalProvider struct{}

// For は Arrival を返す。
func (ArrivalProvider) For(uint32) Sync { return Arrival{} }

// MultiSync は camera_id ごとに推定器を持つ。
//
// 推定器の生成を関数で受けるので、凸包でも Arrival でもテスト用の偽物でも束ねられる。
type MultiSync struct {
	newSync func() Sync

	mu       sync.Mutex
	byCamera map[uint32]Sync
}

// NewMultiSync はカメラごとの推定器を束ねる。
func NewMultiSync(newSync func() Sync) *MultiSync {
	if newSync == nil {
		newSync = func() Sync { return Arrival{} }
	}
	return &MultiSync{newSync: newSync, byCamera: make(map[uint32]Sync)}
}

// NewConvexHullProvider は凸包推定器をカメラごとに持つ Provider を作る。
func NewConvexHullProvider(cfg Config) *MultiSync {
	return NewMultiSync(func() Sync { return NewConvexHull(cfg) })
}

// For は camera_id に対応する推定器を返す。初出のカメラなら作る。
func (m *MultiSync) For(cameraID uint32) Sync {
	m.mu.Lock()
	defer m.mu.Unlock()
	s, ok := m.byCamera[cameraID]
	if !ok {
		s = m.newSync()
		m.byCamera[cameraID] = s
	}
	return s
}

// Cameras はこれまでに観測した camera_id を返す。
func (m *MultiSync) Cameras() []uint32 {
	m.mu.Lock()
	defer m.mu.Unlock()
	out := make([]uint32, 0, len(m.byCamera))
	for id := range m.byCamera {
		out = append(out, id)
	}
	return out
}

// ResetAll は全カメラの推定を捨てる。
func (m *MultiSync) ResetAll() {
	m.mu.Lock()
	defer m.mu.Unlock()
	for _, s := range m.byCamera {
		s.Reset()
	}
}

// StatsFor は camera_id の推定状態を返す。凸包以外なら false。
func (m *MultiSync) StatsFor(cameraID uint32) (Stats, bool) {
	m.mu.Lock()
	s, ok := m.byCamera[cameraID]
	m.mu.Unlock()
	if !ok {
		return Stats{}, false
	}
	ch, ok := s.(*ConvexHull)
	if !ok {
		return Stats{}, false
	}
	return ch.Stats(), true
}
