package timesync

import (
	"math"
	"math/rand"
	"sort"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/localization"
)

func TestMultiSyncKeepsCamerasIndependent(t *testing.T) {
	m := NewConvexHullProvider(Config{})

	// 2 台のカメラ。処理遅延が 25 ms 違う。
	fast := defaultScenario()
	slow := defaultScenario()
	slow.minDelay = fast.minDelay + 25*time.Millisecond

	rng := rand.New(rand.NewSource(3))
	for i := 0; i < 60*30; i++ {
		for id, s := range map[uint32]scenario{0: fast, 1: slow} {
			remote, arrival, _ := s.sample(i, rng)
			m.For(id).Observe(remote, arrival)
		}
	}

	st0, ok0 := m.StatsFor(0)
	st1, ok1 := m.StatsFor(1)
	if !ok0 || !ok1 || !st0.Valid || !st1.Valid {
		t.Fatalf("estimates not ready: %+v %+v", st0, st1)
	}

	// スキューは同じ (同じ PC のクロック)。
	for _, st := range []Stats{st0, st1} {
		if d := math.Abs(st.SkewPpm - fast.skewPpm); d > 10 {
			t.Errorf("skew = %.2f ppm, want %.2f", st.SkewPpm, fast.skewPpm)
		}
	}
	// 最小片方向遅延は 25 ms 違う。混ぜていたらこの差は潰れる。
	gap := time.Duration(st1.OffsetNs - st0.OffsetNs)
	if d := gap - 25*time.Millisecond; d < -2*time.Millisecond || d > 2*time.Millisecond {
		t.Errorf("per-camera delay gap = %v, want 25ms; the cameras are being mixed", gap)
	}
}

func TestMultiSyncCreatesOnDemand(t *testing.T) {
	m := NewMultiSync(nil)
	if got := m.Cameras(); len(got) != 0 {
		t.Errorf("a fresh MultiSync knows about %v", got)
	}
	m.For(3).Observe(0, 0)
	m.For(1).Observe(0, 0)
	m.For(3).Observe(0, 0) // 同じカメラは同じ推定器

	got := m.Cameras()
	sort.Slice(got, func(i, j int) bool { return got[i] < got[j] })
	if len(got) != 2 || got[0] != 1 || got[1] != 3 {
		t.Errorf("Cameras() = %v, want [1 3]", got)
	}
	if m.For(3) != m.For(3) {
		t.Error("For() handed out two different estimators for the same camera")
	}
}

func TestMultiSyncResetAll(t *testing.T) {
	m := NewConvexHullProvider(Config{})
	s := defaultScenario()
	rng := rand.New(rand.NewSource(4))
	for i := 0; i < 60*30; i++ {
		remote, arrival, _ := s.sample(i, rng)
		m.For(0).Observe(remote, arrival)
	}
	if st, _ := m.StatsFor(0); !st.Valid {
		t.Fatal("estimate never became valid")
	}
	m.ResetAll()
	if st, _ := m.StatsFor(0); st.Valid || st.Samples != 0 {
		t.Errorf("ResetAll left state behind: %+v", st)
	}
}

func TestArrivalProvider(t *testing.T) {
	var p Provider = ArrivalProvider{}
	arrival := localization.Stamp(999)
	got, q := p.For(7).ToLocal(123, arrival)
	if got != arrival || q.Valid {
		t.Errorf("ArrivalProvider gave (%v, %+v), want the arrival stamp and Valid=false", got, q)
	}
}

func TestStatsForUnknownCamera(t *testing.T) {
	m := NewConvexHullProvider(Config{})
	if _, ok := m.StatsFor(42); ok {
		t.Error("StatsFor reported stats for a camera that was never seen")
	}
}
