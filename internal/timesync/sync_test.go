package timesync

import (
	"math"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/localization"
)

func TestSecondsToStamp(t *testing.T) {
	cases := []struct {
		sec  float64
		want time.Duration
	}{
		{0, 0},
		{1, time.Second},
		{0.008, 8 * time.Millisecond},
		{1234.5, 1234*time.Second + 500*time.Millisecond},
	}
	for _, c := range cases {
		got := time.Duration(SecondsToStamp(c.sec))
		// double の秒はナノ秒まで正確には表せないので 1 us まで許す。
		if d := got - c.want; d > time.Microsecond || d < -time.Microsecond {
			t.Errorf("SecondsToStamp(%v) = %v, want %v", c.sec, got, c.want)
		}
	}
}

// 試合時間ぶん進んだ t_capture でも分解能が落ちないこと。
// SSL-Vision の t_capture は PC 起動からの秒なので、大きな値が来る。
func TestSecondsToStampKeepsResolutionAtLargeValues(t *testing.T) {
	const base = 86400.0 // 1 日ぶん
	a := SecondsToStamp(base)
	b := SecondsToStamp(base + 0.001)
	got := time.Duration(b - a)
	if math.Abs(float64(got-time.Millisecond)) > float64(time.Microsecond) {
		t.Errorf("1 ms apart at t=%v resolved as %v", base, got)
	}
}

func TestArrivalPassesThrough(t *testing.T) {
	var s Sync = Arrival{}
	remote := SecondsToStamp(1234.5)
	arrival := localization.Stamp(42 * time.Millisecond)

	s.Observe(remote, arrival)
	got, q := s.ToLocal(remote, arrival)

	if got != arrival {
		t.Errorf("ToLocal = %v, want the arrival stamp %v", got, arrival)
	}
	// Valid = false であることが重要。上位が「写像できていない」と判別でき、
	// vision_meta の mapped フラグに正直に出る。
	if q.Valid {
		t.Error("Arrival must report Valid = false so callers know the mapping is a fallback")
	}
	s.Reset()
}
