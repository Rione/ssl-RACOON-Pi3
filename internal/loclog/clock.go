// Package loclog は計測ログの MCAP 書き出しを担う。
//
// RAVEN の foxglove/McapWriter.java と RecordingClock.java の設計を踏襲する
// (計画 §6.1)。Foxglove で RAVEN 側のログと並べて見られることに価値がある。
//
// このパッケージにビルドタグは付けない。開発 PC でそのままテストできる。
package loclog

import (
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// Clock は「壁時計の epoch」と「単調時計」を起動時に 1 度だけ組にして留め、
// 以降は単調差分だけを足す時計。
//
// 単調時計は NTP の step で飛ばないので時刻が巻き戻らない。一方で壁時計の
// アンカーを 1 つ持っておけば、読む側は RAVEN のログへ引き直せる (計画 §5.2)。
type Clock struct {
	// start は単調時計の読み値を内包した起点。Since() がその読み値を使う。
	start time.Time
	// epochWallNs は start と同じ瞬間の壁時計 [ns]。MCAP の metadata に書く。
	epochWallNs int64
}

// NewClock は現在時刻を起点として時計を作る。
func NewClock() *Clock {
	now := time.Now()
	return &Clock{start: now, epochWallNs: now.UnixNano()}
}

// NewClockAt はテスト用に起点を指定して時計を作る。
func NewClockAt(start time.Time) *Clock {
	return &Clock{start: start, epochWallNs: start.UnixNano()}
}

// Now は起点からの単調経過時間を返す。
func (c *Clock) Now() localization.Stamp {
	return localization.Stamp(time.Since(c.start))
}

// StampOf は time.Now() で取った時刻を Stamp へ写す。
//
// SPI 転送の直前・直後で取った time.Time をそのまま渡す用途。
// time.Time 自体は API 境界を越えさせない (マーシャルで単調時計の読み値が落ちる)。
func (c *Clock) StampOf(t time.Time) localization.Stamp {
	return localization.Stamp(t.Sub(c.start))
}

// Midpoint は 2 つの時刻の中点を Stamp で返す。
//
// SPI は Tx の直前と直後で時刻を取り、その中点を転送時刻とする (計画 §5.2)。
// time.Ticker の公称 8 ms は、受信が遅れると間隔を詰めたりティックを落としたり
// するので信用しない。
func (c *Clock) Midpoint(before, after time.Time) localization.Stamp {
	return c.StampOf(before) + localization.Stamp(after.Sub(before)/2)
}

// EpochWallNs は起点の壁時計 [ns] を返す。
func (c *Clock) EpochWallNs() int64 { return c.epochWallNs }

// WallNs は Stamp を壁時計 [ns] へ引き直す。MCAP のログ時刻に使う。
//
// 単調時計で刻んだ値に、起動時に 1 度だけ取ったアンカーを足すだけなので、
// 途中で NTP が壁時計を跳ばしてもログの時刻は単調のまま保たれる。
func (c *Clock) WallNs(s localization.Stamp) int64 {
	return c.epochWallNs + int64(s)
}
