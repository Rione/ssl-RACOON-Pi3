//go:build pi4 || rock5a

package link

import (
	"sync/atomic"
	"time"
)

// 自己位置推定の計測基盤 (計画 P1) をリンク層へ差し込むためのフック。
//
// 既存の送受信処理は変えない。記録は観測に徹し、記録側の不具合が
// 走行機能を巻き込まないようにする。フックが登録されていなければ
// これまでと完全に同じ動きになる。

// SPIObserver は SPI 1 トランザクションを受け取る。
//
// before / after は conn.Tx() の直前・直後に取った時刻。その中点を
// 転送時刻とする。time.Ticker の公称 8 ms は、受信が遅れると間隔を詰めたり
// ティックを落としたりするので信用しない (計画 §5.2)。
type SPIObserver interface {
	ObserveSPI(tx, rx []byte, before, after time.Time)
}

type spiObserverHolder struct{ o SPIObserver }

var spiObserver atomic.Pointer[spiObserverHolder]

// SetSPIObserver は観測フックを登録する。nil で解除。
func SetSPIObserver(o SPIObserver) {
	if o == nil {
		spiObserver.Store(nil)
		return
	}
	spiObserver.Store(&spiObserverHolder{o: o})
}

// NotifySPI は登録されていれば観測フックを呼ぶ。
//
// 推定・記録が panic しても走行機能を巻き込まない (計画 §7.2)。
func NotifySPI(tx, rx []byte, before, after time.Time) {
	h := spiObserver.Load()
	if h == nil {
		return
	}
	defer func() {
		if r := recover(); r != nil {
			logRecovered("SPI observer", r)
		}
	}()
	h.o.ObserveSPI(tx, rx, before, after)
}

// VelocityOverride は送信フレームの速度指令を差し替える。
//
// 機体パラメータの同定 (計画 §8) で 3 自由度を順に加振するために使う。
// ok が false なら通常の指令をそのまま通す。
type VelocityOverride interface {
	OverrideVelocity() (velX, velY, velAng int16, ok bool)
}

type velocityOverrideHolder struct{ o VelocityOverride }

var velocityOverride atomic.Pointer[velocityOverrideHolder]

// SetVelocityOverride は速度指令の差し替えを登録する。nil で解除。
func SetVelocityOverride(o VelocityOverride) {
	if o == nil {
		velocityOverride.Store(nil)
		return
	}
	velocityOverride.Store(&velocityOverrideHolder{o: o})
}

// applyVelocityOverride は登録されていれば速度指令を差し替える。
//
// 非常停止が立っているフレームには一切触らない。加振は自動で走るので、
// ここを踏み外すと人が止められないロボットになる。
func applyVelocityOverride(sendbytes []byte) {
	h := velocityOverride.Load()
	if h == nil {
		return
	}
	if sendbytes[frame.IdxInfo]&InfoEmgStopMask != 0 {
		return
	}

	var velX, velY, velAng int16
	var ok bool
	func() {
		defer func() {
			if r := recover(); r != nil {
				logRecovered("velocity override", r)
				ok = false
			}
		}()
		velX, velY, velAng, ok = h.o.OverrideVelocity()
	}()
	if !ok {
		return
	}

	put := func(lo, hi int, v int16) {
		sendbytes[lo] = byte(uint16(v) & 0xff)
		sendbytes[hi] = byte(uint16(v) >> 8)
	}
	put(frame.IdxVelXLow, frame.IdxVelXHigh, velX)
	put(frame.IdxVelYLow, frame.IdxVelYHigh, velY)
	put(frame.IdxVelAngLow, frame.IdxVelAngHigh, velAng)
	// 加振中は「指令を受けている」ことにする。受信タイムアウトで
	// 速度が毎周期 0 に戻されると加振にならない。
	sendbytes[frame.IdxInfo] |= InfoSignalReceivedMask
}
