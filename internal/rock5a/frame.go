//go:build rock5a

package rock5a

import (
	"fmt"

	"github.com/Rione/ssl-RACOON-Pi3/internal/state"
)

// ensureSendFrame は PC から来た指令 (18 バイト) を取り出す。無ければ非常停止の立ったフレーム。
func ensureSendFrame() []byte {
	const cmdSize = 18 // 下りの中身はファームの世代で変わらない (robot.c の Robot_RockApplyRecvPacket)
	b := state.GetSendPayload()
	if len(b) < cmdSize {
		frame := make([]byte, cmdSize)
		frame[17] = state.InfoEmgStop
		return frame
	}
	if len(b) > cmdSize {
		return b[:cmdSize]
	}
	return b
}

// wrapSPIFrame はヘッダとフッタを付けて、そのレイアウトの長さのフレームにする。
// v2 では中身が 1 バイト長い (余りは 0)。
func wrapSPIFrame(l spiLayout, payload []byte) []byte {
	frame := make([]byte, l.FrameSize)
	frame[0] = SPIFrameHeader
	n := len(payload)
	if n > l.PayloadSize {
		n = l.PayloadSize
	}
	copy(frame[1:], payload[:n])
	frame[l.FrameSize-1] = SPIFrameFooter
	return frame
}

func validateSPIFrameAt(l spiLayout, rx []byte, offset int) error {
	if offset < 0 || offset+l.FrameSize > len(rx) {
		return fmt.Errorf("frame out of range at offset %d", offset)
	}
	if rx[offset] != SPIFrameHeader {
		return fmt.Errorf("header: expected %02x, got %02x", SPIFrameHeader, rx[offset])
	}
	if rx[offset+l.FrameSize-1] != SPIFrameFooter {
		return fmt.Errorf("footer: expected %02x, got %02x", SPIFrameFooter, rx[offset+l.FrameSize-1])
	}
	if l.HasIMU {
		// 12 バイト目から先は IMU で、0 とは限らない。ヘッダとフッタだけで判定する。
		return nil
	}
	for i := offset + 1 + SPIRecvSize; i < offset+l.FrameSize-1; i++ {
		if rx[i] != 0 {
			return fmt.Errorf("padding[%d]: expected 00, got %02x", i-offset, rx[i])
		}
	}
	return nil
}

func validateSPIFrame(l spiLayout, rx []byte) error {
	if len(rx) < l.FrameSize {
		return fmt.Errorf("short frame: got %d bytes, want %d", len(rx), l.FrameSize)
	}
	return validateSPIFrameAt(l, rx, 0)
}

// findSPIFrame はバッファ内の有効フレーム位置を返す (見つからなければ -1)。
// prefer が有効ならそれを使う: IMU 入りのフレームは 0 埋めが無く、ヘッダとフッタだけが手がかりなので、
// たまたま条件を満たす別の位置に飛び移らないよう、前回と同じ位置を優先する。
func findSPIFrame(l spiLayout, buf []byte, prefer int) int {
	if prefer >= 0 && validateSPIFrameAt(l, buf, prefer) == nil {
		return prefer
	}
	last := -1
	for i := 0; i+l.FrameSize <= len(buf); i++ {
		if validateSPIFrameAt(l, buf, i) == nil {
			last = i
		}
	}
	return last
}

func pushSPIRxWindow(l spiLayout, window, chunk []byte) {
	copy(window, window[l.FrameSize:])
	copy(window[l.FrameSize:], chunk[:l.FrameSize])
}
