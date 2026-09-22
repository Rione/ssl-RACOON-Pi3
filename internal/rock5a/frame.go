//go:build rock5a

package rock5a

import (
	"fmt"

	"github.com/Rione/ssl-RACOON-Pi2/internal/state"
)

func ensureSendFrame() []byte {
	b := state.GetSendPayload()
	if len(b) < SPIPayloadSize {
		frame := make([]byte, SPIPayloadSize)
		frame[17] = state.InfoEmgStop
		return frame
	}
	if len(b) > SPIPayloadSize {
		return b[:SPIPayloadSize]
	}
	return b
}

func wrapSPIFrame(payload []byte) []byte {
	frame := make([]byte, SPIFrameSize)
	frame[0] = SPIFrameHeader
	n := len(payload)
	if n > SPIPayloadSize {
		n = SPIPayloadSize
	}
	copy(frame[1:], payload[:n])
	frame[SPIFrameSize-1] = SPIFrameFooter
	return frame
}

func validateSPIFrameAt(rx []byte, offset int) error {
	if offset < 0 || offset+SPIFrameSize > len(rx) {
		return fmt.Errorf("frame out of range at offset %d", offset)
	}
	if rx[offset] != SPIFrameHeader {
		return fmt.Errorf("header: expected %02x, got %02x", SPIFrameHeader, rx[offset])
	}
	if rx[offset+SPIFrameSize-1] != SPIFrameFooter {
		return fmt.Errorf("footer: expected %02x, got %02x", SPIFrameFooter, rx[offset+SPIFrameSize-1])
	}
	for i := offset + 1 + SPIRecvSize; i < offset+SPIFrameSize-1; i++ {
		if rx[i] != 0 {
			return fmt.Errorf("padding[%d]: expected 00, got %02x", i-offset, rx[i])
		}
	}
	return nil
}

func validateSPIFrame(rx []byte) error {
	if len(rx) < SPIFrameSize {
		return fmt.Errorf("short frame: got %d bytes, want %d", len(rx), SPIFrameSize)
	}
	return validateSPIFrameAt(rx, 0)
}

// findSPIFrame はバッファ内の最後の有効フレーム位置を返す (見つからなければ -1)
func findSPIFrame(buf []byte) int {
	last := -1
	for i := 0; i+SPIFrameSize <= len(buf); i++ {
		if validateSPIFrameAt(buf, i) == nil {
			last = i
		}
	}
	return last
}

func pushSPIRxWindow(window, chunk []byte) {
	copy(window, window[SPIFrameSize:])
	copy(window[SPIFrameSize:], chunk[:SPIFrameSize])
}
