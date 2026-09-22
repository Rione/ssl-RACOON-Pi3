//go:build pi4

package pi4

import "github.com/Rione/ssl-RACOON-Pi2/internal/state"

func ensureSendFrame() []byte {
	payload := state.GetSendPayload()
	frame := make([]byte, 19)
	frame[0] = 0xFF
	if len(payload) > 0 {
		copy(frame[1:], payload)
	} else {
		frame[18] = state.InfoEmgStop
	}
	return frame
}
