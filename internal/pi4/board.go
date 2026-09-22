//go:build pi4

package pi4

import (
	"github.com/Rione/ssl-RACOON-Pi2/internal/link"
)

func RegisterLink() {
	link.ConfigureFrame(link.FrameConfig{
		IdxVelXLow:    1,
		IdxVelXHigh:   2,
		IdxVelYLow:    3,
		IdxVelYHigh:   4,
		IdxVelAngLow:  5,
		IdxVelAngHigh: 6,
		IdxDribble:    7,
		IdxKick:       8,
		IdxChip:       9,
		IdxCamBallX:   16,
		IdxCamBallY:   17,
		IdxInfo:       18,
		IdxPowerCmd:   -1,
		EnsureSendFrame: ensureSendFrame,
	})
	link.SetRingBuzzer(RingBuzzer)
}

func RunLink(done <-chan struct{}, myID uint32) {
	RunSerial(done, myID)
}
