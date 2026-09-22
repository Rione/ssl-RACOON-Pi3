//go:build rock5a

package rock5a

import (
	"github.com/Rione/ssl-RACOON-Pi2/internal/link"
)

func RegisterLink() {
	link.ConfigureFrame(link.FrameConfig{
		IdxVelXLow:    0,
		IdxVelXHigh:   1,
		IdxVelYLow:    2,
		IdxVelYHigh:   3,
		IdxVelAngLow:  4,
		IdxVelAngHigh: 5,
		IdxDribble:    6,
		IdxKick:       7,
		IdxChip:       8,
		IdxCamBallX:   15,
		IdxCamBallY:   16,
		IdxInfo:       17,
		IdxPowerCmd:   -1,
		EnsureSendFrame: ensureSendFrame,
	})
	link.SetRingBuzzer(RingBuzzer)
}

func RunLink(done <-chan struct{}, myID uint32) {
	RunSPI(done, myID)
}
