//go:build pi4 || rock5a

package link

import (
	"log"

	"github.com/Rione/ssl-RACOON-Pi2/internal/state"
)

// state 側のビット定義を link 内で参照しやすくしたもの。
const (
	InfoEmgStopMask        = state.InfoEmgStop
	InfoSignalReceivedMask = state.InfoSignalReceived
)

func logRecovered(what string, r any) {
	log.Printf("[LINK] %s panicked and was contained: %v", what, r)
}
