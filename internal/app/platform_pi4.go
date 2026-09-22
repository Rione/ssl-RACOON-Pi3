//go:build pi4

package app

import (
	"github.com/Rione/ssl-RACOON-Pi2/internal/pi4"
	"github.com/Rione/ssl-RACOON-Pi2/internal/receive"
	"github.com/Rione/ssl-RACOON-Pi2/internal/state"
)

func registerPlatform() {
	state.IsNewRobot = false
	pi4.RegisterLink()
	receive.SetPlayBallDetectedSound(pi4.PlayBallDetectedSound)
}

func checkInitialButtonState() bool { return pi4.CheckInitialButtonState() }
func defaultHostname() string       { return pi4.DefaultHostname }
func setupNewHostname()             { pi4.SetupNewHostname() }
func handleLocalUserMode() bool     { return pi4.HandleLocalUserMode() }
func initBoard()                    { pi4.InitBoard() }
func cleanupBoard()                 { pi4.CleanupBoard() }
func readRobotIDFromDIP() int       { return pi4.ReadRobotIDFromDIP() }
func runLink(done <-chan struct{}, myID uint32) {
	pi4.RunLink(done, myID)
}
func runGPIO(done <-chan struct{}) { pi4.RunGPIO(done) }
