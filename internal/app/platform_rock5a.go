//go:build rock5a

package app

import (
	"github.com/Rione/ssl-RACOON-Pi2/internal/receive"
	"github.com/Rione/ssl-RACOON-Pi2/internal/rock5a"
	"github.com/Rione/ssl-RACOON-Pi2/internal/state"
)

func registerPlatform() {
	state.IsNewRobot = true
	rock5a.RegisterLink()
	receive.SetPlayBallDetectedSound(rock5a.PlayBallDetectedSound)
}

func checkInitialButtonState() bool { return rock5a.CheckInitialButtonState() }
func defaultHostname() string       { return rock5a.DefaultHostname }
func setupNewHostname()             { rock5a.SetupNewHostname() }
func handleLocalUserMode() bool     { return rock5a.HandleLocalUserMode() }
func initBoard()                    { rock5a.InitBoard() }
func cleanupBoard()                 { rock5a.CleanupBoard() }
func readRobotIDFromDIP() int       { return rock5a.ReadRobotIDFromDIP() }
func runLink(done <-chan struct{}, myID uint32) {
	rock5a.RunLink(done, myID)
}
func runGPIO(done <-chan struct{}) { rock5a.RunGPIO(done) }
