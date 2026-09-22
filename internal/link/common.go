//go:build pi4 || rock5a

package link

import (
	"fmt"
	"log"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/state"
)

var (
	isSignalReceived     bool
	prevIsSignalReceived bool
	prevEmgStop          bool = true
	prevPowerShutdown    bool
)

// Default capture resolution halves (640x480 legacy fallback for SPI byte mapping).
const (
	cameraFrameHalfWidth  = 320
	cameraFrameHalfHeight = 240
)

func cameraFrameHalfSizes() (halfW, halfH int) {
	halfW = cameraFrameHalfWidth
	halfH = cameraFrameHalfHeight
	if state.ImageDataPtr == nil {
		return halfW, halfH
	}
	if state.ImageDataPtr.FrameWidth > 0 && state.ImageDataPtr.FrameHeight > 0 {
		return state.ImageDataPtr.FrameWidth / 2, state.ImageDataPtr.FrameHeight / 2
	}
	return halfW, halfH
}

func scaleCameraCoord(coord float32, halfSize int) int {
	scaled := 128 + int(coord*127/float32(halfSize))
	if scaled < 0 {
		return 0
	}
	if scaled > 255 {
		return 255
	}
	return scaled
}

func PrepareSendData() []byte {
	sendbytes := frame.EnsureSendFrame()

	updateCameraCoordinates(sendbytes)
	handleReceiveTimeout(sendbytes)

	if state.IsControlByRobotMode {
		sendbytes[frame.IdxInfo] |= state.InfoCtrlByRobot
	}

	if state.LastRecvTime.Since() > state.ChargeStopTimeout {
		sendbytes[frame.IdxInfo] &= ^uint8(state.InfoDoCharge)
	}

	handleReceiveStateChange()

	if state.KickerEnable {
		sendbytes[frame.IdxKick] = state.KickerVal
	} else {
		sendbytes[frame.IdxKick] = 0
	}

	if state.VelX1000 {
		sendbytes[frame.IdxVelXLow] = byte(uint16(1000) & 0xff)
		sendbytes[frame.IdxVelXHigh] = byte(uint16(1000) >> 8)
	}

	handleEmgStopChange(sendbytes)

	return sendbytes
}

// PrepareHardwareTx returns the frame actually sent on serial/SPI.
// In dry-run mode, motion fields (VelX/Y/Ang, dribble, kick, chip) are zeroed.
func PrepareHardwareTx(sendbytes []byte) []byte {
	out := append([]byte(nil), sendbytes...)
	if frame.IdxPowerCmd >= 0 {
		if len(out) <= frame.IdxPowerCmd {
			extended := make([]byte, frame.IdxPowerCmd+1)
			copy(extended, out)
			out = extended
		}
		if state.PowerShutdownMode {
			out[frame.IdxPowerCmd] = state.PowerCmdShutdown
		} else {
			out[frame.IdxPowerCmd] = 0x00
		}
	}
	handlePowerShutdownChange()
	if !state.DryRun {
		return out
	}
	for i := frame.IdxVelXLow; i <= frame.IdxChip; i++ {
		out[i] = 0
	}
	return out
}

func CheckBatteryStatus() {
	if state.Recvdata.Volt < uint8(state.BatteryCriticalThreshold) {
		state.IsRobotError = true
		state.RobotErrorCode = 2
		state.RobotErrorMessage = "バッテリ電圧異常(回路故障の可能性)"
	} else if state.Recvdata.Volt < uint8(state.BatteryLowThreshold) {
		state.IsRobotError = true
		state.RobotErrorCode = 2
		state.RobotErrorMessage = "バッテリ電圧異常"
	}
}

func FinishLinkCycle() {
	prevIsSignalReceived = isSignalReceived
}

func updateCameraCoordinates(sendbytes []byte) {
	if state.ImageDataPtr == nil || !state.ImageDataPtr.IsBallExit {
		sendbytes[frame.IdxCamBallX] = 0
		sendbytes[frame.IdxCamBallY] = 0
		return
	}

	halfW, halfH := cameraFrameHalfSizes()
	scaledX := scaleCameraCoord(state.ImageDataPtr.ImageX, halfW)
	sendbytes[frame.IdxCamBallX] = byte(scaledX)

	scaledY := scaleCameraCoord(state.ImageDataPtr.ImageY, halfH)
	sendbytes[frame.IdxCamBallY] = byte(scaledY)
}

func handleReceiveTimeout(sendbytes []byte) {
	if state.LastCmdRecvTime.Since() > state.NoRecvTimeout && !state.IsControlByRobotMode {
		for i := frame.IdxVelXLow; i <= frame.IdxChip; i++ {
			sendbytes[i] = 0
		}
		sendbytes[frame.IdxInfo] &= ^uint8(state.InfoSignalReceived)
		isSignalReceived = false
	} else {
		sendbytes[frame.IdxInfo] |= state.InfoSignalReceived
		isSignalReceived = true
	}
}

func handleReceiveStateChange() {
	if !isSignalReceived && prevIsSignalReceived {
		log.Println("No Data Recv")
		RingBuzzerAsync(3, 500*time.Millisecond, 0)
	}

	if isSignalReceived && !prevIsSignalReceived {
		RingBuzzerAsync(10, 500*time.Millisecond, 0)
	}
}

func handleEmgStopChange(sendbytes []byte) {
	emgActive := sendbytes[frame.IdxInfo]&state.InfoEmgStop != 0
	if prevEmgStop && !emgActive {
		log.Println("Emergency stop released (InfoEmgStop: 1 -> 0)")
	}
	if !prevEmgStop && emgActive {
		log.Println("Emergency stop activated (InfoEmgStop: 0 -> 1)")
	}
	prevEmgStop = emgActive
}

func handlePowerShutdownChange() {
	if !prevPowerShutdown && state.PowerShutdownMode {
		log.Printf("Power shutdown command activated (byte[%d]: 0x%02x)", frame.IdxPowerCmd, state.PowerCmdShutdown)
	}
	prevPowerShutdown = state.PowerShutdownMode
}

func LogSendData(sendbytes []byte) {
	velx := int16(sendbytes[frame.IdxVelXLow]) | int16(sendbytes[frame.IdxVelXHigh])<<8
	vely := int16(sendbytes[frame.IdxVelYLow]) | int16(sendbytes[frame.IdxVelYHigh])<<8
	velang := int16(sendbytes[frame.IdxVelAngLow]) | int16(sendbytes[frame.IdxVelAngHigh])<<8

	log.Printf("[Link TX] VelX: %d, VelY: %d, VelAng: %d, Dribble: %d, Kick: %d, Chip: %d, Info: 0b%08b",
		velx, vely, velang, sendbytes[frame.IdxDribble], sendbytes[frame.IdxKick], sendbytes[frame.IdxChip], sendbytes[frame.IdxInfo])
	log.Printf("[Link TX] CamBallX: %d, CamBallY: %d, Raw: %v",
		sendbytes[frame.IdxCamBallX], sendbytes[frame.IdxCamBallY], sendbytes)
	fmt.Println("---")
}
