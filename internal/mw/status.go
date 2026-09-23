package mw

import (
	"log"
	"net"

	"github.com/Rione/ssl-RACOON-Pi3/internal/state"
	"github.com/Rione/ssl-RACOON-Pi3/proto/pb_gen"
	"google.golang.org/protobuf/proto"
)

// 既存の車輪・電圧・ボール状態の返信。新しい推定結果の境界はestimate.go。
func createStatus(robotID uint32, detectPhotoSensor, detectDribbler, isNewDribbler bool,
	batteryVoltage, capPower uint32, isBallExit bool, imageX, imageY float32,
	minThreshold, maxThreshold string, ballDetectRadius int32, circularityThreshold float32,
	flWheelSpeed, blWheelSpeed, brWheelSpeed, frWheelSpeed float32) *pb_gen.PiToMw {
	isNewRobot := state.IsNewRobot
	piToMw := &pb_gen.PiToMw{
		IsNewRobot: &isNewRobot,
		RobotsStatus: &pb_gen.Robot_Status{
			RobotId:                &robotID,
			IsDetectPhotoSensor:    &detectPhotoSensor,
			IsDetectDribblerSensor: &detectDribbler,
			IsNewDribbler:          &isNewDribbler,
			BatteryVoltage:         &batteryVoltage,
			CapPower:               &capPower,
			FlWheelSpeed:           &flWheelSpeed,
			BlWheelSpeed:           &blWheelSpeed,
			BrWheelSpeed:           &brWheelSpeed,
			FrWheelSpeed:           &frWheelSpeed,
		},
		BallStatus: &pb_gen.Ball_Status{
			IsBallExit:  &isBallExit,
			BallCameraX: &imageX,
			BallCameraY: &imageY,
		},
		Ball: &pb_gen.Ball{
			MinThreshold:         &minThreshold,
			MaxThreshold:         &maxThreshold,
			BallDetectRadius:     &ballDetectRadius,
			CircularityThreshold: &circularityThreshold,
		},
	}
	// MACアドレス(NIC由来)が取得できていれば付与する。
	if state.MACAddress != "" {
		mac := state.MACAddress
		piToMw.MacAddress = &mac
	}
	// バージョンが取得できていれば付与する（RAVEN の Robot Status ペイン表示用）。
	if state.Version != "" {
		version := state.Version
		piToMw.Version = &version
	}
	return piToMw
}

func sendStatusToMW(conn *net.UDPConn, targetAddr *net.UDPAddr, myID uint32, adjustment state.Adjustment) {
	detectPhotoSensor := state.Recvdata.SensorInformation&state.SensorPhotoMask != 0
	detectDribblerSensor := state.Recvdata.SensorInformation&state.SensorDribblerMask != 0
	isNewDribbler := state.Recvdata.SensorInformation&state.SensorNewDribMask != 0

	var isBallExit bool
	var imageX, imageY float32 = state.BallCoordMissing, state.BallCoordMissing
	if state.ImageDataPtr != nil {
		isBallExit = state.ImageDataPtr.IsBallExit
		imageX = state.ImageDataPtr.ImageX
		imageY = state.ImageDataPtr.ImageY
		if !isBallExit {
			imageX = state.BallCoordMissing
			imageY = state.BallCoordMissing
		}
	}

	status := createStatus(
		myID,
		detectPhotoSensor,
		detectDribblerSensor,
		isNewDribbler,
		uint32(state.Recvdata.Volt),
		uint32(state.Recvdata.CapPower),
		isBallExit,
		imageX,
		imageY,
		adjustment.MinThreshold,
		adjustment.MaxThreshold,
		int32(adjustment.BallDetectRadius),
		adjustment.CircularityThreshold,
		state.FlWheelSpeedRadS,
		state.BlWheelSpeedRadS,
		state.BrWheelSpeedRadS,
		state.FrWheelSpeedRadS,
	)

	data, err := proto.Marshal(status)
	if err != nil {
		log.Printf("Protobuf marshal error: %v", err)
		return
	}

	header := byte((myID << 4) | 0x05)
	sendData := append([]byte{header}, data...)

	if _, err := conn.WriteToUDP(sendData, targetAddr); err != nil {
		log.Printf("UDP send error: %v", err)
	}
}
