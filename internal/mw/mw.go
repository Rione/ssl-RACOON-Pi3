package mw

import (
	"encoding/json"
	"fmt"
	"log"
	"net"
	"os"
	"sync"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/state"
	"github.com/Rione/ssl-RACOON-Pi2/internal/util"
	"github.com/Rione/ssl-RACOON-Pi2/proto/pb_gen"
	"google.golang.org/protobuf/proto"
)

// The adjustment (HSV thresholds etc.) is cached so that calibration and manual
// changes take effect without restarting the MW loop. ReloadAdjustment re-reads
// threshold.json, which is the source of truth shared with the camera process.
var (
	adjustmentMu      sync.RWMutex
	currentAdjustment state.Adjustment
	adjustmentLoaded  bool
)

// GetAdjustment returns the cached adjustment, loading it from disk on first use.
func GetAdjustment() state.Adjustment {
	adjustmentMu.RLock()
	if adjustmentLoaded {
		adj := currentAdjustment
		adjustmentMu.RUnlock()
		return adj
	}
	adjustmentMu.RUnlock()
	return ReloadAdjustment()
}

// ReloadAdjustment re-reads threshold.json into the cache and returns it.
func ReloadAdjustment() state.Adjustment {
	adj := loadOrCreateThresholdConfig()
	adjustmentMu.Lock()
	currentAdjustment = adj
	adjustmentLoaded = true
	adjustmentMu.Unlock()
	return adj
}

const (
	thresholdFile    = "threshold.json"
	sendInterval     = time.Second / 60
	discoverInterval = 1500 * time.Millisecond // DISCOVER再送間隔。PCは最終通信から1.5s以内のDISCOVERを重複として破棄する
	okInterval       = 100 * time.Millisecond
	robotTimeout     = 3 * time.Second // PCから3s音沙汰がなければタイムアウト
)

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

func RunServer(done <-chan struct{}, myID uint32) {
	mcastAddr, err := net.ResolveUDPAddr("udp", state.MulticastAddr+":"+state.MulticastPort)
	util.CheckError(err)

	conn, err := net.ListenUDP("udp", nil)
	util.CheckError(err)
	defer conn.Close()

	ReloadAdjustment()

	ticker := time.NewTicker(sendInterval)
	defer ticker.Stop()

	var lastDiscoverTime time.Time
	var lastOkTime time.Time

	for {
		select {
		case <-done:
			return
		case <-ticker.C:
			state.StateMu.Lock()

			if state.ConnectionState != state.StateDiscovering && state.LastRecvTime.Since() > robotTimeout {
				log.Println("[AI TX] PC connection timed out. Reverting to DISCOVERING.")
				state.ConnectionState = state.StateDiscovering
				state.PcAddress = nil
			}

			currentState := state.ConnectionState
			currentPcAddr := state.PcAddress

			state.StateMu.Unlock()

			switch currentState {
			case state.StateDiscovering:
				if time.Since(lastDiscoverTime) > discoverInterval {
					header := []byte{byte((myID << 4) | 0x01)}
					if _, err := conn.WriteToUDP(header, mcastAddr); err != nil {
						log.Printf("Failed to send DISCOVER: %v", err)
					}
					lastDiscoverTime = time.Now()
				}

			case state.StateOffered:
				if time.Since(lastOkTime) > okInterval && currentPcAddr != nil {
					header := []byte{byte((myID << 4) | 0x03)}
					if _, err := conn.WriteToUDP(header, currentPcAddr); err != nil {
						log.Printf("Failed to send OK_ROBOT: %v", err)
					}
					lastOkTime = time.Now()
				}

			case state.StateConnected:
				if currentPcAddr != nil {
					sendStatusToMW(conn, currentPcAddr, myID, GetAdjustment())
				}
			}
		}
	}
}

func loadOrCreateThresholdConfig() state.Adjustment {
	if _, err := os.Stat(thresholdFile); os.IsNotExist(err) {
		if err := saveAdjustmentConfig(state.DefaultAdjustment); err != nil {
			log.Printf("しきい値ファイル作成エラー: %v", err)
		}
		return state.DefaultAdjustment
	}

	file, err := os.Open(thresholdFile)
	if err != nil {
		log.Printf("しきい値ファイル読み込みエラー: %v", err)
		return state.DefaultAdjustment
	}
	defer file.Close()

	var adjustment state.Adjustment
	if err := json.NewDecoder(file).Decode(&adjustment); err != nil {
		log.Printf("しきい値JSONデコードエラー: %v", err)
		return state.DefaultAdjustment
	}

	return adjustment
}

func saveAdjustmentConfig(adjustment state.Adjustment) error {
	// Preserve any extra keys (e.g. camera* settings shared with the Python
	// camera process, such as cameraFlip180 / cameraExposure / cameraGain) by
	// merging into the existing file instead of overwriting it with only the
	// Adjustment fields.
	merged := map[string]interface{}{}
	if data, err := os.ReadFile(thresholdFile); err == nil {
		if err := json.Unmarshal(data, &merged); err != nil {
			merged = map[string]interface{}{}
		}
	}

	merged["minThreshold"] = adjustment.MinThreshold
	merged["maxThreshold"] = adjustment.MaxThreshold
	merged["ballDetectRadius"] = adjustment.BallDetectRadius
	merged["circularityThreshold"] = adjustment.CircularityThreshold

	jsonData, err := json.Marshal(merged)
	if err != nil {
		return fmt.Errorf("JSON変換エラー: %w", err)
	}
	return os.WriteFile(thresholdFile, jsonData, 0644)
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

func SaveAdjustmentConfig(adjustment state.Adjustment) error {
	return saveAdjustmentConfig(adjustment)
}
