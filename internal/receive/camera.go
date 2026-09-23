package receive

import (
	"encoding/json"
	"log"
	"net"

	"github.com/Rione/ssl-RACOON-Pi3/internal/state"
	"github.com/Rione/ssl-RACOON-Pi3/internal/util"
)

// 機上ボールカメラの受信。自己位置用SSL-Visionはlocadapter/vision.goが担当する。
func ReceiveData(done <-chan struct{}, myID uint32, ip string) {
	serverAddr := &net.UDPAddr{
		IP:   net.ParseIP(ip),
		Port: state.UDPCameraPort,
	}

	serverConn, err := net.ListenUDP("udp", serverAddr)
	util.CheckError(err)
	defer serverConn.Close()

	buf := make([]byte, 20240)

	for {
		select {
		case <-done:
			return
		default:
			n, _, _ := serverConn.ReadFromUDP(buf)

			jsonData := &state.ImageData{}
			if err := json.Unmarshal(buf[0:n], jsonData); err != nil {
				log.Printf("JSON unmarshal error: %v", err)
				continue
			}

			state.ApplyMissingBallCoords(jsonData)

			state.ImageDataPtr = jsonData
			state.ImageResponseData.Frame = jsonData.Frame

			if jsonData.IsBallExit && !state.PrevBallDetected {
				if state.DebugCamera && playBallDetectedSound != nil {
					go playBallDetectedSound()
				}
			}
			state.PrevBallDetected = jsonData.IsBallExit
		}
	}
}
