//go:build rock5a

package rock5a

import (
	"fmt"
	"log"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/link"
	"github.com/Rione/ssl-RACOON-Pi3/internal/state"
	"github.com/Rione/ssl-RACOON-Pi3/internal/util"
	"github.com/Rione/ssl-RACOON-Pi3/internal/wheelgraph"
	"periph.io/x/conn/v3/physic"
	"periph.io/x/conn/v3/spi"
	"periph.io/x/conn/v3/spi/spireg"
	"periph.io/x/host/v3"
)

var (
	isSPIFrameValid   bool = true
	prevSPIFrameValid bool = true
	spiRxWindow       [MaxSPIFrameSize * 2]byte

	// 今のフレームの形。STM のファームの世代で 20 / 21 バイトと違うので、
	// 有効なフレームが続けて取れなければもう一方へ切り替えて探す (spiLayoutProbeCycles)。
	curLayout      = spiLayoutV1
	lastFrameAt    = -1 // 前回フレームが見つかった位置 (誤同期を避けるため優先する)
	layoutMisses   int
	layoutSettled  bool
	layoutAnnounce bool
)

// spiLayoutProbeCycles は、この回数だけ続けてフレームが取れなければ形を切り替える。
// 8 ms 周期なので 0.4 s。起動直後の数フレームの取りこぼしでは切り替わらない長さにしてある。
const spiLayoutProbeCycles = 50

func RunSPI(done <-chan struct{}, myID uint32) {
	if _, err := host.Init(); err != nil {
		log.Fatal(err)
	}

	port, err := spireg.Open(SPIDevPath)
	if err != nil {
		log.Fatal(err)
	}
	defer port.Close()

	conn, err := port.Connect(physic.Frequency(SPISpeedHz)*physic.Hertz, spi.Mode0, 8)
	if err != nil {
		log.Fatal(err)
	}

	state.Recvdata = state.RecvData{}
	past := time.Date(2000, 1, 1, 0, 0, 0, 0, time.UTC)
	state.LastRecvTime.Store(past)
	state.LastCmdRecvTime.Store(past)

	ticker := time.NewTicker(SPIPeriodMs * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-done:
			return
		case <-ticker.C:
			processSPICommunication(conn)
		}
	}
}

func processSPICommunication(conn spi.Conn) {
	sendbytes := link.PrepareSendData()
	payload := link.PrepareHardwareTx(sendbytes)
	if len(payload) > curLayout.PayloadSize {
		payload = payload[:curLayout.PayloadSize]
	}
	tx := wrapSPIFrame(curLayout, payload)
	rx := make([]byte, curLayout.FrameSize)

	// 転送時刻は Tx の直前・直後の中点とする。time.Ticker の公称 8 ms は、
	// 受信が遅れると間隔を詰めたりティックを落としたりするので信用しない
	// (docs/self-localization-plan.md §5.2)。
	before := time.Now()
	txErr := conn.Tx(tx, rx)
	after := time.Now()
	if txErr != nil {
		util.CheckError(txErr)
	}

	// 計測基盤へ渡す。登録が無ければ何もしない。
	link.NotifySPI(tx, rx, before, after)

	window := spiRxWindow[:curLayout.FrameSize*2]
	pushSPIRxWindow(curLayout, window, rx)
	frameOffset, frameErr := resolveSPIRxFrame(curLayout, window, lastFrameAt)
	if frameErr == nil {
		lastFrameAt = frameOffset
	} else {
		lastFrameAt = -1
	}
	isSPIFrameValid = frameErr == nil
	handleSPIFrameValidationChange(frameErr)
	updateSPILayout(frameErr == nil)

	if frameErr == nil {
		state.Recvdata = parseRecvBufAt(curLayout, window, frameOffset)
		state.SPIRxValidAt.Store(after.UnixNano())

		state.FlWheelSpeedRadS = motorRawToWheelMS(state.Recvdata.FlWheelSpeed)
		state.BlWheelSpeedRadS = motorRawToWheelMS(state.Recvdata.BlWheelSpeed)
		state.BrWheelSpeedRadS = motorRawToWheelMS(state.Recvdata.BrWheelSpeed)
		state.FrWheelSpeedRadS = motorRawToWheelMS(state.Recvdata.FrWheelSpeed)
		applyImu(state.Recvdata)

		if state.DebugWheelGraph {
			wheelgraph.Record(
				state.Recvdata.FlWheelSpeed,
				state.Recvdata.BlWheelSpeed,
				state.Recvdata.BrWheelSpeed,
				state.Recvdata.FrWheelSpeed,
			)
		}
	}

	if state.DebugSerial {
		if frameErr != nil {
			log.Printf("[SPI RX] FRAME ERROR: %v", frameErr)
			log.Printf("[SPI RX] full (%dB): % x", curLayout.FrameSize, rx)
		} else {
			log.Printf("[SPI RX] Raw: % 02X", rx[1:1+SPIRecvSize])
			log.Printf("[SPI RX] Volt: %d (%.1fV), SensorInfo: 0b%08b, CapPower: %d",
				state.Recvdata.Volt, float32(state.Recvdata.Volt)*0.1, state.Recvdata.SensorInformation, state.Recvdata.CapPower)
			log.Printf("[SPI RX] Wheel(raw) FL: %d, BL: %d, BR: %d, FR: %d",
				state.Recvdata.FlWheelSpeed, state.Recvdata.BlWheelSpeed, state.Recvdata.BrWheelSpeed, state.Recvdata.FrWheelSpeed)
			log.Printf("[SPI RX] Wheel(m/s) FL: %.3f, BL: %.3f, BR: %.3f, FR: %.3f",
				state.FlWheelSpeedRadS, state.BlWheelSpeedRadS, state.BrWheelSpeedRadS, state.FrWheelSpeedRadS)
			log.Printf("[SPI RX] full (%dB): % x", curLayout.FrameSize, rx)
		}
		log.Printf("[SPI TX] full (%dB): % x", curLayout.FrameSize, tx)
		link.LogSendData(sendbytes)
		if state.DryRun {
			link.LogSendData(payload)
		}
	}

	link.CheckBatteryStatus()
	link.FinishLinkCycle()
	prevSPIFrameValid = isSPIFrameValid
}

// updateSPILayout は、フレームが取れない状態が続いたらもう一方の形へ切り替える。
// 新旧のファームが混ざっていても、機体ごとに勝手に合う形に落ち着く。
func updateSPILayout(ok bool) {
	if ok {
		layoutMisses = 0
		if !layoutSettled {
			layoutSettled = true
			log.Printf("SPI frame layout: %s", curLayout.Name)
		}
		return
	}
	layoutMisses++
	if layoutMisses < spiLayoutProbeCycles {
		return
	}
	layoutMisses = 0
	layoutSettled = false
	if curLayout.FrameSize == spiLayoutV1.FrameSize {
		curLayout = spiLayoutV2
	} else {
		curLayout = spiLayoutV1
	}
	if !layoutAnnounce {
		layoutAnnounce = true
		log.Printf("SPI: no valid frame for %d cycles; trying %s", spiLayoutProbeCycles, curLayout.Name)
	}
	for i := range spiRxWindow {
		spiRxWindow[i] = 0
	}
}

func resolveSPIRxFrame(l spiLayout, window []byte, prefer int) (offset int, err error) {
	offset = findSPIFrame(l, window, prefer)
	if offset < 0 {
		return 0, fmt.Errorf("no valid frame in %d-byte window", len(window))
	}
	return offset, nil
}

func parseRecvBufAt(l spiLayout, rx []byte, frameOffset int) state.RecvData {
	off := frameOffset + 1
	d := state.RecvData{
		Volt:              rx[off+0],
		SensorInformation: rx[off+1],
		CapPower:          rx[off+2],
		FlWheelSpeed:      int16(rx[off+3]) | int16(rx[off+4])<<8,
		BlWheelSpeed:      int16(rx[off+5]) | int16(rx[off+6])<<8,
		BrWheelSpeed:      int16(rx[off+7]) | int16(rx[off+8])<<8,
		FrWheelSpeed:      int16(rx[off+9]) | int16(rx[off+10])<<8,
	}
	if l.HasIMU {
		// 12〜19 バイト目: 加速度 X・Y [mg]、ヨーの角速度、Madgwick の姿勢角
		// (ssl-Circuit MainBoard_V26_2 src/unit/robot.c の Robot_RockBuildTxPacket)。
		d.HasIMU = true
		d.AccelXRaw = int16(rx[off+11]) | int16(rx[off+12])<<8
		d.AccelYRaw = int16(rx[off+13]) | int16(rx[off+14])<<8
		d.YawRateRaw = int16(rx[off+15]) | int16(rx[off+16])<<8
		d.YawAngleRaw = int16(rx[off+17]) | int16(rx[off+18])<<8
	}
	return d
}

// applyImu は受け取った IMU の生値を SI に直して state に置く。
func applyImu(d state.RecvData) {
	state.ImuValid = d.HasIMU
	if !d.HasIMU {
		return
	}
	state.ImuAccelXMS2 = float64(d.AccelXRaw) * spiAccelPerLSBG * gravityMS2
	state.ImuAccelYMS2 = float64(d.AccelYRaw) * spiAccelPerLSBG * gravityMS2
	state.ImuYawRateRadS = float64(d.YawRateRaw) * spiYawRatePerLSBRad
	state.ImuYawRad = float64(d.YawAngleRaw) * spiYawPerLSBRad
}

func motorRawToWheelMS(raw int16) float32 {
	wheelRadS := float32(raw) / 100.0
	wheelRadiusM := float32(WheelDiameterMm / 2000.0)
	return wheelRadS * wheelRadiusM
}

func handleSPIFrameValidationChange(frameErr error) {
	if frameErr != nil && prevSPIFrameValid {
		log.Printf("SPI recv frame mismatch: %v", frameErr)
		link.RingBuzzerAsync(5, 300*time.Millisecond, 0)
	}
	if frameErr == nil && !prevSPIFrameValid {
		log.Println("SPI recv frame recovered")
		link.RingBuzzerAsync(10, 200*time.Millisecond, 0)
	}
}
