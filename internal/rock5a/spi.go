//go:build rock5a

package rock5a

import (
	"fmt"
	"log"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/link"
	"github.com/Rione/ssl-RACOON-Pi2/internal/state"
	"github.com/Rione/ssl-RACOON-Pi2/internal/util"
	"github.com/Rione/ssl-RACOON-Pi2/internal/wheelgraph"
	"periph.io/x/conn/v3/physic"
	"periph.io/x/conn/v3/spi"
	"periph.io/x/conn/v3/spi/spireg"
	"periph.io/x/host/v3"
)

var (
	isSPIFrameValid   bool = true
	prevSPIFrameValid bool = true
	spiRxWindow       [SPIFrameSize * 2]byte
)

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
	if len(payload) > SPIPayloadSize {
		payload = payload[:SPIPayloadSize]
	}
	tx := wrapSPIFrame(payload)
	rx := make([]byte, SPIFrameSize)

	if err := conn.Tx(tx, rx); err != nil {
		util.CheckError(err)
	}

	pushSPIRxWindow(spiRxWindow[:], rx)
	frameOffset, frameErr := resolveSPIRxFrame(spiRxWindow[:])
	isSPIFrameValid = frameErr == nil
	handleSPIFrameValidationChange(frameErr)

	if frameErr == nil {
		state.Recvdata = parseRecvBufAt(spiRxWindow[:], frameOffset)

		state.FlWheelSpeedRadS = motorRawToWheelMS(state.Recvdata.FlWheelSpeed)
		state.BlWheelSpeedRadS = motorRawToWheelMS(state.Recvdata.BlWheelSpeed)
		state.BrWheelSpeedRadS = motorRawToWheelMS(state.Recvdata.BrWheelSpeed)
		state.FrWheelSpeedRadS = motorRawToWheelMS(state.Recvdata.FrWheelSpeed)

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
			log.Printf("[SPI RX] full (%dB): % x", SPIFrameSize, rx)
		} else {
			log.Printf("[SPI RX] Raw: % 02X", rx[1:1+SPIRecvSize])
			log.Printf("[SPI RX] Volt: %d (%.1fV), SensorInfo: 0b%08b, CapPower: %d",
				state.Recvdata.Volt, float32(state.Recvdata.Volt)*0.1, state.Recvdata.SensorInformation, state.Recvdata.CapPower)
			log.Printf("[SPI RX] Wheel(raw) FL: %d, BL: %d, BR: %d, FR: %d",
				state.Recvdata.FlWheelSpeed, state.Recvdata.BlWheelSpeed, state.Recvdata.BrWheelSpeed, state.Recvdata.FrWheelSpeed)
			log.Printf("[SPI RX] Wheel(m/s) FL: %.3f, BL: %.3f, BR: %.3f, FR: %.3f",
				state.FlWheelSpeedRadS, state.BlWheelSpeedRadS, state.BrWheelSpeedRadS, state.FrWheelSpeedRadS)
			log.Printf("[SPI RX] full (%dB): % x", SPIFrameSize, rx)
		}
		log.Printf("[SPI TX] full (%dB): % x", SPIFrameSize, tx)
		link.LogSendData(sendbytes)
		if state.DryRun {
			link.LogSendData(payload)
		}
	}

	link.CheckBatteryStatus()
	link.FinishLinkCycle()
	prevSPIFrameValid = isSPIFrameValid
}

func resolveSPIRxFrame(window []byte) (offset int, err error) {
	offset = findSPIFrame(window)
	if offset < 0 {
		return 0, fmt.Errorf("no valid frame in %d-byte window", len(window))
	}
	return offset, nil
}

func parseRecvBufAt(rx []byte, frameOffset int) state.RecvData {
	off := frameOffset + 1
	return state.RecvData{
		Volt:              rx[off+0],
		SensorInformation: rx[off+1],
		CapPower:          rx[off+2],
		FlWheelSpeed:      int16(rx[off+3]) | int16(rx[off+4])<<8,
		BlWheelSpeed:      int16(rx[off+5]) | int16(rx[off+6])<<8,
		BrWheelSpeed:      int16(rx[off+7]) | int16(rx[off+8])<<8,
		FrWheelSpeed:      int16(rx[off+9]) | int16(rx[off+10])<<8,
	}
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
