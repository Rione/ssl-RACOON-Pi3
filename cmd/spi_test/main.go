//go:build rock5a

// SPI送受信テスト用ツール（Rock5A Master → ロボットMCU Slave）
//
// 本番 racoon-pi2-rock5a と同じ 20 バイト TX/RX レイアウト。
// ヘッダ 0xFF + 18 バイトペイロード + フッタ 0xAA。
// RX は先頭 11 バイトが有効データ、続く 7 バイトはパディング（0x00）。
//
// ビルド:
//
//	go build -tags rock5a -o spi_test ./cmd/spi_test
//
// 実行例:
//
//	sudo ./spi_test                          # 1秒間隔で連続送信
//	sudo ./spi_test -interval 8ms            # 本番と同じ 125Hz
//	sudo ./spi_test -count 10 -velx 500      # VelX=500mm/s を10回送信
//	sudo ./spi_test -once -kick 50 -dribble 30
//	sudo ./spi_test -sweep -interval 16ms   # VelX: -1000→0→1000→0→-1000 を繰り返し
//	sudo ./spi_test -sweep -mismatch-only   # フレームずれ時のみ表示
//	sudo ./spi_test -pattern -once           # FF 01 02 ... 12 AA の固定パターン
//
// Ctrl+C 終了時に OK/NG パケット数の統計を表示する。
package main

import (
	"bytes"
	"encoding/binary"
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"time"

	"periph.io/x/conn/v3/physic"
	"periph.io/x/conn/v3/spi"
	"periph.io/x/conn/v3/spi/spireg"
	"periph.io/x/host/v3"
)

const (
	spiDevPath     = "/dev/spidev4.0"
	spiSpeedHz     = 1_000_000
	spiFrameSize   = 20
	spiPayloadSize = 18
	spiRecvSize    = 11
	spiFrameHeader = 0xFF
	spiFrameFooter = 0xAA
)

const (
	infoEmgStop        = 0b00000001
	infoDoCharge       = 0b00010000
	infoSignalReceived = 0b00100000
)

// sendFrame は本番 SendStruct と同じ 18 バイトペイロード（LittleEndian）
type sendFrame struct {
	VelX          int16
	VelY          int16
	VelAng        int16
	DribblePower  uint8
	KickPower     uint8
	ChipPower     uint8
	RelativeX     int16
	RelativeY     int16
	RelativeTheta int16
	CameraBallX   uint8
	CameraBallY   uint8
	Informations  uint8
}

// recvFrame は本番 SPI 受信と同じ先頭 11 バイト（リトルエンディアン）
type recvFrame struct {
	Volt              uint8
	SensorInformation uint8
	CapPower          uint8
	FlWheelSpeed      int16
	BlWheelSpeed      int16
	BrWheelSpeed      int16
	FrWheelSpeed      int16
}

// velXSweep は VelX を -max → 0 → max → 0 → -max と三角波スイープする
type velXSweep struct {
	value int
	step  int
	max   int
	dir   int // 1: 増加, -1: 減少
}

type frameStats struct {
	total int
	ok    int
	ng    int
}

func (s *frameStats) record(frameErr error) {
	s.total++
	if frameErr == nil {
		s.ok++
		return
	}
	s.ng++
}

func (s *frameStats) printSummary(interrupted bool) {
	fmt.Println("--- statistics ---")
	if interrupted {
		fmt.Println("stopped by Ctrl+C")
	}
	if s.total == 0 {
		fmt.Println("packets: 0 (no SPI exchange completed)")
		return
	}
	errPct := float64(s.ng) / float64(s.total) * 100
	fmt.Printf("packets total: %d\n", s.total)
	fmt.Printf("  OK: %d\n", s.ok)
	fmt.Printf("  NG: %d (%.2f%%)\n", s.ng, errPct)
}

func newVelXSweep(max, step int) *velXSweep {
	if step <= 0 {
		step = 100
	}
	if max <= 0 {
		max = 1000
	}
	return &velXSweep{value: -max, step: step, max: max, dir: 1}
}

func (s *velXSweep) next() int {
	current := s.value
	if s.dir > 0 {
		if current >= s.max {
			s.dir = -1
			s.value = s.max - s.step
		} else {
			s.value = current + s.step
		}
	} else {
		if current <= -s.max {
			s.dir = 1
			s.value = -s.max + s.step
		} else {
			s.value = current - s.step
		}
	}
	return current
}

func main() {
	dev := flag.String("dev", spiDevPath, "SPIデバイスパス")
	hz := flag.Int("hz", spiSpeedHz, "SPIクロック [Hz]")
	interval := flag.Duration("interval", time.Second, "送信間隔")
	count := flag.Int("count", 0, "送信回数（0で無限ループ）")
	once := flag.Bool("once", false, "1回だけ送信して終了")

	velX := flag.Int("velx", 0, "VelX [mm/s]")
	velY := flag.Int("vely", 0, "VelY [mm/s]")
	velAng := flag.Int("velang", 0, "VelAng [mrad/s]")
	dribble := flag.Int("dribble", 0, "ドリブルパワー [0-100]")
	kick := flag.Int("kick", 0, "キックパワー [0-255]")
	chip := flag.Int("chip", 0, "チップパワー [0-255]")
	camX := flag.Int("camx", 0, "カメラボールX [0-255]")
	camY := flag.Int("camy", 0, "カメラボールY [0-255]")
	charge := flag.Bool("charge", false, "Informations: DoCharge (0b00010000)")
	signalRecv := flag.Bool("signal", true, "Informations: SignalReceived (0b00100000)。本番 MW 接続時と同様")
	emgStop := flag.Bool("emgstop", false, "Informations: EmgStop (0b00000001)。起動直後 idle のみで使用")
	sweep := flag.Bool("sweep", false, "VelXを -max→0→max→0→-max とスイープする")
	sweepMax := flag.Int("sweep-max", 1000, "スイープ時のVelX最大絶対値 [mm/s]")
	sweepStep := flag.Int("sweep-step", 100, "スイープ時のVelX刻み幅 [mm/s]")
	mismatchOnly := flag.Bool("mismatch-only", false, "受信フレームずれ(NG)時のみ詳細を表示")
	pattern := flag.Bool("pattern", false, "固定テストパターン送信 (255 01 02 ... 18 170 = FF + 1..18 + AA)")

	flag.Parse()

	if *once {
		*count = 1
	}

	if _, err := host.Init(); err != nil {
		log.Fatalf("host.Init: %v", err)
	}

	port, err := spireg.Open(*dev)
	if err != nil {
		log.Fatalf("spireg.Open(%s): %v", *dev, err)
	}
	defer port.Close()

	conn, err := port.Connect(physic.Frequency(*hz)*physic.Hertz, spi.Mode0, 8)
	if err != nil {
		log.Fatalf("Connect: %v", err)
	}

	log.Printf("SPI test start: dev=%s speed=%dHz mode=0 frame=%dB recv=%dB", *dev, *hz, spiFrameSize, spiRecvSize)
	if *sweep {
		log.Printf("Sweep mode: VelX %d → 0 → %d → 0 → %d (step %d)", -*sweepMax, *sweepMax, -*sweepMax, *sweepStep)
	}
	if *mismatchOnly {
		log.Println("Mismatch-only mode: NG frames only")
	}
	if *pattern {
		log.Println("Pattern mode: TX = FF 01 02 03 ... 12 AA (decimal: 255 1..18 170)")
	}

	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt)
	defer signal.Stop(sig)

	ticker := time.NewTicker(*interval)
	defer ticker.Stop()

	var sweeper *velXSweep
	if *sweep {
		sweeper = newVelXSweep(*sweepMax, *sweepStep)
	}

	stats := &frameStats{}
	interrupted := false
	defer func() {
		stats.printSummary(interrupted)
	}()

	sent := 0
	prevFrameOK := true
loop:
	for {
		select {
		case <-sig:
			interrupted = true
			break loop
		default:
		}

		var tx []byte
		if *pattern {
			tx = buildPatternFrame()
		} else {
			velXNow := *velX
			if sweeper != nil {
				velXNow = sweeper.next()
			}
			tx = buildFrame(velXNow, *velY, *velAng, *dribble, *kick, *chip, *camX, *camY, *charge, *signalRecv, *emgStop)
		}
		rx := make([]byte, spiFrameSize)
		txCopy := append([]byte(nil), tx...)
		if err := conn.Tx(tx, rx); err != nil {
			log.Fatalf("Tx: %v", err)
		}

		sent++
		frameErr := validateSPIFrame(rx)
		stats.record(frameErr)
		frameOK := frameErr == nil
		if !frameOK && prevFrameOK {
			log.Printf("!! SPI RX FRAME MISMATCH: %v", frameErr)
		}
		if frameOK && !prevFrameOK {
			log.Println("SPI RX frame recovered")
		}
		prevFrameOK = frameOK

		if !*mismatchOnly || frameErr != nil {
			if *pattern {
				printPatternResult(sent, txCopy, rx, frameErr)
			} else {
				printResult(sent, txCopy, rx, frameErr)
			}
		}

		if *count > 0 && sent >= *count {
			break loop
		}

		select {
		case <-sig:
			interrupted = true
			break loop
		case <-ticker.C:
		}
	}
}

func buildPatternFrame() []byte {
	tx := make([]byte, spiFrameSize)
	tx[0] = spiFrameHeader
	for i := range spiPayloadSize {
		tx[1+i] = byte(i + 1)
	}
	tx[spiFrameSize-1] = spiFrameFooter
	return tx
}

func buildFrame(velX, velY, velAng, dribble, kick, chip, camX, camY int, charge, signal, emgStop bool) []byte {
	var info uint8
	if emgStop {
		info |= infoEmgStop
	}
	if signal {
		info |= infoSignalReceived
	}
	if charge {
		info |= infoDoCharge
	}

	frame := sendFrame{
		VelX:         int16(velX),
		VelY:         int16(velY),
		VelAng:       int16(velAng),
		DribblePower: clampUint8(dribble, 0, 100),
		KickPower:    clampUint8(kick, 0, 255),
		ChipPower:    clampUint8(chip, 0, 255),
		CameraBallX:  clampUint8(camX, 0, 255),
		CameraBallY:  clampUint8(camY, 0, 255),
		Informations: info,
	}

	buf := &bytes.Buffer{}
	if err := binary.Write(buf, binary.LittleEndian, frame); err != nil {
		log.Fatalf("binary.Write: %v", err)
	}
	payload := buf.Bytes()
	if len(payload) != spiPayloadSize {
		log.Fatalf("payload size: got %d bytes, want %d", len(payload), spiPayloadSize)
	}

	tx := make([]byte, spiFrameSize)
	tx[0] = spiFrameHeader
	copy(tx[1:], payload)
	tx[spiFrameSize-1] = spiFrameFooter
	return tx
}

func validateSPIFrame(rx []byte) error {
	if len(rx) < spiFrameSize {
		return fmt.Errorf("short frame: got %d bytes, want %d", len(rx), spiFrameSize)
	}
	if rx[0] != spiFrameHeader {
		return fmt.Errorf("header: expected %02x, got %02x", spiFrameHeader, rx[0])
	}
	if rx[spiFrameSize-1] != spiFrameFooter {
		return fmt.Errorf("footer: expected %02x, got %02x", spiFrameFooter, rx[spiFrameSize-1])
	}
	for i := 1 + spiRecvSize; i < spiFrameSize-1; i++ {
		if rx[i] != 0 {
			return fmt.Errorf("padding[%d]: expected 00, got %02x", i, rx[i])
		}
	}
	return nil
}

func printPatternResult(n int, tx, rx []byte, frameErr error) {
	status := "OK"
	if frameErr != nil {
		status = "NG"
	}
	fmt.Printf("[%d] %s TX pattern: ", n, status)
	for i, b := range tx {
		if i > 0 {
			fmt.Print(" ")
		}
		fmt.Print(b)
	}
	fmt.Println()
	fmt.Printf("     TX raw (%dB): % x\n", len(tx), tx)
	fmt.Printf("     RX raw (%dB): % x\n", len(rx), rx)
	if frameErr != nil {
		fmt.Printf("     !! %v\n", frameErr)
	}
}

func printResult(n int, tx, rx []byte, frameErr error) {
	var recv recvFrame
	if err := binary.Read(bytes.NewReader(rx[1:1+spiRecvSize]), binary.LittleEndian, &recv); err != nil {
		log.Printf("[%d] RX parse error: %v", n, err)
		return
	}

	var sent sendFrame
	_ = binary.Read(bytes.NewReader(tx[1:1+spiPayloadSize]), binary.LittleEndian, &sent)

	status := "OK"
	if frameErr != nil {
		status = "NG"
	}

	fmt.Printf("[%d] %s TX vel=(%d,%d,%d) dribble=%d kick=%d chip=%d info=0b%08b\n",
		n, status, sent.VelX, sent.VelY, sent.VelAng,
		sent.DribblePower, sent.KickPower, sent.ChipPower, sent.Informations)
	fmt.Printf("     RX volt=%d (%.1fV) sensor=0b%08b cap=%d wheels=(%d,%d,%d,%d)\n",
		recv.Volt, float32(recv.Volt)*0.1, recv.SensorInformation, recv.CapPower,
		recv.FlWheelSpeed, recv.BlWheelSpeed, recv.BrWheelSpeed, recv.FrWheelSpeed)
	fmt.Printf("     TX raw (%dB): % x\n", len(tx), tx)
	fmt.Printf("     RX raw (%dB): % x\n", len(rx), rx)
	if frameErr != nil {
		fmt.Printf("     !! %v\n", frameErr)
	}
}

func clampUint8(v, min, max int) uint8 {
	if v < min {
		v = min
	}
	if v > max {
		v = max
	}
	return uint8(v)
}
