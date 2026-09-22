//go:build rock5a

// SPI キック/チップ/ドリブル対話テスト（Rock5A Master → ロボット MCU Slave）
//
// キックは指定時間だけ SPI フレームに載せ、直後に 0 に戻します。
// 連続してキック値を送り続けないよう、クールダウンと確認プロンプトを設けています。
//
// ビルド:
//
//	go build -tags rock5a -o kick_test ./cmd/kick_test
//
// 実行:
//
//	sudo ./kick_test
package main

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"fmt"
	"log"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"

	"periph.io/x/conn/v3/physic"
	"periph.io/x/conn/v3/spi"
	"periph.io/x/conn/v3/spi/spireg"
	"periph.io/x/host/v3"
)

const (
	spiDevPath   = "/dev/spidev4.0"
	spiSpeedHz   = 1_000_000
	spiFrameSize = 18
	spiRecvSize  = 11

	infoDoCharge       = 0b00010000
	infoSignalReceived = 0b00100000
	infoDirectKick     = 0b00000010
	infoDirectChip     = 0b00000100

	defaultSPIPeriod        = 8 * time.Millisecond
	defaultKickHold         = 32 * time.Millisecond
	defaultKickCooldown     = 2 * time.Second
	defaultDischargeDuration = 3 * time.Second
)

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

type recvFrame struct {
	Volt              uint8
	SensorInformation uint8
	CapPower          uint8
	FlWheelSpeed      int16
	BlWheelSpeed      int16
	BrWheelSpeed      int16
	FrWheelSpeed      int16
}

type txSettings struct {
	mu sync.Mutex

	spiPeriod time.Duration
	charge    bool

	velX   int16
	velY   int16
	velAng int16

	dribble uint8

	kickHold    time.Duration
	kickCooldown time.Duration
	lastKickAt  time.Time
}

type pulseKind int

const (
	pulseKick pulseKind = iota
	pulseChip
)

type activePulse struct {
	kind       pulseKind
	power      uint8
	directFlag bool
	expiresAt  time.Time
}

type spiRunner struct {
	conn spi.Conn
	tx   txSettings

	mu sync.Mutex

	running     bool
	shutdownOnce sync.Once
	txCount     int
	lastTX      []byte
	lastRX      []byte
	lastErr     error
	pulse       *activePulse
}

func main() {
	if _, err := host.Init(); err != nil {
		log.Fatalf("host.Init: %v", err)
	}

	port, err := spireg.Open(spiDevPath)
	if err != nil {
		log.Fatalf("spireg.Open(%s): %v", spiDevPath, err)
	}
	defer port.Close()

	conn, err := port.Connect(physic.Frequency(spiSpeedHz)*physic.Hertz, spi.Mode0, 8)
	if err != nil {
		log.Fatalf("Connect: %v", err)
	}

	runner := &spiRunner{
		conn: conn,
		tx: txSettings{
			spiPeriod:    defaultSPIPeriod,
			kickHold:     defaultKickHold,
			kickCooldown: defaultKickCooldown,
			charge:       true,
		},
	}

	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-sig
		fmt.Println("\n終了シグナル受信")
		runner.shutdown()
		os.Exit(0)
	}()

	runner.start()
	defer runner.shutdown()

	reader := bufio.NewReader(os.Stdin)
	printBanner(runner)
	runChargePhase(runner, reader)

	for {
		printMenu()
		line, err := reader.ReadString('\n')
		if err != nil {
			fmt.Println("入力エラー:", err)
			return
		}
		cmd := strings.TrimSpace(line)
		if cmd == "" {
			continue
		}

		switch strings.ToLower(cmd) {
		case "1", "k", "kick":
			doKickPulse(reader, runner, pulseKick, false)
		case "2", "c", "chip":
			doKickPulse(reader, runner, pulseChip, false)
		case "3", "dk":
			doKickPulse(reader, runner, pulseKick, true)
		case "4", "dc":
			doKickPulse(reader, runner, pulseChip, true)
		case "5", "d", "dribble":
			doDribbleTest(reader, runner)
		case "6", "s", "status":
			runner.printStatus()
		case "7", "m", "monitor":
			doMonitor(reader, runner)
		case "8", "o", "once":
			doOnce(reader, runner)
		case "9", "cfg", "config":
			doConfig(reader, runner)
		case "h", "help", "?":
			printHelp()
		case "q", "quit", "exit":
			fmt.Println("終了します。")
			return
		default:
			fmt.Printf("不明なコマンド: %q (h でヘルプ)\n", cmd)
		}
	}
}

func printBanner(r *spiRunner) {
	fmt.Println("=== SPI キック対話テスト (Rock5A) ===")
	fmt.Printf("デバイス: %s @ %d Hz, フレーム %dB\n", spiDevPath, spiSpeedHz, spiFrameSize)
	fmt.Printf("SPI周期: %s, キックホールド: %s, クールダウン: %s, チャージ: ON\n",
		r.tx.spiPeriod, r.tx.kickHold, r.tx.kickCooldown)
	fmt.Println()
	fmt.Println("⚠  キック/チップは短いパルスのみ送信します。連続送信は機構破損の恐れがあります。")
	fmt.Println()
}

func runChargePhase(r *spiRunner, reader *bufio.Reader) {
	r.tx.mu.Lock()
	r.tx.charge = true
	r.tx.mu.Unlock()

	fmt.Println("--- コンデンサ充電 ---")
	fmt.Println("InfoDoCharge を ON にして SPI 送信を開始しました。")
	fmt.Println("CapPower が十分になったら Enter でメニューへ進んでください。")
	fmt.Println()

	done := make(chan struct{})
	go func() {
		_, _ = reader.ReadString('\n')
		close(done)
	}()

	ticker := time.NewTicker(500 * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-done:
			fmt.Println("充電フェーズ完了。テストメニューへ進みます。")
			fmt.Println()
			return
		case <-ticker.C:
			capPower, volt, ok := r.lastCapStatus()
			if !ok {
				fmt.Println("  充電中... (SPI 受信待ち)")
				continue
			}
			fmt.Printf("  [%s] volt=%.1fV cap=%d (charge=ON)\n",
				time.Now().Format("15:04:05"), float32(volt)*0.1, capPower)
		}
	}
}

func (r *spiRunner) lastCapStatus() (capPower, volt uint8, ok bool) {
	r.mu.Lock()
	defer r.mu.Unlock()
	if len(r.lastRX) < spiRecvSize {
		return 0, 0, false
	}
	return r.lastRX[2], r.lastRX[0], true
}

func printMenu() {
	fmt.Println("--- メニュー ---")
	fmt.Println(" 1) ストレートキック")
	fmt.Println(" 2) チップキック")
	fmt.Println(" 3) ダイレクト・ストレートキック (InfoDirectKick)")
	fmt.Println(" 4) ダイレクト・チップキック (InfoDirectChip)")
	fmt.Println(" 5) ドリブルテスト (キックなし)")
	fmt.Println(" 6) 最新 SPI ステータス表示")
	fmt.Println(" 7) SPI モニタ (キック=0, Enter で戻る)")
	fmt.Println(" 8) 1回送受信 (任意パラメータ)")
	fmt.Println(" 9) 設定変更")
	fmt.Println(" h) ヘルプ")
	fmt.Println(" q) 終了")
	fmt.Print("> ")
}

func printHelp() {
	fmt.Println(`
使い方:
  各テスト実行前にパワー [0-255] を入力します (Enter で前回値/既定値)。
  キック系は確認プロンプト (yes) が出ます。

安全機能:
  • キック/チップ値はホールド時間だけ載せ、以降は 0 を送信
  • クールダウン中は再送信不可
  • 終了時はチャージ OFF でディスチャージ後、キック=0 で停止

設定 (9):
  hold=<ms>      キック/チップを載せる時間
  cooldown=<ms>  再キックまでの待ち時間
  period=<ms>    SPI 送信周期
  charge=on|off  コンデンサ充電フラグ (既定: on)
  vel=(x,y,ang)  ベース速度 [mm/s, mm/s, mrad/s]
`)
}

func (r *spiRunner) start() {
	r.mu.Lock()
	if r.running {
		r.mu.Unlock()
		return
	}
	r.running = true
	r.mu.Unlock()

	go r.loop()
}

func (r *spiRunner) shutdown() {
	r.shutdownOnce.Do(func() {
		r.mu.Lock()
		if r.running {
			r.running = false
		}
		r.pulse = nil
		r.mu.Unlock()

		r.tx.mu.Lock()
		r.tx.charge = false
		period := r.tx.spiPeriod
		r.tx.mu.Unlock()

		fmt.Println("コンデンサディスチャージ中 (InfoDoCharge=OFF)...")
		deadline := time.Now().Add(defaultDischargeDuration)
		nextLog := time.Now()
		for time.Now().Before(deadline) {
			tx, rx, err := r.sendSafeFrame()
			if err != nil {
				log.Printf("discharge Tx: %v", err)
			}
			r.mu.Lock()
			r.txCount++
			r.lastTX = tx
			r.lastRX = rx
			r.lastErr = err
			r.mu.Unlock()

			if time.Now().After(nextLog) {
				capPower, volt, ok := r.lastCapStatus()
				if ok {
					fmt.Printf("  [%s] volt=%.1fV cap=%d (charge=OFF)\n",
						time.Now().Format("15:04:05"), float32(volt)*0.1, capPower)
				}
				nextLog = time.Now().Add(500 * time.Millisecond)
			}
			time.Sleep(period)
		}

		for i := 0; i < 5; i++ {
			_, _, _ = r.sendSafeFrame()
			time.Sleep(period)
		}
		fmt.Println("ディスチャージ完了")
	})
}

func (r *spiRunner) sendSafeFrame() ([]byte, []byte, error) {
	r.mu.Lock()
	tx := r.buildFrameLocked(nil)
	r.mu.Unlock()

	rx := make([]byte, spiFrameSize)
	err := r.conn.Tx(tx, rx)
	return append([]byte(nil), tx...), append([]byte(nil), rx...), err
}

func (r *spiRunner) loop() {
	for {
		r.mu.Lock()
		if !r.running {
			r.mu.Unlock()
			return
		}
		period := r.tx.spiPeriod
		pulse := r.pulse
		if pulse != nil && time.Now().After(pulse.expiresAt) {
			r.pulse = nil
			pulse = nil
		}
		tx := r.buildFrameLocked(pulse)
		r.mu.Unlock()

		rx := make([]byte, spiFrameSize)
		err := r.conn.Tx(tx, rx)

		r.mu.Lock()
		r.txCount++
		r.lastTX = append([]byte(nil), tx...)
		r.lastRX = append([]byte(nil), rx...)
		r.lastErr = err
		r.mu.Unlock()

		if err != nil {
			log.Printf("SPI Tx error: %v", err)
		}

		time.Sleep(period)
	}
}

func (r *spiRunner) buildFrameLocked(pulse *activePulse) []byte {
	s := r.tx
	info := uint8(infoSignalReceived)
	if s.charge {
		info |= infoDoCharge
	}

	kick := uint8(0)
	chip := uint8(0)
	if pulse != nil {
		switch pulse.kind {
		case pulseKick:
			kick = pulse.power
			if pulse.directFlag {
				info |= infoDirectKick
			}
		case pulseChip:
			chip = pulse.power
			if pulse.directFlag {
				info |= infoDirectChip
			}
		}
	}

	return buildFrame(int(s.velX), int(s.velY), int(s.velAng), int(s.dribble), int(kick), int(chip), 0, 0, s.charge, info)
}

func buildFrame(velX, velY, velAng, dribble, kick, chip, camX, camY int, charge bool, infoOverride uint8) []byte {
	info := infoOverride
	if infoOverride == 0 {
		info = infoSignalReceived
		if charge {
			info |= infoDoCharge
		}
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
	return buf.Bytes()
}

func (r *spiRunner) armPulse(kind pulseKind, power uint8, direct bool) error {
	r.tx.mu.Lock()
	defer r.tx.mu.Unlock()

	if !r.tx.lastKickAt.IsZero() {
		remain := r.tx.kickCooldown - time.Since(r.tx.lastKickAt)
		if remain > 0 {
			return fmt.Errorf("クールダウン中: あと %s", remain.Round(time.Millisecond))
		}
	}

	hold := r.tx.kickHold
	r.tx.lastKickAt = time.Now()

	r.mu.Lock()
	r.pulse = &activePulse{
		kind:       kind,
		power:      power,
		directFlag: direct,
		expiresAt:  time.Now().Add(hold),
	}
	r.mu.Unlock()
	return nil
}

func (r *spiRunner) setDribble(power uint8) {
	r.tx.mu.Lock()
	r.tx.dribble = power
	r.tx.mu.Unlock()
}

func (r *spiRunner) clearDribble() {
	r.setDribble(0)
}

func (r *spiRunner) printStatus() {
	r.mu.Lock()
	defer r.mu.Unlock()

	if len(r.lastRX) < spiRecvSize {
		fmt.Println("まだ SPI 受信データがありません。")
		return
	}

	var recv recvFrame
	_ = binary.Read(bytes.NewReader(r.lastRX[:spiRecvSize]), binary.LittleEndian, &recv)

	var sent sendFrame
	if len(r.lastTX) >= 18 {
		_ = binary.Read(bytes.NewReader(r.lastTX), binary.LittleEndian, &sent)
	}

	frameErr := validateRecvFrame(r.lastRX)
	status := "OK"
	if frameErr != nil {
		status = "NG"
	}

	r.tx.mu.Lock()
	hold := r.tx.kickHold
	cooldown := r.tx.kickCooldown
	period := r.tx.spiPeriod
	charge := r.tx.charge
	r.tx.mu.Unlock()

	fmt.Printf("SPI TX count: %d, period: %s, hold: %s, cooldown: %s, charge: %t\n",
		r.txCount, period, hold, cooldown, charge)
	if r.lastErr != nil {
		fmt.Printf("最終 SPI エラー: %v\n", r.lastErr)
	}

	fmt.Printf("[%s] TX vel=(%d,%d,%d) dribble=%d kick=%d chip=%d info=0b%08b\n",
		status, sent.VelX, sent.VelY, sent.VelAng,
		sent.DribblePower, sent.KickPower, sent.ChipPower, sent.Informations)
	fmt.Printf("     RX volt=%d (%.1fV) sensor=0b%08b cap=%d wheels=(%d,%d,%d,%d)\n",
		recv.Volt, float32(recv.Volt)*0.1, recv.SensorInformation, recv.CapPower,
		recv.FlWheelSpeed, recv.BlWheelSpeed, recv.BrWheelSpeed, recv.FrWheelSpeed)
	fmt.Printf("     TX raw: % x\n", r.lastTX)
	fmt.Printf("     RX raw: % x\n", r.lastRX)
	if frameErr != nil {
		fmt.Printf("     !! %v\n", frameErr)
	}
	fmt.Println()
}

func validateRecvFrame(rx []byte) error {
	if len(rx) < spiFrameSize {
		return fmt.Errorf("short frame: got %d bytes, want %d", len(rx), spiFrameSize)
	}
	for i := spiRecvSize; i < spiFrameSize; i++ {
		if rx[i] != 0 {
			return fmt.Errorf("padding[%d]: expected 00, got %02x", i, rx[i])
		}
	}
	return nil
}

func readLine(reader *bufio.Reader, prompt string) string {
	fmt.Print(prompt)
	line, _ := reader.ReadString('\n')
	return strings.TrimSpace(line)
}

func readPower(reader *bufio.Reader, label string, defaultVal int) (uint8, bool) {
	line := readLine(reader, fmt.Sprintf("%s パワー [0-255, Enter=%d]: ", label, defaultVal))
	if line == "" {
		return clampUint8(defaultVal, 0, 255), true
	}
	v, err := strconv.Atoi(line)
	if err != nil || v < 0 || v > 255 {
		fmt.Println("無効な値です。0-255 の整数を入力してください。")
		return 0, false
	}
	return uint8(v), true
}

func confirm(reader *bufio.Reader, msg string) bool {
	line := readLine(reader, msg+" [yes/no]: ")
	return strings.EqualFold(line, "yes") || strings.EqualFold(line, "y")
}

func doKickPulse(reader *bufio.Reader, r *spiRunner, kind pulseKind, direct bool) {
	label := "キック"
	defaultPower := 50
	if kind == pulseChip {
		label = "チップ"
		defaultPower = 40
	}
	if direct {
		label += " (ダイレクト)"
	}

	power, ok := readPower(reader, label, defaultPower)
	if !ok {
		return
	}

	r.tx.mu.Lock()
	hold := r.tx.kickHold
	cooldown := r.tx.kickCooldown
	r.tx.mu.Unlock()

	fmt.Printf("  ホールド: %s → 自動で kick/chip=0\n", hold)
	fmt.Printf("  クールダウン: %s\n", cooldown)
	if !confirm(reader, fmt.Sprintf("%s パワー=%d を送信します。よろしいですか", label, power)) {
		fmt.Println("キャンセルしました。")
		return
	}

	if err := r.armPulse(kind, power, direct); err != nil {
		fmt.Println("送信できません:", err)
		return
	}

	fmt.Printf("✓ %s パルス送信 (power=%d, hold=%s)\n", label, power, hold)
	time.Sleep(hold + 20*time.Millisecond)
	r.printStatus()
}

func doDribbleTest(reader *bufio.Reader, r *spiRunner) {
	power, ok := readPower(reader, "ドリブル", 30)
	if !ok {
		return
	}
	durationLine := readLine(reader, "継続時間 [ms, Enter=1000]: ")
	duration := 1000 * time.Millisecond
	if durationLine != "" {
		ms, err := strconv.Atoi(durationLine)
		if err != nil || ms <= 0 {
			fmt.Println("無効な時間です。")
			return
		}
		duration = time.Duration(ms) * time.Millisecond
	}
	if !confirm(reader, fmt.Sprintf("ドリブル=%d を %s 送ります (キックなし)", power, duration)) {
		fmt.Println("キャンセルしました。")
		return
	}

	r.setDribble(power)
	fmt.Printf("ドリブル ON (power=%d) ...\n", power)
	time.Sleep(duration)
	r.clearDribble()
	fmt.Println("ドリブル OFF")
	r.printStatus()
}

func doMonitor(reader *bufio.Reader, r *spiRunner) {
	fmt.Println("SPI モニタ開始 (kick/chip=0)。Enter でメニューに戻ります。")
	ticker := time.NewTicker(500 * time.Millisecond)
	defer ticker.Stop()

	done := make(chan struct{})
	go func() {
		_, _ = reader.ReadString('\n')
		close(done)
	}()

	for {
		select {
		case <-done:
			fmt.Println("モニタ終了")
			return
		case <-ticker.C:
			r.printStatus()
		}
	}
}

func doOnce(reader *bufio.Reader, r *spiRunner) {
	velX := readIntDefault(reader, "VelX [mm/s]", 0)
	velY := readIntDefault(reader, "VelY [mm/s]", 0)
	velAng := readIntDefault(reader, "VelAng [mrad/s]", 0)
	dribble := readIntDefault(reader, "Dribble [0-100]", 0)
	kick := readIntDefault(reader, "Kick [0-255, 0推奨]", 0)
	chip := readIntDefault(reader, "Chip [0-255, 0推奨]", 0)

	if kick > 0 || chip > 0 {
		fmt.Println("⚠  1回送受信で kick/chip>0 は危険です。パルス送信 (1-4) を推奨します。")
		if !confirm(reader, "それでも 1 フレームだけ送りますか") {
			fmt.Println("キャンセルしました。")
			return
		}
	}

	r.tx.mu.Lock()
	charge := r.tx.charge
	r.tx.mu.Unlock()

	tx := buildFrame(velX, velY, velAng, dribble, kick, chip, 0, 0, charge, 0)
	rx := make([]byte, spiFrameSize)
	if err := r.conn.Tx(tx, rx); err != nil {
		fmt.Println("Tx error:", err)
		return
	}

	r.mu.Lock()
	r.txCount++
	r.lastTX = append([]byte(nil), tx...)
	r.lastRX = append([]byte(nil), rx...)
	r.lastErr = nil
	r.mu.Unlock()

	fmt.Println("1回送受信完了")
	r.printStatus()
}

func doConfig(reader *bufio.Reader, r *spiRunner) {
	fmt.Println(`設定例:
  hold=32
  cooldown=2000
  period=8
  charge=on
  vel=0,0,0
空 Enter で戻る`)
	for {
		line := readLine(reader, "cfg> ")
		if line == "" {
			return
		}
		if err := applyConfig(r, line); err != nil {
			fmt.Println("エラー:", err)
		} else {
			fmt.Println("OK")
		}
	}
}

func applyConfig(r *spiRunner, line string) error {
	parts := strings.SplitN(line, "=", 2)
	if len(parts) != 2 {
		return fmt.Errorf("key=value 形式で入力してください")
	}
	key := strings.TrimSpace(strings.ToLower(parts[0]))
	val := strings.TrimSpace(parts[1])

	r.tx.mu.Lock()
	defer r.tx.mu.Unlock()

	switch key {
	case "hold":
		ms, err := strconv.Atoi(val)
		if err != nil || ms <= 0 {
			return fmt.Errorf("hold は正の整数 [ms]")
		}
		r.tx.kickHold = time.Duration(ms) * time.Millisecond
	case "cooldown":
		ms, err := strconv.Atoi(val)
		if err != nil || ms < 0 {
			return fmt.Errorf("cooldown は 0 以上の整数 [ms]")
		}
		r.tx.kickCooldown = time.Duration(ms) * time.Millisecond
	case "period":
		ms, err := strconv.Atoi(val)
		if err != nil || ms <= 0 {
			return fmt.Errorf("period は正の整数 [ms]")
		}
		r.tx.spiPeriod = time.Duration(ms) * time.Millisecond
	case "charge":
		switch strings.ToLower(val) {
		case "on", "1", "true":
			r.tx.charge = true
		case "off", "0", "false":
			r.tx.charge = false
		default:
			return fmt.Errorf("charge=on|off")
		}
	case "vel":
		fields := strings.Split(val, ",")
		if len(fields) != 3 {
			return fmt.Errorf("vel=x,y,ang")
		}
		x, err1 := strconv.Atoi(strings.TrimSpace(fields[0]))
		y, err2 := strconv.Atoi(strings.TrimSpace(fields[1]))
		ang, err3 := strconv.Atoi(strings.TrimSpace(fields[2]))
		if err1 != nil || err2 != nil || err3 != nil {
			return fmt.Errorf("vel の各要素は整数")
		}
		r.tx.velX = int16(x)
		r.tx.velY = int16(y)
		r.tx.velAng = int16(ang)
	default:
		return fmt.Errorf("未知のキー: %s", key)
	}
	return nil
}

func readIntDefault(reader *bufio.Reader, prompt string, defaultVal int) int {
	line := readLine(reader, fmt.Sprintf("%s [Enter=%d]: ", prompt, defaultVal))
	if line == "" {
		return defaultVal
	}
	v, err := strconv.Atoi(line)
	if err != nil {
		fmt.Printf("無効な値。既定値 %d を使用\n", defaultVal)
		return defaultVal
	}
	return v
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
