//go:build rock5a

// spi_diag は「ロボットが動かない」原因を Rock5A 側から順に潰すための試験。
//
// 車輪は回らない。既定では非常停止のビットを立てたフレームしか送らないので、
// この道具を動かしている間ロボットは走らない (-led と -dribble を除く)。
//
// 使い方は docs/spi-troubleshooting.md の手順に沿う。
//
//	spi_diag                 手順 1-2: Pi 側の確認と MISO の国勢調査
//	spi_diag -loopback       手順 0: MOSI と MISO を線で繋いで Pi 自身を試す
//	spi_diag -led -sec 20    手順 3: メインボードの LED2 を見るための連続送信
//	spi_diag -dribble 80     手順 4: ドリブラだけ回して上りが通っているか見る
package main

import (
	"flag"
	"fmt"
	"os"
	"os/signal"
	"sort"
	"syscall"
	"time"

	"periph.io/x/conn/v3/physic"
	"periph.io/x/conn/v3/spi"
	"periph.io/x/conn/v3/spi/spireg"
	"periph.io/x/host/v3"
)

const (
	devPath = "/dev/spidev4.0"
	header  = 0xFF
	footer  = 0xAA

	infoEmgStop        = 0x01
	infoSignalReceived = 0x20
)

// 試す速度。STM 側が 1 MHz 前提でも、桁を変えると「同期だけずれている」のか
// 「そもそも返事が無い」のかが分かれる。
var speeds = []int{100_000, 500_000, 1_000_000, 2_000_000}

func main() {
	loopback := flag.Bool("loopback", false, "手順 0: MOSI(PIN_19) と MISO(PIN_21) を線で繋いだ状態で Pi 自身を試す")
	led := flag.Bool("led", false, "手順 3: 指令受信のビットを立てて送り続ける (速度 0。メインボードの LED2 を見る)")
	dribble := flag.Int("dribble", 0, "手順 4: ドリブラの強さ (0-255)。車輪は回さない")
	sec := flag.Float64("sec", 10, "-led / -dribble のときの秒数")
	frameSize := flag.Int("frame", 21, "フレーム長 (21: IMU 入り / 20: 旧)")
	flag.Parse()

	if _, err := host.Init(); err != nil {
		fail("periph の初期化に失敗: %v", err)
	}
	if _, err := os.Stat(devPath); err != nil {
		fail("%s が無い: %v\n  → Pi 側の SPI が有効になっていない (overlay を確認)", devPath, err)
	}
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM)

	switch {
	case *loopback:
		runLoopback()
	case *led || *dribble > 0:
		runStream(*frameSize, *sec, *dribble, sig)
	default:
		runCensus(*frameSize, sig)
	}
}

// open は速度と mode を指定して SPI を開く。periph の sysfs-spi は 1 ポートにつき
// Connect を 1 回しか許さないので、設定を変えるときは必ず開き直す。
func open(speed int, mode spi.Mode) (spi.PortCloser, spi.Conn, error) {
	port, err := spireg.Open(devPath)
	if err != nil {
		return nil, nil, err
	}
	conn, err := port.Connect(physic.Frequency(speed)*physic.Hertz, mode, 8)
	if err != nil {
		port.Close()
		return nil, nil, err
	}
	return port, conn, nil
}

func fail(format string, a ...any) {
	fmt.Printf("NG  "+format+"\n", a...)
	os.Exit(1)
}

// txFrame は本番と同じ形の下りフレームを作る。
func txFrame(size, vx, vy, w, dribble int, info byte) []byte {
	payload := make([]byte, 18)
	put := func(i, v int) {
		payload[i] = byte(uint16(int16(v)) & 0xff)
		payload[i+1] = byte(uint16(int16(v)) >> 8)
	}
	put(0, vx)
	put(2, vy)
	put(4, w)
	payload[6] = byte(dribble)
	payload[17] = info
	f := make([]byte, size)
	f[0] = header
	copy(f[1:], payload)
	f[size-1] = footer
	return f
}

// ---- 手順 0: Pi 自身の確認 -------------------------------------------------

// runLoopback は MOSI と MISO を線で繋いだ状態で、送った通りが返るかを見る。
// 返れば Rock5A の SPI (クロック・出力・入力・ドライバ) は健全だと確定する。
func runLoopback() {
	fmt.Println("手順 0: 折り返しの試験")
	fmt.Println("  PIN_19 (MOSI) と PIN_21 (MISO) を線で繋いでから実行すること。")
	fmt.Println("  メインボードは繋いだままでよい。")
	fmt.Println()
	port, conn, err := open(1_000_000, spi.Mode0)
	if err != nil {
		fail("SPI を開けない: %v", err)
	}
	defer port.Close()
	pattern := []byte{0x00, 0xFF, 0x55, 0xAA, 0x0F, 0xF0, 0x12, 0x34, 0x56, 0x78}
	rx := make([]byte, len(pattern))
	if err := conn.Tx(pattern, rx); err != nil {
		fail("Tx: %v", err)
	}
	fmt.Printf("  送信: % x\n  受信: % x\n\n", pattern, rx)
	switch {
	case eq(pattern, rx):
		fmt.Println("OK  そのまま返ってきた → Rock5A の SPI は正常。")
		fmt.Println("    線を外して手順 1 へ。以降 ff しか返らないなら原因はメインボード側。")
	case allSame(rx, 0xFF):
		fmt.Println("NG  全部 ff → 線が繋がっていないか、Pi の MISO が読めていない。")
		fmt.Println("    線を繋ぎ直して再実行。それでも ff なら Rock5A 側の配線/ピン設定を疑う。")
	default:
		fmt.Println("NG  送った値と違う → クロックかピン設定がおかしい。")
	}
}

// ---- 手順 1-2: MISO の国勢調査 ---------------------------------------------

// runCensus は速度と mode を総当たりして、MISO が一度でも下がるかを数える。
//
// SPI の MISO は誰も駆動しなければプルアップで 1 のまま (= 全部 ff)。
// 相手の中身には必ず 0 のバイトが混ざるので、何千バイト読んで ff しか無いなら
// 「同期がずれている」ではなく「相手が MISO を一切駆動していない」が確定する。
func runCensus(frameSize int, sig chan os.Signal) {
	fmt.Println("手順 1: MISO の国勢調査 (車輪は回らない)")
	fmt.Println("  速度と mode を総当たりして、STM が MISO を駆動しているかを見る。")
	fmt.Println()

	type result struct {
		speed  int
		mode   spi.Mode
		total  int
		nonFF  int
		zeros  int
		sample []byte
	}
	var results []result
	hist := map[byte]int{}
	anyNonFF := false

	modes := []spi.Mode{spi.Mode0, spi.Mode1, spi.Mode2, spi.Mode3}
	for _, sp := range speeds {
		for _, m := range modes {
			select {
			case <-sig:
				fmt.Println("中断")
				return
			default:
			}
			port, conn, err := open(sp, m)
			if err != nil {
				fmt.Printf("  %7d Hz mode%d: SPI を開けない: %v\n", sp, m&3, err)
				continue
			}
			r := result{speed: sp, mode: m}
			// 8 ms 周期で 25 フレーム = 本番と同じ間合いで 0.2 秒ぶん
			for i := 0; i < 25; i++ {
				tx := txFrame(frameSize, 0, 0, 0, 0, infoEmgStop)
				rx := make([]byte, frameSize)
				if err := conn.Tx(tx, rx); err != nil {
					fmt.Printf("  %7d Hz mode%d: Tx: %v\n", sp, m&3, err)
					break
				}
				if r.sample == nil {
					r.sample = append([]byte(nil), rx...)
				}
				for _, b := range rx {
					r.total++
					hist[b]++
					if b != 0xFF {
						r.nonFF++
					}
					if b == 0x00 {
						r.zeros++
					}
				}
				time.Sleep(8 * time.Millisecond)
			}
			port.Close()
			if r.nonFF > 0 {
				anyNonFF = true
			}
			results = append(results, r)
		}
	}

	fmt.Println("  速度      mode  読んだ  ff以外  00    最初のフレーム")
	for _, r := range results {
		fmt.Printf("  %7d Hz  %d  %6d  %6d  %4d  % x\n",
			r.speed, r.mode&3, r.total, r.nonFF, r.zeros, head(r.sample, 8))
	}
	fmt.Println()
	fmt.Println("  受信したバイトの内訳:")
	for _, kv := range topBytes(hist, 6) {
		fmt.Printf("    %02x : %d 回\n", kv.b, kv.n)
	}
	fmt.Println()

	total, nonFF := 0, 0
	for _, r := range results {
		total += r.total
		nonFF += r.nonFF
	}
	fmt.Printf("=== 判定 (%d バイト中 ff 以外が %d バイト) ===\n", total, nonFF)
	if anyNonFF {
		fmt.Println("STM は MISO を駆動している。ボードは生きていて、噛み合っていないだけ。")
		fmt.Println("  → 上の表で ff 以外が出た速度と mode を見る。")
		fmt.Println("     本番と違う mode でだけ出るなら、ファーム側の CPOL/CPHA が変わっている。")
		fmt.Println("     本番と同じ設定でも中身が壊れているなら、フレーム長かバイト順の食い違い。")
	} else {
		fmt.Println("STM は MISO を一切駆動していない (全部 ff = プルアップのまま)。")
		fmt.Println("  同期のずれではない。原因はメインボード側の 4 つのどれか:")
		fmt.Println("   A. NSS が SPI_NSS_HARD_INPUT のままで、CS が繋がっておらず常に high")
		fmt.Println("      → SPI2 が無効のまま。今回の書き換えで最も疑わしい。")
		fmt.Println("   B. MX_SPI2_Init() が走っていない / SPI_MODE_SLAVE でない")
		fmt.Println("   C. PC2 (MISO) の AF 設定が外れている、または起動直後に HardFault")
		fmt.Println("   D. メインボードに電源が来ていない")
		fmt.Println()
		fmt.Println("  → 次は手順 3 (LED2 を見る) で A/B と C を分ける。")
	}
}

// ---- 手順 3-4: LED とドリブラ -----------------------------------------------

// runStream は速度 0 のまま指令受信のビットを立てて送り続ける。
// メインボードの LED2 は is_signal_received を映すので、点けば「下りは通っている」。
func runStream(frameSize int, sec float64, dribble int, sig chan os.Signal) {
	if dribble > 0 {
		fmt.Printf("手順 4: ドリブラ %d で %.0f 秒 (車輪は回さない)\n", dribble, sec)
		fmt.Println("  回れば、Pi → STM → ドライバ の下りは通っている。")
	} else {
		fmt.Printf("手順 3: 指令受信のビットを立てて %.0f 秒 送り続ける (速度 0)\n", sec)
		fmt.Println("  メインボードの LED2 を見ること。")
	}
	fmt.Println()
	port, conn, err := open(1_000_000, spi.Mode0)
	if err != nil {
		fail("SPI を開けない: %v", err)
	}
	defer port.Close()
	send := func(d int, info byte) []byte {
		rx := make([]byte, frameSize)
		if err := conn.Tx(txFrame(frameSize, 0, 0, 0, d, info), rx); err != nil {
			fmt.Printf("  Tx: %v\n", err)
		}
		return rx
	}
	// 終わりに必ず止める
	defer func() {
		for i := 0; i < 30; i++ {
			send(0, infoSignalReceived)
			time.Sleep(8 * time.Millisecond)
		}
		for i := 0; i < 10; i++ {
			send(0, infoEmgStop)
			time.Sleep(8 * time.Millisecond)
		}
		fmt.Println("止めた")
	}()

	deadline := time.Now().Add(time.Duration(sec * float64(time.Second)))
	tick := time.NewTicker(8 * time.Millisecond)
	defer tick.Stop()
	n, alive := 0, 0
	for time.Now().Before(deadline) {
		select {
		case <-sig:
			fmt.Println("signal: 止めます")
			return
		case <-tick.C:
		}
		rx := send(dribble, infoSignalReceived)
		n++
		for _, b := range rx {
			if b != 0xFF {
				alive++
			}
		}
		if n%125 == 0 {
			fmt.Printf("  %4.0f 秒  受信で ff 以外だったバイト: %d\n", float64(n)*0.008, alive)
		}
	}
	fmt.Printf("\n終了: %d フレーム送信、受信で ff 以外は %d バイト\n", n, alive)
	if alive == 0 {
		fmt.Println("  返事は最後まで無し。LED2 が点いたかどうかで切り分ける:")
		fmt.Println("    点いた → 下りは届いている。壊れているのは返事 (MISO) だけ。")
		fmt.Println("    点かない → SPI2 が動いていない。上の A か B。")
	}
}

// ---- 小物 ------------------------------------------------------------------

func eq(a, b []byte) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

func allSame(b []byte, v byte) bool {
	for _, x := range b {
		if x != v {
			return false
		}
	}
	return len(b) > 0
}

func head(b []byte, n int) []byte {
	if len(b) < n {
		return b
	}
	return b[:n]
}

type byteCount struct {
	b byte
	n int
}

func topBytes(h map[byte]int, n int) []byteCount {
	out := make([]byteCount, 0, len(h))
	for b, c := range h {
		out = append(out, byteCount{b, c})
	}
	sort.Slice(out, func(i, j int) bool { return out[i].n > out[j].n })
	if len(out) > n {
		out = out[:n]
	}
	return out
}
