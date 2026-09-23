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
	"path/filepath"
	"sort"
	"strconv"
	"strings"
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
	blink := flag.Float64("blink", 0, "手順 2: N 秒間、1 秒ごとに指令受信ビットを on/off して LED2 を意図的に点滅させる")
	txsweep := flag.Bool("txsweep", false, "手順 5: 送信側を 0-7 ビットずらしながらドリブラを回し、どれで噛み合うかを見る")
	align := flag.Int("align", 0, "手順 1c: N 回ぶん、毎フレームのビットずれ量を測って安定しているか見る")
	raw := flag.Int("raw", 0, "手順 1b: 1 回の転送で N バイト連続で読んで、そのまま 16 進で出す (解析用)")
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
	warnOtherSPIUsers()
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM)

	switch {
	case *blink > 0:
		runBlink(*frameSize, *blink, sig)
	case *txsweep:
		runTxSweep(*frameSize, *dribble, sig)
	case *align > 0:
		runAlign(*align, *frameSize)
	case *raw > 0:
		runRaw(*raw, *frameSize)
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

// warnOtherSPIUsers は自分以外に SPI を開いているプロセスがいないか調べる。
//
// 本番サービスが動いたまま試験を流すと、2 つのマスターが同じ線に指令を流すことになる。
// STM はどちらのフレームも受け取ってしまうので、こちらの指令は相手の指令に上書きされ、
// 「下りが届かない」ように見える。2026-09-23 に実際これで何時間も遠回りした。
func warnOtherSPIUsers() {
	procs, _ := filepath.Glob("/proc/[0-9]*/fd/*")
	self := os.Getpid()
	seen := map[string]bool{}
	for _, fd := range procs {
		if tgt, err := os.Readlink(fd); err != nil || tgt != devPath {
			continue
		}
		parts := strings.Split(fd, "/")
		if len(parts) < 3 {
			continue
		}
		pid := parts[2]
		if pid == strconv.Itoa(self) || seen[pid] {
			continue
		}
		seen[pid] = true
		name, _ := os.ReadFile("/proc/" + pid + "/comm")
		fmt.Printf("NG  他のプロセスが %s を開いている: pid %s (%s)\n", devPath, pid, strings.TrimSpace(string(name)))
	}
	if len(seen) > 0 {
		fmt.Println("    2 つのマスターが同じ線に指令を流すと、こちらの指令は相手に上書きされる。")
		fmt.Println("    先に止めること:  systemctl stop ssl-racoon.service")
		os.Exit(1)
	}
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

// runRaw は 1 回の転送で長く読む。転送を切らないので、スレーブ側が
// 自由走行 (NSS_SOFT) でも途中で位相が変わらない。周期と境目を後から解析するため、
// 復号は一切せずそのまま出す。
func runRaw(n, frameSize int) {
	port, conn, err := open(1_000_000, spi.Mode0)
	if err != nil {
		fail("SPI を開けない: %v", err)
	}
	defer port.Close()

	// 先に本番と同じ間合いで少し送って、スレーブを起こして揃えておく。
	for i := 0; i < 250; i++ {
		rx := make([]byte, frameSize)
		conn.Tx(txFrame(frameSize, 0, 0, 0, 0, infoSignalReceived), rx)
		time.Sleep(8 * time.Millisecond)
	}

	// 1 回の転送で n バイト。送る側はフレームを並べて埋める。
	tx := make([]byte, n)
	f := txFrame(frameSize, 0, 0, 0, 0, infoSignalReceived)
	for i := 0; i < n; i++ {
		tx[i] = f[i%frameSize]
	}
	rx := make([]byte, n)
	if err := conn.Tx(tx, rx); err != nil {
		fail("Tx: %v", err)
	}
	fmt.Printf("RAW %d\n", n)
	for i := 0; i < n; i += 32 {
		e := i + 32
		if e > n {
			e = n
		}
		fmt.Printf("%04d % x\n", i, rx[i:e])
	}
	// 止めておく
	for i := 0; i < 10; i++ {
		conn.Tx(txFrame(frameSize, 0, 0, 0, 0, infoEmgStop), make([]byte, frameSize))
		time.Sleep(8 * time.Millisecond)
	}
}

// runAlign は本番と同じ間合いで 1 フレームずつ読み、毎回のビットずれ量を測る。
//
// ずれが毎回同じなら、受信側でずらし直すだけで中身を取り戻せる (ファームを待たずに動かせる)。
// ずれが毎回変わるなら、ずらし直しでは追いつかないので配線かファームを直すしかない。
//
// ずれた分を拾えるよう、1 フレームぶん多く読む。
func runAlign(n, frameSize int) {
	port, conn, err := open(1_000_000, spi.Mode0)
	if err != nil {
		fail("SPI を開けない: %v", err)
	}
	defer port.Close()
	defer func() {
		for i := 0; i < 10; i++ {
			conn.Tx(txFrame(frameSize, 0, 0, 0, 0, infoEmgStop), make([]byte, frameSize))
			time.Sleep(8 * time.Millisecond)
		}
	}()

	fmt.Printf("手順 1c: %d フレームぶん、ビットずれ量を測る (速度 0。車輪は回らない)\n\n", n)

	readLen := frameSize*2 + 1
	tx := make([]byte, readLen)
	f := txFrame(frameSize, 0, 0, 0, 0, infoSignalReceived)
	for i := range tx {
		tx[i] = f[i%frameSize]
	}

	hist := map[int]int{}
	found, missing := 0, 0
	var first, last []byte
	tick := time.NewTicker(8 * time.Millisecond)
	defer tick.Stop()
	for i := 0; i < n; i++ {
		<-tick.C
		rx := make([]byte, readLen)
		if err := conn.Tx(tx, rx); err != nil {
			fmt.Printf("  Tx: %v\n", err)
			continue
		}
		k, frames := bestShift(rx)
		if frames == 0 {
			missing++
			continue
		}
		found++
		hist[k]++
		b := bitShift(rx, k)
		for j := 0; j+20 < len(b); j++ {
			if b[j] == header && b[j+20] == footer {
				last = append([]byte(nil), b[j:j+21]...)
				if first == nil {
					first = last
				}
				break
			}
		}
	}

	fmt.Printf("  読めた %d / 取り出せなかった %d\n\n  ビットずれ量の分布:\n", found, missing)
	keys := make([]int, 0, len(hist))
	for k := range hist {
		keys = append(keys, k)
	}
	sort.Ints(keys)
	for _, k := range keys {
		fmt.Printf("    %d ビット: %d 回 (%.0f%%)\n", k, hist[k], 100*float64(hist[k])/float64(found))
	}
	if first != nil {
		fmt.Printf("\n  最初のフレーム: % x\n", first)
		fmt.Printf("  最後のフレーム: % x\n", last)
		show := func(tag string, f []byte) {
			i16 := func(i int) int {
				v := int(f[i]) | int(f[i+1])<<8
				if v >= 32768 {
					v -= 65536
				}
				return v
			}
			fmt.Printf("  %s 電圧 %.1f V  車輪 %.2f %.2f %.2f %.2f rad/s  加速度 %d %d mg  yawRate %.3f rad/s  yaw %.3f rad\n",
				tag, float64(f[1])*0.2,
				float64(i16(4))*0.01, float64(i16(6))*0.01, float64(i16(8))*0.01, float64(i16(10))*0.01,
				i16(12), i16(14), float64(i16(16))/900, float64(i16(18))/10000)
		}
		show("最初:", first)
		show("最後:", last)
	}

	fmt.Println("\n=== 判定 ===")
	switch {
	case found == 0:
		fmt.Println("1 フレームも取り出せない → 手順 2 (LED2) へ。")
	case len(hist) == 1 && missing*10 < found:
		k := keys[0]
		if k == 0 {
			fmt.Println("ずれていない。SPI は正常。→ 手順 6 (走らせる) へ。")
		} else {
			fmt.Printf("ずれは %d ビットで一定。受信側でずらし直せば今すぐ使える。\n", k)
			fmt.Println("  → internal/rock5a/spi.go の枠探しをビット単位に広げれば通る。")
			fmt.Println("  ただし根本はスレーブが任意のビット位置で走り出していること。")
			fmt.Println("  CS を配線して SPI_NSS_HARD_INPUT に戻すのが本筋の直し方。")
		}
	default:
		fmt.Println("ずれ量が一定しない。ずらし直しでは追いつかない。")
		fmt.Println("  → 毎フレームの境目でスレーブを数え直させる必要がある。")
		fmt.Println("     CS を配線して SPI_NSS_HARD_INPUT に戻すのが本筋の直し方。")
		fmt.Println("     NSS_SOFT のままにするなら、SPI_CR1 の SSI を下げて")
		fmt.Println("     スレーブが常に選択された状態になっているかファーム側で確認する。")
	}
}

// bitShiftRight は列全体を k ビット「遅らせる」。スレーブの区切りがこちらより
// k ビット後ろにずれているとき、送る側を同じだけ遅らせれば相手の区切りに乗る。
func bitShiftRight(b []byte, k int) []byte {
	if k == 0 {
		return append([]byte(nil), b...)
	}
	out := make([]byte, len(b)+1)
	for i := range b {
		out[i] |= b[i] >> uint(k)
		out[i+1] = b[i] << uint(8-k)
	}
	return out
}

// runTxSweep は送信側のビットずらし量を 0 から 7 まで変えながらドリブラを回す。
//
// 受信が k ビットずれているなら、スレーブはこちらの MOSI も同じだけずれて読んでいる。
// つまり車輪が回らないのは「指令が届いていない」からで、送る側を同じだけずらせば届くはず。
// どれかでドリブラが回れば、ビットずれが原因だと確定する。
//
// **ドリブラが回る。車輪は回らない。**
func runTxSweep(frameSize, dribble int, sig chan os.Signal) {
	if dribble <= 0 {
		dribble = 80
	}
	port, conn, err := open(1_000_000, spi.Mode0)
	if err != nil {
		fail("SPI を開けない: %v", err)
	}
	defer port.Close()

	// まず受信側のずれを測って、予想を出しておく。
	probe := make([]byte, frameSize*2+1)
	for i := 0; i < 125; i++ {
		conn.Tx(txFrame(frameSize, 0, 0, 0, 0, infoSignalReceived), make([]byte, frameSize))
		time.Sleep(8 * time.Millisecond)
	}
	conn.Tx(probe, probe)
	rxShift, frames := bestShift(probe)
	fmt.Printf("手順 5: 送信側のビットずらしを 0-7 で試す (ドリブラ %d。車輪は回らない)\n", dribble)
	if frames > 0 {
		fmt.Printf("  受信のずれは %d ビット → 送信も %d ビットずらすと噛み合うはず\n", rxShift, rxShift)
	}
	fmt.Println("  各 2 秒。ドリブラが回った番号を覚えておくこと。")
	fmt.Println()

	// フレームを 48 個ぶん並べてから、まとめてずらす。
	// 途中で切っても相手の数えは続くので、バイト境目で分けて送ってよい。
	const framesPerBuf = 48
	base := make([]byte, 0, frameSize*framesPerBuf)
	f := txFrame(frameSize, 0, 0, 0, dribble, infoSignalReceived)
	for i := 0; i < framesPerBuf; i++ {
		base = append(base, f...)
	}
	stop := txFrame(frameSize, 0, 0, 0, 0, infoEmgStop)
	defer func() {
		for i := 0; i < 20; i++ {
			conn.Tx(stop, make([]byte, frameSize))
			time.Sleep(8 * time.Millisecond)
		}
		fmt.Println("止めた")
	}()

	for k := 0; k < 8; k++ {
		select {
		case <-sig:
			fmt.Println("signal: 止めます")
			return
		default:
		}
		mark := ""
		if frames > 0 && k == rxShift {
			mark = "  ← 予想はここ"
		}
		fmt.Printf("  %d ビットずらし%s\n", k, mark)
		buf := bitShiftRight(base, k)
		deadline := time.Now().Add(2 * time.Second)
		for time.Now().Before(deadline) {
			for off := 0; off < len(buf); off += 1008 {
				e := off + 1008
				if e > len(buf) {
					e = len(buf)
				}
				conn.Tx(buf[off:e], make([]byte, e-off))
			}
			time.Sleep(4 * time.Millisecond)
		}
		// 次に移る前にいったん止める
		for i := 0; i < 10; i++ {
			conn.Tx(stop, make([]byte, frameSize))
			time.Sleep(8 * time.Millisecond)
		}
	}
	fmt.Println()
	fmt.Println("=== 判定 ===")
	fmt.Println("  回った番号があった → ビットずれが原因だと確定。ファーム側で NSS を直す。")
	fmt.Println("  どれでも回らなかった → 下りは別の理由で届いていない。手順 2 (LED2) へ。")
}

// runBlink は 1 秒ごとに is_signal_received を立てたり落としたりする。
//
// ファーム (main_mode.c) では LED2 がこのビットをそのまま映すので、
// **こちらの合図どおりに点滅する LED があれば、それが LED2 で、下りは届いている**。
// 基板上のどれが LED2 か分からなくても判定できるのが狙い。
// 生存表示 (heart_beat) はゆっくり明滅し続けるだけなので、1 秒の点滅とは区別できる。
//
// 速度は 0 のままなので車輪は回らない。
func runBlink(frameSize int, sec float64, sig chan os.Signal) {
	port, conn, err := open(1_000_000, spi.Mode0)
	if err != nil {
		fail("SPI を開けない: %v", err)
	}
	defer port.Close()

	fmt.Printf("手順 2: %.0f 秒間、1 秒ごとに LED2 を点滅させる (速度 0。車輪は回らない)\n", sec)
	fmt.Println("  基板の LED をひとつずつ見て、下の「点ける/消す」に合わせて動くものを探すこと。")
	fmt.Println("  ゆっくり明滅し続けるものは生存表示なので違う。")
	fmt.Println()

	// 消す側は 0x00 (非常停止も立てない)。立てるのは指令受信ビットだけ。
	send := func(info byte) {
		conn.Tx(txFrame(frameSize, 0, 0, 0, 0, info), make([]byte, frameSize))
	}
	defer func() {
		for i := 0; i < 10; i++ {
			send(infoEmgStop)
			time.Sleep(8 * time.Millisecond)
		}
		fmt.Println("止めた")
	}()

	deadline := time.Now().Add(time.Duration(sec * float64(time.Second)))
	on := true
	for time.Now().Before(deadline) {
		if on {
			fmt.Println("  点ける")
		} else {
			fmt.Println("  消す")
		}
		info := byte(0x00)
		if on {
			info = infoSignalReceived
		}
		end := time.Now().Add(time.Second)
		for time.Now().Before(end) {
			select {
			case <-sig:
				fmt.Println("signal: 止めます")
				return
			default:
			}
			send(info)
			time.Sleep(8 * time.Millisecond)
		}
		on = !on
	}
	fmt.Println()
	fmt.Println("=== 判定 ===")
	fmt.Println("  合図どおりに点滅した LED がある → 下りは届いている。原因はもっと後ろ。")
	fmt.Println("  どの LED も変わらない → 下りが届いていない。MOSI (PIN_19) の配線か STM の受信側。")
}

// ---- 手順 1-2: MISO の国勢調査 ---------------------------------------------

// bitShift は受信した列を連続したビット列とみなして k ビットずらして読み直す。
// CPHA (mode) が食い違うと、中身は正しいのに全体が 1 ビットずれた形で届く。
func bitShift(b []byte, k int) []byte {
	if k == 0 {
		return b
	}
	out := make([]byte, len(b)-1)
	for i := range out {
		out[i] = b[i]<<uint(k) | b[i+1]>>uint(8-k)
	}
	return out
}

// countFrames は「ヘッダ FF、20 バイト後にフッタ AA」が何組あるかを数える。
func countFrames(b []byte) int {
	n := 0
	for i := 0; i+20 < len(b); i++ {
		if b[i] == header && b[i+20] == footer {
			n++
		}
	}
	return n
}

// bestShift は 0..7 ビットのうち、そろうフレームが最も多いずらし量を返す。
func bestShift(b []byte) (k, frames int) {
	for i := 0; i < 8; i++ {
		if n := countFrames(bitShift(b, i)); n > frames {
			k, frames = i, n
		}
	}
	return
}

// runCensus は速度と mode を総当たりして、どの設定なら噛み合うかを見る。
//
// 各設定で「まず本番と同じ間合いで起こしてから、1 回の転送で連続して読む」。
// 転送を切らずに読むので、スレーブが自由走行 (NSS_SOFT) でも途中で位相が変わらない。
// 読んだ列は 0..7 ビットずらして、ヘッダ FF とフッタ AA がそろう数を数える。
//
//	ずらし 0 でそろう  → その設定が正しい
//	ずらし 1-7 でそろう → 中身は正しいが CPHA がずれている
//	どれでもそろわない  → 返事が無いか、フレームの形が違う
func runCensus(frameSize int, sig chan os.Signal) {
	fmt.Println("手順 1: どの設定なら噛み合うか (速度 0。車輪は回らない)")
	fmt.Println()

	const rawLen = 256
	type result struct {
		speed, mode, nonFF, shift, frames int
		sample                            []byte
	}
	var results []result

	for _, sp := range speeds {
		for m := 0; m < 4; m++ {
			select {
			case <-sig:
				fmt.Println("中断")
				return
			default:
			}
			port, conn, err := open(sp, spi.Mode(m))
			if err != nil {
				fmt.Printf("  %7d Hz mode%d: SPI を開けない: %v\n", sp, m, err)
				continue
			}
			// 本番と同じ間合いで起こす (STM は 750 ms で組み直す)
			for i := 0; i < 125; i++ {
				conn.Tx(txFrame(frameSize, 0, 0, 0, 0, infoSignalReceived), make([]byte, frameSize))
				time.Sleep(8 * time.Millisecond)
			}
			// 転送を切らずに連続で読む
			tx := make([]byte, rawLen)
			f := txFrame(frameSize, 0, 0, 0, 0, infoSignalReceived)
			for i := range tx {
				tx[i] = f[i%frameSize]
			}
			rx := make([]byte, rawLen)
			if err := conn.Tx(tx, rx); err != nil {
				fmt.Printf("  %7d Hz mode%d: Tx: %v\n", sp, m, err)
				port.Close()
				continue
			}
			port.Close()

			r := result{speed: sp, mode: m, sample: rx[:12]}
			for _, x := range rx {
				if x != 0xFF {
					r.nonFF++
				}
			}
			r.shift, r.frames = bestShift(rx)
			results = append(results, r)
		}
	}

	fmt.Println("  速度      mode  ff以外  そろったフレーム  ビットずれ  最初の 12 バイト")
	okMode := -1
	shifted := false
	for _, r := range results {
		note := ""
		switch {
		case r.frames == 0:
			note = "返事なし/形が違う"
		case r.shift == 0:
			note = "★ そのまま噛み合う"
			if okMode < 0 {
				okMode = r.mode
			}
		default:
			note = fmt.Sprintf("%d ビットずれ", r.shift)
			shifted = true
		}
		fmt.Printf("  %7d Hz  %d  %5d  %8d       %d      % x  %s\n",
			r.speed, r.mode, r.nonFF, r.frames, r.shift, r.sample, note)
	}

	fmt.Println()
	fmt.Println("=== 判定 ===")
	switch {
	case okMode >= 0:
		fmt.Printf("mode%d ならそのまま噛み合う。\n", okMode)
		fmt.Printf("  → 本番 (internal/rock5a/spi.go) の spi.Mode0 を mode%d に直せば通る。\n", okMode)
	case shifted:
		fmt.Println("中身は届いているが、全体がビットずれしている = CPHA (mode) の食い違い。")
		fmt.Println("  → 上の表でずれが 0 に近い mode を使うか、ファーム側の CPOL/CPHA を戻す。")
		fmt.Println("     どの mode でもずれるなら、スレーブがクロックを 1 つ取りこぼしている。")
		fmt.Println("     NSS_SOFT では毎フレームの境目で数え直せないので、いちど始まるとずれ続ける。")
		fmt.Println("     CS を配線して SPI_NSS_HARD_INPUT に戻すのが本筋の直し方。")
	default:
		fmt.Println("どの設定でもフレームがそろわない。")
		fmt.Println("  ff 以外が 0 なら STM は MISO を駆動していない → 手順 2 (LED2) へ。")
		fmt.Println("  ff 以外があるのにそろわないなら、フレームの形が docs/SPI_PROTOCOL.md と違う。")
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
