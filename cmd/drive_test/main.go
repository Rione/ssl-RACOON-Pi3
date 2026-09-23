//go:build rock5a

// drive_test は SPI に速度の指令だけを流す最小の試験。
//
// PoC・vision・自己位置推定・RAVEN のどれも通さず、Rock5A から STM へ
// 「前へ N mm/s」を出し続けるだけ。車輪が回るかどうかの切り分けに使う。
//
//	drive_test              前へ 200 mm/s を 3 秒 (既定)
//	drive_test -vel 300 -sec 2 -frame 21
//
// **ロボットが走る。** 終わったら必ず速度 0 と非常停止を送ってから終わる
// (Ctrl+C でも同じ)。走る前に周りを空けること。
package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"syscall"
	"time"

	"periph.io/x/conn/v3/physic"
	"periph.io/x/conn/v3/spi"
	"periph.io/x/conn/v3/spi/spireg"
	"periph.io/x/host/v3"
)

const (
	devPath = "/dev/spidev4.0"
	speedHz = 1_000_000
	header  = 0xFF
	footer  = 0xAA
	// 状態バイト (SPI_PROTOCOL.md §4): bit0 非常停止 / bit5 指令を受けている
	infoEmgStop        = 0x01
	infoSignalReceived = 0x20
)

func main() {
	vel := flag.Int("vel", 200, "前への速度 [mm/s] (機体座標の x)")
	velY := flag.Int("vely", 0, "左への速度 [mm/s]")
	omega := flag.Int("omega", 0, "角速度 [mrad/s]")
	sec := flag.Float64("sec", 3, "走らせる秒数")
	frameSize := flag.Int("frame", 21, "SPI のフレーム長 (21: IMU 入りの新ファーム / 20: 旧ファーム)")
	dribble := flag.Int("dribble", 0, "ドリブラの強さ (0-255)。車輪を動かさずに『STM が指令を受け取れているか』を試すのに使う")
	flag.Parse()
	if *frameSize != 20 && *frameSize != 21 {
		log.Fatal("-frame は 20 か 21")
	}

	if _, err := host.Init(); err != nil {
		log.Fatal(err)
	}
	port, err := spireg.Open(devPath)
	if err != nil {
		log.Fatal(err)
	}
	defer port.Close()
	conn, err := port.Connect(physic.Frequency(speedHz)*physic.Hertz, spi.Mode0, 8)
	if err != nil {
		log.Fatal(err)
	}

	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM)

	send := func(vx, vy, w int, info byte) []byte {
		payload := make([]byte, 18)
		put := func(i, v int) {
			payload[i] = byte(uint16(int16(v)) & 0xff)
			payload[i+1] = byte(uint16(int16(v)) >> 8)
		}
		put(0, vx)
		put(2, vy)
		put(4, w)
		payload[6] = byte(*dribble)
		payload[17] = info
		tx := make([]byte, *frameSize)
		tx[0] = header
		copy(tx[1:], payload)
		tx[*frameSize-1] = footer
		rx := make([]byte, *frameSize)
		if err := conn.Tx(tx, rx); err != nil {
			log.Printf("SPI: %v", err)
		}
		return rx
	}

	// 止めるときは速度 0 を十分に送ってから非常停止を立てる。
	stop := func() {
		for i := 0; i < 30; i++ {
			send(0, 0, 0, infoSignalReceived)
			time.Sleep(8 * time.Millisecond)
		}
		for i := 0; i < 10; i++ {
			send(0, 0, 0, infoEmgStop)
			time.Sleep(8 * time.Millisecond)
		}
	}
	defer stop()

	fmt.Printf("drive_test: 前 %d mm/s, 左 %d mm/s, 角 %d mrad/s, ドリブラ %d を %.1f 秒 (フレーム %d バイト)\n",
		*vel, *velY, *omega, *dribble, *sec, *frameSize)
	if *dribble > 0 {
		fmt.Println("ドリブラが回れば、STM はこちらのフレームを受け取れている (走ってよい状態)")
	}
	fmt.Println("止めるときは Ctrl+C。終わりに速度 0 と非常停止を送る")

	// STM がこちらのフレームを受け取ると LED2 が点く (is_signal_received)。
	deadline := time.Now().Add(time.Duration(*sec * float64(time.Second)))
	tick := time.NewTicker(8 * time.Millisecond)
	defer tick.Stop()
	n := 0
	for time.Now().Before(deadline) {
		select {
		case <-sig:
			fmt.Println("signal: 止めます")
			return
		case <-tick.C:
		}
		rx := send(*vel, *velY, *omega, infoSignalReceived)
		n++
		if n%25 == 0 { // 0.2 秒ごとに STM から返ってきた値を出す
			ok := len(rx) == *frameSize && rx[0] == header && rx[*frameSize-1] == footer
			w := func(i int) float64 { return float64(int16(rx[i])|int16(rx[i+1])<<8) * 0.01 }
			if ok {
				fmt.Printf("  t=%4.1fs 電圧 raw=%3d  車輪 %6.2f %6.2f %6.2f %6.2f rad/s\n",
					float64(n)*0.008, rx[1], w(4), w(6), w(8), w(10))
			} else {
				fmt.Printf("  t=%4.1fs STM からの受信が壊れている: % x\n", float64(n)*0.008, rx)
			}
		}
	}
	fmt.Println("時間切れ: 止めます")
}
