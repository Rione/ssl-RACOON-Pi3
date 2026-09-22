// vision_rec は SSL-Vision の生の検出を 1 台ぶん CSV に記録する (受信するだけ)。
// 時刻は Linux の CLOCK_MONOTONIC [ns] で、Java の System.nanoTime と同じ時計。
// RAVEN の軌道追従の比較 (--trajpoc) と同時に回し、traj_eval で位置の良し悪しを出す。
//
//	vision_rec -team blue -id 15 -iface wlp113s0f0 -out rec.csv   (Ctrl+C か -sec で終わる)
package main

import (
	"flag"
	"fmt"
	"net"
	"os"
	"os/signal"
	"syscall"
	"time"

	"golang.org/x/sys/unix"
	"google.golang.org/protobuf/proto"

	"github.com/Rione/ssl-RACOON-Pi3/proto/pb_gen"
)

func monoNs() int64 {
	var ts unix.Timespec
	_ = unix.ClockGettime(unix.CLOCK_MONOTONIC, &ts)
	return ts.Nano()
}

func main() {
	addr := flag.String("addr", "224.5.23.2:10006", "SSL-Vision のマルチキャスト")
	iface := flag.String("iface", "", "受信に使う NIC")
	team := flag.String("team", "blue", "blue | yellow")
	id := flag.Uint("id", 0, "記録する機体の模様の番号")
	out := flag.String("out", "", "出力 CSV (必須)")
	sec := flag.Float64("sec", 0, "記録する秒数 (0 なら Ctrl+C まで)")
	flag.Parse()
	if *out == "" {
		fmt.Fprintln(os.Stderr, "vision_rec: -out is required")
		os.Exit(2)
	}
	ga, err := net.ResolveUDPAddr("udp4", *addr)
	check(err)
	var ifi *net.Interface
	if *iface != "" {
		ifi, err = net.InterfaceByName(*iface)
		check(err)
	}
	conn, err := net.ListenMulticastUDP("udp4", ifi, ga)
	check(err)
	f, err := os.Create(*out)
	check(err)
	defer f.Close()
	fmt.Fprintf(f, "# vision_rec team=%s id=%d addr=%s\n", *team, *id, *addr)
	fmt.Fprintln(f, "arrival_mono_ns,t_capture_s,t_sent_s,camera,x_mm,y_mm,theta_rad,confidence")

	stop := make(chan os.Signal, 1)
	signal.Notify(stop, os.Interrupt, syscall.SIGTERM)
	var deadline time.Time
	if *sec > 0 {
		deadline = time.Now().Add(time.Duration(*sec * float64(time.Second)))
	}
	buf := make([]byte, 65536)
	n := 0
	for {
		select {
		case <-stop:
			fmt.Fprintf(os.Stderr, "vision_rec: %d frames -> %s\n", n, *out)
			return
		default:
		}
		if !deadline.IsZero() && time.Now().After(deadline) {
			fmt.Fprintf(os.Stderr, "vision_rec: %d frames -> %s\n", n, *out)
			return
		}
		_ = conn.SetReadDeadline(time.Now().Add(200 * time.Millisecond))
		k, _, err := conn.ReadFromUDP(buf)
		if err != nil {
			continue
		}
		arrival := monoNs()
		var pkt pb_gen.SSL_WrapperPacket
		if proto.Unmarshal(buf[:k], &pkt) != nil || pkt.GetDetection() == nil {
			continue
		}
		det := pkt.GetDetection()
		robots := det.GetRobotsBlue()
		if *team == "yellow" {
			robots = det.GetRobotsYellow()
		}
		for _, r := range robots {
			if r.GetRobotId() != uint32(*id) {
				continue
			}
			fmt.Fprintf(f, "%d,%.6f,%.6f,%d,%.2f,%.2f,%.5f,%.3f\n", arrival, det.GetTCapture(), det.GetTSent(),
				det.GetCameraId(), r.GetX(), r.GetY(), r.GetOrientation(), r.GetConfidence())
			n++
		}
	}
}

func check(err error) {
	if err != nil {
		fmt.Fprintln(os.Stderr, "vision_rec:", err)
		os.Exit(1)
	}
}
