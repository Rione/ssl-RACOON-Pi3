// vision_list は SSL-Vision に今写っているロボットを一覧にする (受信するだけ)。
// PoC を回す前に、-trajvisionid の模様が本当にそのロボットかを確かめるのに使う。
//
//	vision_list [-addr 224.5.23.2:10006] [-iface wlp113s0f0] [-sec 2]
package main

import (
	"flag"
	"fmt"
	"math"
	"net"
	"os"
	"sort"
	"time"

	"google.golang.org/protobuf/proto"

	"github.com/Rione/ssl-RACOON-Pi3/proto/pb_gen"
)

type seen struct {
	n                        int
	x0, y0, th0, x1, y1, th1 float64
	cams                     map[uint32]bool
}

func main() {
	addr := flag.String("addr", "224.5.23.2:10006", "SSL-Vision のマルチキャスト")
	iface := flag.String("iface", "", "受信に使う NIC")
	sec := flag.Float64("sec", 2, "受信する秒数")
	flag.Parse()
	ga, err := net.ResolveUDPAddr("udp4", *addr)
	check(err)
	var ifi *net.Interface
	if *iface != "" {
		ifi, err = net.InterfaceByName(*iface)
		check(err)
	}
	conn, err := net.ListenMulticastUDP("udp4", ifi, ga)
	check(err)
	defer conn.Close()
	robots := map[string]*seen{}
	buf := make([]byte, 65536)
	end := time.Now().Add(time.Duration(*sec * float64(time.Second)))
	frames := 0
	for time.Now().Before(end) {
		_ = conn.SetReadDeadline(end)
		n, _, err := conn.ReadFromUDP(buf)
		if err != nil {
			break
		}
		var pkt pb_gen.SSL_WrapperPacket
		if proto.Unmarshal(buf[:n], &pkt) != nil || pkt.GetDetection() == nil {
			continue
		}
		det := pkt.GetDetection()
		frames++
		add := func(team string, rs []*pb_gen.SSL_DetectionRobot) {
			for _, r := range rs {
				k := fmt.Sprintf("%-6s %2d", team, r.GetRobotId())
				s := robots[k]
				x, y, th := float64(r.GetX()), float64(r.GetY()), float64(r.GetOrientation())
				if s == nil {
					s = &seen{x0: x, y0: y, th0: th, cams: map[uint32]bool{}}
					robots[k] = s
				}
				s.n++
				s.x1, s.y1, s.th1 = x, y, th
				s.cams[det.GetCameraId()] = true
			}
		}
		add("blue", det.GetRobotsBlue())
		add("yellow", det.GetRobotsYellow())
	}
	keys := make([]string, 0, len(robots))
	for k := range robots {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	fmt.Printf("%d detection frames in %.1f s\n", frames, *sec)
	for _, k := range keys {
		s := robots[k]
		cams := []uint32{}
		for c := range s.cams {
			cams = append(cams, c)
		}
		fmt.Printf("  %s  (%6.0f, %6.0f) mm  %6.1f deg  seen %4d  moved %5.0f mm  cams %v\n", k, s.x1, s.y1,
			s.th1*180/math.Pi, s.n, math.Hypot(s.x1-s.x0, s.y1-s.y0), cams)
	}
}

func check(err error) {
	if err != nil {
		fmt.Fprintln(os.Stderr, "vision_list:", err)
		os.Exit(1)
	}
}
