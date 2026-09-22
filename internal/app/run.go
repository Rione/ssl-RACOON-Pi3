//go:build pi4 || rock5a

package app

import (
	"flag"
	"fmt"
	"log"
	"net"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"strings"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/api"
	"github.com/Rione/ssl-RACOON-Pi2/internal/mw"
	"github.com/Rione/ssl-RACOON-Pi2/internal/receive"
	"github.com/Rione/ssl-RACOON-Pi2/internal/state"
	"github.com/Rione/ssl-RACOON-Pi2/internal/upgrade"
	"github.com/Rione/ssl-RACOON-Pi2/internal/wheelgraph"
)

func kickCheck(done <-chan struct{}) {
	ticker := time.NewTicker(16 * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-done:
			return
		case <-ticker.C:
			if state.KickerEnable {
				time.Sleep(state.KickHoldDuration)
				state.KickerEnable = false
				state.KickerVal = 0
				state.DoDirectKick = false
			}
			if state.ChipEnable {
				time.Sleep(state.KickHoldDuration)
				state.ChipEnable = false
				state.ChipVal = 0
				state.DoDirectChipKick = false
			}
			if state.ImuError {
				time.Sleep(state.KickHoldDuration)
				state.ImuError = false
			}
		}
	}
}

func Run() {
	parseFlags()
	registerPlatform()

	if checkInitialButtonState() {
		log.Println("Button1 is pressed. Start Robot Control Mode")
		state.IsControlByRobotMode = true
	}

	hostname := getHostname()
	fmt.Println(hostname)

	if hostname == defaultHostname() {
		setupNewHostname()
		return
	}

	if state.IsControlByRobotMode {
		log.Println("Robot Control Mode is ON")
		hostname = "localuser\n"
	}

	if hostname == "localuser\n" {
		if !handleLocalUserMode() {
			os.Exit(0)
		}
	}

	// バージョンを記録し、PiToMw で RAVEN へ通知する（Robot Status ペイン表示用）。
	state.Version = upgrade.GetVersion()
	log.Printf("RACOON-Pi2 version: %s", state.Version)

	go upgrade.ConfirmAndSelfUpdate()

	initBoard()
	defer cleanupBoard()

	robotID := readRobotIDFromDIP()
	fmt.Println("GOT ID FROM DIP SW:", robotID)

	ip := getLocalIP()
	ipCamera := "127.0.0.1"

	setupSignalHandler()

	var myID uint32 = uint32(robotID)

	done := make(chan struct{})

	go receive.RunClient(done, myID, ip)
	go mw.RunServer(done, myID)
	go runLink(done, myID)
	go kickCheck(done)
	go runGPIO(done)
	go api.Run(done, myID)
	go receive.ReceiveData(done, myID, ipCamera)

	if state.DebugWheelGraph {
		wheelgraph.SetEnabled(true)
		go wheelgraph.RunServer(done)
	}

	select {}
}

func parseFlags() {
	flag.BoolVar(&state.DebugSerial, "ds", false, "ロボットリンク送受信のモニタリングを有効化")
	flag.BoolVar(&state.DebugReceive, "dr", false, "AIからの受信結果表示を有効化")
	flag.BoolVar(&state.DebugCamera, "dc", false, "カメラプロセスのデバッグログを有効化")
	flag.BoolVar(&state.DebugWheelGraph, "dw", false, "Wheel(raw)のリアルタイムグラフを有効化 (http://<robot>:9192/wheel-graph)")
	flag.BoolVar(&state.DryRun, "dryrun", false, "serial/SPIへ速度・キック等の動作指令を送らない")
	flag.BoolVar(&state.VelX1000, "velx1000", false, "テスト用: VelX=1000 を送信フレームに設定")
	flag.Parse()

	if state.DebugSerial {
		log.Println("Debug Mode: Link monitoring enabled (-ds)")
	}
	if state.DebugReceive {
		log.Println("Debug Mode: AI receive monitoring enabled (-dr)")
	}
	if state.DebugCamera {
		log.Println("Debug Mode: Camera logging enabled (-dc)")
	}
	if state.DebugWheelGraph {
		log.Println("Debug Mode: Wheel(raw) graph enabled (-dw)")
	}
	if state.DryRun {
		log.Println("Dry-run mode: motion commands are not sent on serial/SPI (-dryrun)")
	}
	if state.VelX1000 {
		log.Println("Test mode: VelX=1000 (-velx1000)")
	}
}

func getHostname() string {
	cmd := exec.Command("hostname")
	out, err := cmd.Output()
	if err != nil {
		panic(err)
	}
	return string(out)
}

func getLocalIP() string {
	ifaces, err := net.Interfaces()
	if err != nil {
		log.Println("Local IP for AI receive: not found (using 0.0.0.0)")
		return "0.0.0.0"
	}
	for _, iface := range ifaces {
		if !interfaceLinkUp(iface) {
			continue
		}
		addrs, err := iface.Addrs()
		if err != nil {
			continue
		}
		for _, addr := range addrs {
			ipNet, ok := addr.(*net.IPNet)
			if !ok || ipNet.IP.IsLoopback() || ipNet.IP.To4() == nil {
				continue
			}
			ip := ipNet.IP.String()
			// 同じNICのMACアドレスを記録しておく。RAVEN側で各ロボットの基板を
			// 一意に識別し、モータ個体差の管理に使う。
			if mac := iface.HardwareAddr.String(); mac != "" {
				state.MACAddress = mac
				log.Printf("Local MAC for AI receive: %s (%s)", mac, iface.Name)
			}
			log.Printf("Local IP for AI receive: %s (%s)", ip, iface.Name)
			return ip
		}
	}
	log.Println("Local IP for AI receive: not found (using 0.0.0.0)")
	return "0.0.0.0"
}

// interfaceLinkUp reports whether the interface has an active link (not merely admin-up).
// On Linux, eth0 can stay FlagUp with NO-CARRIER; operstate stays "down" until carrier is present.
func interfaceLinkUp(iface net.Interface) bool {
	if iface.Flags&net.FlagLoopback != 0 || iface.Flags&net.FlagUp == 0 {
		return false
	}
	data, err := os.ReadFile(filepath.Join("/sys/class/net", iface.Name, "operstate"))
	if err != nil {
		return iface.Flags&net.FlagRunning != 0
	}
	return strings.TrimSpace(string(data)) == "up"
}

func setupSignalHandler() {
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)
	go func() {
		for range c {
			api.StopPythonProcess()
			cleanupBoard()
			log.Println("Bye")
			os.Exit(0)
		}
	}()
}
