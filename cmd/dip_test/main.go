//go:build rock5a

// DIPスイッチ診断ツール（Rock5A）
//
// sysfs の生値・反転後のビット・IOC MMIO のプル設定を表示する。
// プル設定は rock5a-gpio-go v0.2.0 以降の gpio.SetPull を使用する。
//
// ビルド:
//
//	go build -o dip-test ./cmd/dip_test
//
// 実行例:
//
//	sudo ./dip-test                         # 内部プルアップ有効で 0.5 秒間隔
//	sudo ./dip-test -once                   # 1 回だけ表示
//	sudo ./dip-test -pull floating          # プルなしで読み取り
//	sudo ./dip-test -pull down              # 内部プルダウンで読み取り
//	sudo ./dip-test -pull compare           # floating / up / down を順に比較
//	sudo ./dip-test -no-apply               # SetPull せず IOC レジスタ現状のみ
package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	"github.com/Yuzz1e/rock5a-gpio-go"
)

type dipPin struct {
	name      string
	physPin   int
	bank      int
	port      int
	pin       int
	bitWeight int
}

var dipPins = []dipPin{
	{name: "DIP1", physPin: 7, bank: 1, port: 1, pin: 3, bitWeight: 1},
	{name: "DIP2", physPin: 29, bank: 1, port: 1, pin: 2, bitWeight: 2},
	{name: "DIP3", physPin: 31, bank: 1, port: 1, pin: 1, bitWeight: 4},
	{name: "DIP4", physPin: 22, bank: 1, port: 1, pin: 5, bitWeight: 8},
}

type pullChoice int

const (
	pullFloating pullChoice = iota
	pullUp
	pullDown
	pullNone
	pullCompare
)

func parsePull(s string) (pullChoice, error) {
	switch strings.ToLower(s) {
	case "floating", "float", "none-io":
		return pullFloating, nil
	case "up", "pullup", "pull-up":
		return pullUp, nil
	case "down", "pulldown", "pull-down":
		return pullDown, nil
	case "none", "skip":
		return pullNone, nil
	case "compare":
		return pullCompare, nil
	default:
		return 0, fmt.Errorf("unknown pull mode %q (use floating|up|down|none|compare)", s)
	}
}

func (p pullChoice) gpioMode() (gpio.PullMode, bool) {
	switch p {
	case pullFloating:
		return gpio.Floating, true
	case pullUp:
		return gpio.PullUp, true
	case pullDown:
		return gpio.PullDown, true
	default:
		return gpio.Floating, false
	}
}

func (p pullChoice) label() string {
	switch p {
	case pullFloating:
		return "floating"
	case pullUp:
		return "pull-up"
	case pullDown:
		return "pull-down"
	case pullNone:
		return "(no SetPull)"
	case pullCompare:
		return "compare"
	default:
		return "?"
	}
}

type pinSample struct {
	dip          dipPin
	linuxGPIO    int
	gpioName     string
	iocAfter     uint32
	pullPE       bool
	pullPS       bool
	pullDecoded  string
	raw          string
	level        string
	switchON     uint8
	readErr      error
	pullApplyErr error
}

func gpioName(bank, port, pin int) string {
	return fmt.Sprintf("GPIO%d_%c%d", bank, rune('A'+port), pin)
}

func dipPortReg() (uint32, error) {
	d := dipPins[0]
	return readIOCReg(d.bank, d.port)
}

func applyPullAll(mode gpio.PullMode) error {
	var first error
	for _, d := range dipPins {
		if err := gpio.SetPull(d.bank, rune('A'+d.port), d.pin, mode); err != nil && first == nil {
			first = fmt.Errorf("%s: %w", d.name, err)
		}
	}
	return first
}

func readRaw(g *gpio.GPIO) (string, error) {
	v, err := g.Read()
	if err != nil {
		return "", err
	}
	if v == "0" {
		return "0", nil
	}
	return "1", nil
}

func invertedBit(raw string) uint8 {
	if raw == "0" {
		return 1
	}
	return 0
}

func readSysfsPin(d dipPin) (raw, level string, err error) {
	g, err := gpio.OpenGPIO(d.bank, d.port, d.pin)
	if err != nil {
		return "", "", err
	}
	defer g.Close()
	if err := g.SetDirection("in"); err != nil {
		return "", "", err
	}
	raw, err = readRaw(g)
	if err != nil {
		return "", "", err
	}
	if raw == "1" {
		level = "HIGH"
	} else {
		level = "LOW"
	}
	return raw, level, nil
}

func collectSamples(apply bool, mode gpio.PullMode) ([]pinSample, uint32, uint32, error) {
	before, err := dipPortReg()
	if err != nil {
		return nil, 0, 0, err
	}

	var applyErr error
	if apply {
		applyErr = applyPullAll(mode)
	}

	after, err := dipPortReg()
	if err != nil {
		return nil, before, 0, err
	}

	samples := make([]pinSample, 0, len(dipPins))
	for _, d := range dipPins {
		s := pinSample{
			dip:          d,
			linuxGPIO:    gpio.GPIONum(d.bank, d.port, d.pin),
			gpioName:     gpioName(d.bank, d.port, d.pin),
			iocAfter:     after,
			pullApplyErr: applyErr,
		}
		s.pullPE, s.pullPS, s.pullDecoded = decodePull(d.pin, after)

		raw, level, err := readSysfsPin(d)
		if err != nil {
			s.readErr = err
		} else {
			s.raw = raw
			s.level = level
			s.switchON = invertedBit(raw)
		}
		samples = append(samples, s)
	}
	return samples, before, after, nil
}

func robotID(samples []pinSample) int {
	id := 0
	for _, s := range samples {
		if s.readErr != nil {
			continue
		}
		id += int(s.switchON) * s.dip.bitWeight
	}
	return id
}

func printBanner(pull pullChoice, apply bool, once bool, interval time.Duration) {
	fmt.Println("=== DIP Switch Test (Rock5A) ===")
	fmt.Println("Pull driver: rock5a-gpio-go SetPull (v0.2.0+)")
	if once {
		fmt.Println("Mode: single shot")
	} else {
		fmt.Printf("Mode: poll every %s (Ctrl+C to stop)\n", interval)
	}
	if pull == pullCompare {
		fmt.Println("Pull: compare floating → pull-up → pull-down")
	} else if apply {
		fmt.Printf("Pull requested: %s\n", pull.label())
	} else {
		fmt.Println("Pull requested: (skipped - IOC register not modified)")
	}
	fmt.Println()
}

func printPortIOC(before, after uint32, apply bool) {
	fmt.Println("GPIO1_B IOC pull register (shared by DIP1-4)")
	fmt.Printf("  before SetPull: 0x%08X (lower=0x%04X)\n", before, before&0xFFFF)
	if apply {
		changed := ""
		if before == after {
			changed = "  !! unchanged"
		}
		fmt.Printf("  after  SetPull: 0x%08X (lower=0x%04X)%s\n", after, after&0xFFFF, changed)
	} else {
		fmt.Println("  (SetPull skipped)")
	}
	fmt.Println()
}

func printSamples(samples []pinSample) {
	fmt.Printf("%-4s %-5s %-9s %-4s %-3s %-10s %-2s %-2s %-10s %-3s %-5s %-8s\n",
		"DIP", "Phys", "GPIO", "Lin#", "Pin", "IOC(after)", "PE", "PS", "Pull(IO)", "Raw", "Level", "SW(ON=1)")
	fmt.Println(strings.Repeat("-", 88))

	for _, s := range samples {
		pe, ps, pullIO := "-", "-", s.pullDecoded
		if s.pullPE {
			pe = "1"
		} else if s.pullDecoded != "" {
			pe = "0"
		}
		if s.pullPS {
			ps = "1"
		} else if s.pullDecoded != "" {
			ps = "0"
		}

		raw, level, sw := "-", "-", "-"
		if s.readErr != nil {
			raw = "ERR"
			level = fmt.Sprintf("%v", s.readErr)
		} else {
			raw, level, sw = s.raw, s.level, fmt.Sprintf("%d", s.switchON)
		}

		fmt.Printf("%-4s Pin%-3d %-9s %-4d %-3d 0x%08X %-2s %-2s %-10s %-3s %-5s %-8s\n",
			s.dip.name, s.dip.physPin, s.gpioName, s.linuxGPIO, s.dip.pin,
			s.iocAfter, pe, ps, pullIO, raw, level, sw)

		if s.pullApplyErr != nil {
			fmt.Printf("     !! SetPull error: %v\n", s.pullApplyErr)
		}
	}

	id := robotID(samples)
	fmt.Println()
	fmt.Printf("Robot ID: %d (0b%04b)\n", id, id)
	fmt.Println()
	fmt.Println("Legend:")
	fmt.Println("  Raw/Level : sysfs (0=LOW, 1=HIGH)")
	fmt.Println("  SW(ON=1)  : active-low (ON=GND → 1)")
	fmt.Println("  PE/PS     : IOC MMIO after SetPull (PE=enable, PS=1→up)")
	fmt.Println("  OFF+pull-up → Raw=1/SW=0   ON → Raw=0/SW=1")
}

func runCompare() {
	modes := []struct {
		label string
		mode  gpio.PullMode
	}{
		{"floating", gpio.Floating},
		{"pull-up", gpio.PullUp},
		{"pull-down", gpio.PullDown},
	}

	for i, m := range modes {
		if i > 0 {
			fmt.Println()
			fmt.Println(strings.Repeat("=", 72))
			fmt.Println()
		}
		fmt.Printf("--- pull mode: %s ---\n", m.label)
		samples, before, after, err := collectSamples(true, m.mode)
		if err != nil {
			log.Printf("collect failed: %v", err)
			continue
		}
		printPortIOC(before, after, true)
		printSamples(samples)
	}

	fmt.Println("Compare notes:")
	fmt.Println("  • IOC 'after' がモードごとに変わる → MMIO プル設定は有効")
	fmt.Println("  • OFF のスイッチ: pull-up で Raw=1、pull-down で Raw=0 になるはず")
	fmt.Println("  • ON のスイッチ (GND): どのモードでも Raw=0")
}

func main() {
	interval := flag.Duration("interval", 500*time.Millisecond, "ポーリング間隔")
	once := flag.Bool("once", false, "1回だけ表示して終了")
	pullFlag := flag.String("pull", "up", "floating|up|down|none|compare")
	noApply := flag.Bool("no-apply", false, "SetPull を呼ばず IOC 現状のみ読む")
	flag.Parse()

	pull, err := parsePull(*pullFlag)
	if err != nil {
		log.Fatal(err)
	}
	if pull == pullCompare {
		*once = true
	}
	if *once {
		*interval = 0
	}

	apply := !*noApply && pull != pullCompare && pull != pullNone
	mode, hasMode := pull.gpioMode()

	printBanner(pull, apply, *once || *interval == 0, *interval)

	if pull == pullCompare {
		runCompare()
		return
	}

	tick := func() {
		if *interval > 0 {
			fmt.Printf("[%s]\n", time.Now().Format("15:04:05.000"))
		}
		var samples []pinSample
		var before, after uint32
		var err error
		if apply && hasMode {
			samples, before, after, err = collectSamples(true, mode)
		} else {
			samples, before, after, err = collectSamples(false, gpio.Floating)
		}
		if err != nil {
			log.Printf("read failed: %v", err)
			return
		}
		printPortIOC(before, after, apply && hasMode)
		printSamples(samples)
	}

	tick()

	if *once || *interval == 0 {
		return
	}

	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM)

	ticker := time.NewTicker(*interval)
	defer ticker.Stop()

	for {
		select {
		case <-sig:
			fmt.Println("Bye")
			return
		case <-ticker.C:
			fmt.Println()
			tick()
		}
	}
}
