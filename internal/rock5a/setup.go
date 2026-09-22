//go:build rock5a

package rock5a

import (
	"fmt"
	"log"
	"os/exec"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/state"
	"github.com/Yuzz1e/rock5a-gpio-go"
)

const (
	ledBlinkNormal = 500 * time.Millisecond
	ledBlinkFast   = 75 * time.Millisecond
)

func InitBoard() {
	initBuzzerPWM()
}

func CleanupBoard() {
	if buzzerPWM != nil {
		buzzerPWM.close()
	}
}

func CheckInitialButtonState() bool {
	button1, err := openInputGPIO(PIN_BUTTON1_BANK, PIN_BUTTON1_PORT, PIN_BUTTON1_PIN)
	if err != nil {
		log.Fatalf("Button1 pin request failed: %v", err)
	}
	defer button1.Close()
	return isPressed(button1)
}

func SetupNewHostname() {
	unixtime := time.Now().UnixNano() % 100000
	hostname := fmt.Sprintf("racoon-%05d", unixtime)

	log.Printf("Unixtime is %d", time.Now().UnixNano())
	log.Println("Change Hostname To " + hostname)

	exec.Command("sudo", "sh", "-c", fmt.Sprintf("echo '%s' > /etc/hostname", hostname)).Run()
	exec.Command("sudo", "hostname", hostname).Run()
	exec.Command("sudo", "sed", "-i", "s/DietPi/"+hostname+"/g", "/etc/hosts").Run()

	log.Println("=====Reboot=====")

	playRebootMelody()

	exec.Command("reboot").Run()
}

func playRebootMelody() {
	notes := []int{1175, 1396, 1760}
	for _, freq := range notes {
		ringBuzzerDirect(freq, 500*time.Millisecond)
	}
}

func HandleLocalUserMode() bool {
	ringBuzzerDirect(1175, 1000*time.Millisecond)
	time.Sleep(1000 * time.Millisecond)

	button1, err := openInputGPIO(PIN_BUTTON1_BANK, PIN_BUTTON1_PORT, PIN_BUTTON1_PIN)
	if err != nil {
		log.Fatalf("Button1 pin request failed: %v", err)
	}
	defer button1.Close()

	if isPressed(button1) {
		state.IsControlByRobotMode = true
		log.Println("Robot Control Mode is ON")
		for i := 0; i < 2; i++ {
			ringBuzzerDirect(1244, 100*time.Millisecond)
			time.Sleep(100 * time.Millisecond)
		}
		return true
	}
	return false
}

func ReadRobotIDFromDIP() int {
	dip1, err := openDIPInputGPIO(PIN_DIP1_BANK, PIN_DIP1_PORT, PIN_DIP1_PIN)
	if err != nil {
		log.Fatalf("DIP1 pin request failed: %v", err)
	}
	defer dip1.Close()
	dip2, err := openDIPInputGPIO(PIN_DIP2_BANK, PIN_DIP2_PORT, PIN_DIP2_PIN)
	if err != nil {
		log.Fatalf("DIP2 pin request failed: %v", err)
	}
	defer dip2.Close()
	dip3, err := openDIPInputGPIO(PIN_DIP3_BANK, PIN_DIP3_PORT, PIN_DIP3_PIN)
	if err != nil {
		log.Fatalf("DIP3 pin request failed: %v", err)
	}
	defer dip3.Close()
	dip4, err := openDIPInputGPIO(PIN_DIP4_BANK, PIN_DIP4_PORT, PIN_DIP4_PIN)
	if err != nil {
		log.Fatalf("DIP4 pin request failed: %v", err)
	}
	defer dip4.Close()

	fmt.Println("DIP1:", readDIP(dip1))
	fmt.Println("DIP2:", readDIP(dip2))
	fmt.Println("DIP3:", readDIP(dip3))
	fmt.Println("DIP4:", readDIP(dip4))

	return int(readDIP(dip1) + readDIP(dip2)*2 + readDIP(dip3)*4 + readDIP(dip4)*8)
}

func openOutputGPIO(bank, port, pin int, initialHigh bool) (*gpio.GPIO, error) {
	g, err := gpio.OpenGPIO(bank, port, pin)
	if err != nil {
		return nil, err
	}
	if err := g.SetDirection("out"); err != nil {
		g.Close()
		return nil, err
	}
	val := "0"
	if initialHigh {
		val = "1"
	}
	if err := g.Write(val); err != nil {
		g.Close()
		return nil, err
	}
	return g, nil
}

func openInputGPIO(bank, port, pin int) (*gpio.GPIO, error) {
	g, err := gpio.OpenGPIO(bank, port, pin)
	if err != nil {
		return nil, err
	}
	if err := g.SetDirection("in"); err != nil {
		g.Close()
		return nil, err
	}
	return g, nil
}

func openDIPInputGPIO(bank, port, pin int) (*gpio.GPIO, error) {
	if err := gpio.SetPull(bank, rune('A'+port), pin, gpio.PullUp); err != nil {
		return nil, err
	}
	return openInputGPIO(bank, port, pin)
}

func readDIP(g *gpio.GPIO) uint8 {
	v, err := g.Read()
	if err != nil {
		log.Printf("GPIO read error: %v", err)
		return 0
	}
	if v == "1" {
		return 1
	}
	return 0
}

func isPressed(g *gpio.GPIO) bool {
	v, err := g.Read()
	if err != nil {
		return false
	}
	return v == "0"
}

func setOutput(g *gpio.GPIO, high bool) {
	if high {
		_ = g.Write("1")
	} else {
		_ = g.Write("0")
	}
}

func RunGPIO(done <-chan struct{}) {
	led, err := openOutputGPIO(PIN_LED1_BANK, PIN_LED1_PORT, PIN_LED1_PIN, false)
	if err != nil {
		log.Printf("GPIO LED1 request failed (%d,%d,%d): %v", PIN_LED1_BANK, PIN_LED1_PORT, PIN_LED1_PIN, err)
		return
	}
	defer led.Close()

	led2, err := openOutputGPIO(PIN_LED2_BANK, PIN_LED2_PORT, PIN_LED2_PIN, false)
	if err != nil {
		log.Printf("GPIO LED2 request failed (%d,%d,%d): %v", PIN_LED2_BANK, PIN_LED2_PORT, PIN_LED2_PIN, err)
		return
	}
	defer led2.Close()

	button1, err := openInputGPIO(PIN_BUTTON1_BANK, PIN_BUTTON1_PORT, PIN_BUTTON1_PIN)
	if err != nil {
		log.Printf("GPIO Button1 request failed: %v", err)
		return
	}
	defer button1.Close()

	button2, err := openInputGPIO(PIN_BUTTON2_BANK, PIN_BUTTON2_PORT, PIN_BUTTON2_PIN)
	if err != nil {
		log.Printf("GPIO Button2 request failed: %v", err)
		return
	}
	defer button2.Close()

	playStartupMelody()
	printDIPStatus()

	ledInterval := ledBlinkNormal
	alarmVoltage := state.BatteryLowThreshold

	for {
		select {
		case <-done:
			return
		default:
			if state.Recvdata.Volt <= uint8(alarmVoltage) {
				handleBatteryAlarm(led2, button1, &alarmVoltage)
			} else {
				ledInterval = handleNormalOperation(led, button1, button2, ledInterval)
			}
		}
	}
}

func playStartupMelody() {
	melody := []struct {
		tone     int
		duration time.Duration
	}{
		{13, 150 * time.Millisecond},
		{9, 150 * time.Millisecond},
		{4, 150 * time.Millisecond},
		{9, 150 * time.Millisecond},
		{11, 150 * time.Millisecond},
		{16, 300 * time.Millisecond},
	}

	for _, note := range melody {
		RingBuzzer(note.tone, note.duration, 0)
	}

	time.Sleep(75 * time.Millisecond)

	melody2 := []struct {
		tone     int
		duration time.Duration
	}{
		{4, 150 * time.Millisecond},
		{11, 150 * time.Millisecond},
		{13, 150 * time.Millisecond},
		{11, 150 * time.Millisecond},
		{4, 150 * time.Millisecond},
		{9, 300 * time.Millisecond},
	}

	for _, note := range melody2 {
		RingBuzzer(note.tone, note.duration, 0)
	}
}

func printDIPStatus() {
	dip1, err := openDIPInputGPIO(PIN_DIP1_BANK, PIN_DIP1_PORT, PIN_DIP1_PIN)
	if err != nil {
		log.Printf("GPIO DIP1 request failed: %v", err)
		return
	}
	defer dip1.Close()
	dip2, err := openDIPInputGPIO(PIN_DIP2_BANK, PIN_DIP2_PORT, PIN_DIP2_PIN)
	if err != nil {
		log.Printf("GPIO DIP2 request failed: %v", err)
		return
	}
	defer dip2.Close()
	dip3, err := openDIPInputGPIO(PIN_DIP3_BANK, PIN_DIP3_PORT, PIN_DIP3_PIN)
	if err != nil {
		log.Printf("GPIO DIP3 request failed: %v", err)
		return
	}
	defer dip3.Close()
	dip4, err := openDIPInputGPIO(PIN_DIP4_BANK, PIN_DIP4_PORT, PIN_DIP4_PIN)
	if err != nil {
		log.Printf("GPIO DIP4 request failed: %v", err)
		return
	}
	defer dip4.Close()

	fmt.Println("DIP1:", readDIP(dip1))
	fmt.Println("DIP2:", readDIP(dip2))
	fmt.Println("DIP3:", readDIP(dip3))
	fmt.Println("DIP4:", readDIP(dip4))

	hex := readDIP(dip1) + readDIP(dip2)*2 + readDIP(dip3)*4 + readDIP(dip4)*8
	fmt.Println("HEX:", int(hex))
}

func handleBatteryAlarm(led2, button1 *gpio.GPIO, alarmVoltage *int) {
	log.Println("BATTERY ALARM")

	for {
		if state.Recvdata.Volt <= uint8(state.BatteryCriticalThreshold) {
			RingBuzzer(25, 5000*time.Millisecond, 0)
			continue
		}

		setOutput(led2, true)
		go RingBuzzer(25, 50*time.Millisecond, 0)
		time.Sleep(60 * time.Millisecond)
		go RingBuzzer(20, 90*time.Millisecond, 0)
		setOutput(led2, false)
		time.Sleep(120 * time.Millisecond)

		if isPressed(button1) || state.AlarmIgnore {
			log.Println("BATTERY ALARM IGNORED")
			*alarmVoltage = state.BatteryCriticalThreshold
			playAlarmDismissSound()
			break
		}
	}
}

func playAlarmDismissSound() {
	for i := 0; i < 3; i++ {
		time.Sleep(100 * time.Millisecond)
		go RingBuzzer(20, 20*time.Millisecond, 0)
		time.Sleep(300 * time.Millisecond)
	}
}

func PlayBallDetectedSound() {
	for state.ImageDataPtr != nil && state.ImageDataPtr.IsBallExit {
		RingBuzzer(10, 50*time.Millisecond, 0)
		time.Sleep(80 * time.Millisecond)
	}
}

func handleNormalOperation(led, button1, button2 *gpio.GPIO, ledInterval time.Duration) time.Duration {
	time.Sleep(ledInterval)
	setOutput(led, true)

	if isPressed(button1) {
		ledInterval = ledBlinkFast
		go RingBuzzer(20, 20*time.Millisecond, 0)
	} else {
		ledInterval = ledBlinkNormal
	}

	if isPressed(button2) {
		go RingBuzzer(10, 50*time.Millisecond, 0)
	}

	time.Sleep(ledInterval)
	setOutput(led, false)

	return ledInterval
}
