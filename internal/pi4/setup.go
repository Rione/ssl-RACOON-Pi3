//go:build pi4

package pi4

import (
	"fmt"
	"log"
	"os"
	"os/exec"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/state"
	"github.com/Rione/ssl-RACOON-Pi2/internal/util"
	"github.com/stianeikeland/go-rpio/v4"
)

func InitBoard() {
	if err := rpio.Open(); err != nil {
		fmt.Println(err)
		os.Exit(1)
	}
}

func CleanupBoard() {
	rpio.Close()
}

func CheckInitialButtonState() bool {
	if err := rpio.Open(); err != nil {
		log.Fatal("Error: ", err)
	}
	defer rpio.Close()

	button1 := rpio.Pin(PIN_BUTTON1)
	button1.Input()
	button1.PullUp()

	return button1.Read()^1 == rpio.High
}

func SetupNewHostname() {
	unixtime := time.Now().UnixNano() % 100000
	hostname := fmt.Sprintf("racoon-%05d", unixtime)

	log.Printf("Unixtime is %d", time.Now().UnixNano())
	log.Println("Change Hostname To " + hostname)

	if err := exec.Command("hostnamectl", "set-hostname", hostname).Run(); err != nil {
		log.Printf("hostnamectl set-hostname failed: %v", err)
	}

	// Keep /etc/hosts in sync so sudo and other tools can resolve the new name.
	// Use the same sed form as rock5a (pi4 previously passed /etc/hosts as the -i
	// backup suffix, which is incorrect). Also rewrite the 127.0.1.1 line explicitly
	// because cloud-init may have left "raspberrypi" there.
	hostsScript := fmt.Sprintf(
		`sed -i -e 's/raspberrypi/%[1]s/g' -e 's/^127\.0\.1\.1.*/127.0.1.1 %[1]s %[1]s/' /etc/hosts`,
		hostname,
	)
	if err := exec.Command("sh", "-c", hostsScript).Run(); err != nil {
		log.Printf("failed to update /etc/hosts: %v", err)
	}

	log.Println("=====Reboot=====")

	if err := rpio.Open(); err == nil {
		playRebootMelody()
		rpio.Close()
	}

	exec.Command("reboot").Run()
}

func HandleLocalUserMode() bool {
	if err := rpio.Open(); err != nil {
		util.CheckError(err)
	}

	buzzer := rpio.Pin(PIN_BUZZER)
	buzzer.Mode(rpio.Pwm)
	buzzer.Freq(1175 * 64)
	buzzer.DutyCycle(16, 32)
	time.Sleep(1000 * time.Millisecond)
	buzzer.DutyCycle(0, 32)
	time.Sleep(1000 * time.Millisecond)

	button1 := rpio.Pin(PIN_BUTTON1)
	button1.Input()
	button1.PullUp()

	if button1.Read()^1 == rpio.High {
		state.IsControlByRobotMode = true
		log.Println("Robot Control Mode is ON")
		for i := 0; i < 2; i++ {
			buzzer.Freq(1244 * 64)
			buzzer.DutyCycle(16, 32)
			time.Sleep(100 * time.Millisecond)
			buzzer.DutyCycle(0, 32)
			time.Sleep(100 * time.Millisecond)
		}
		return true
	}
	return false
}

func ReadRobotIDFromDIP() int {
	dip1 := rpio.Pin(PIN_DIP1)
	dip1.Input()
	dip1.PullUp()
	dip2 := rpio.Pin(PIN_DIP2)
	dip2.Input()
	dip2.PullUp()
	dip3 := rpio.Pin(PIN_DIP3)
	dip3.Input()
	dip3.PullUp()
	dip4 := rpio.Pin(PIN_DIP4)
	dip4.Input()
	dip4.PullUp()

	fmt.Println("DIP1:", dip1.Read()^1)
	fmt.Println("DIP2:", dip2.Read()^1)
	fmt.Println("DIP3:", dip3.Read()^1)
	fmt.Println("DIP4:", dip4.Read()^1)

	return int(dip1.Read() ^ 1 + (dip2.Read()^1)*2 + (dip3.Read()^1)*4 + (dip4.Read()^1)*8)
}

func RunGPIO(done <-chan struct{}) {
	if err := rpio.Open(); err != nil {
		util.CheckError(err)
	}

	led := rpio.Pin(PIN_LED1)
	led.Output()
	led2 := rpio.Pin(PIN_LED2)
	led2.Output()

	button1 := rpio.Pin(PIN_BUTTON1)
	button1.Input()
	button1.PullUp()
	button2 := rpio.Pin(PIN_BUTTON2)
	button2.Input()
	button2.PullUp()

	playStartupMelody()
	printDIPStatus()

	ledInterval := 500 * time.Millisecond
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
	dip1 := rpio.Pin(PIN_DIP1)
	dip1.Input()
	dip1.PullUp()
	dip2 := rpio.Pin(PIN_DIP2)
	dip2.Input()
	dip2.PullUp()
	dip3 := rpio.Pin(PIN_DIP3)
	dip3.Input()
	dip3.PullUp()
	dip4 := rpio.Pin(PIN_DIP4)
	dip4.Input()
	dip4.PullUp()

	fmt.Println("DIP1:", dip1.Read()^1)
	fmt.Println("DIP2:", dip2.Read()^1)
	fmt.Println("DIP3:", dip3.Read()^1)
	fmt.Println("DIP4:", dip4.Read()^1)

	hex := dip1.Read() ^ 1 + (dip2.Read()^1)*2 + (dip3.Read()^1)*4 + (dip4.Read()^1)*8
	fmt.Println("HEX:", int(hex))
}

func handleBatteryAlarm(led2, button1 rpio.Pin, alarmVoltage *int) {
	log.Println("BATTERY ALARM")

	for {
		if state.Recvdata.Volt <= uint8(state.BatteryCriticalThreshold) {
			RingBuzzer(25, 5000*time.Millisecond, 0)
			continue
		}

		led2.High()
		go RingBuzzer(25, 50*time.Millisecond, 0)
		time.Sleep(60 * time.Millisecond)
		go RingBuzzer(20, 90*time.Millisecond, 0)
		led2.Low()
		time.Sleep(120 * time.Millisecond)

		if button1.Read()^1 == rpio.High || state.AlarmIgnore {
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

func handleNormalOperation(led, button1, button2 rpio.Pin, ledInterval time.Duration) time.Duration {
	const ledBlinkFast = 75 * time.Millisecond
	const ledBlinkNormal = 500 * time.Millisecond

	time.Sleep(ledInterval)
	led.Write(rpio.High)

	if button1.Read()^1 == rpio.High {
		ledInterval = ledBlinkFast
		go RingBuzzer(20, 20*time.Millisecond, 0)
	} else {
		ledInterval = ledBlinkNormal
	}

	if button2.Read()^1 == rpio.High {
		go RingBuzzer(10, 50*time.Millisecond, 0)
	}

	time.Sleep(ledInterval)
	led.Write(rpio.Low)

	return ledInterval
}
