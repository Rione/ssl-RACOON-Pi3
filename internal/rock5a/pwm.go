//go:build rock5a

package rock5a

import (
	"fmt"
	"log"
	"math"
	"os"
	"strconv"
	"sync"
	"time"
)

type sysfsPWM struct {
	mu       sync.Mutex
	chipPath string
	pwmPath  string
	channel  int
}

var buzzerPWM *sysfsPWM

func initBuzzerPWM() {
	pwm := &sysfsPWM{
		chipPath: PWMChipPath,
		pwmPath:  fmt.Sprintf("%s/pwm%d", PWMChipPath, PWMChannel),
		channel:  PWMChannel,
	}

	if _, err := os.Stat(pwm.pwmPath); os.IsNotExist(err) {
		log.Println("Exporting PWM...")
		if err := writeSysfs(pwm.chipPath+"/export", strconv.Itoa(PWMChannel)); err != nil {
			log.Printf("WARNING: Failed to export PWM (%v). Buzzer disabled.", err)
			return
		}
		time.Sleep(100 * time.Millisecond)
	}

	writeSysfs(pwm.pwmPath+"/duty_cycle", "0")
	writeSysfs(pwm.pwmPath+"/enable", "0")

	buzzerPWM = pwm
	log.Printf("Buzzer PWM initialized: %s", pwm.pwmPath)
}

func writeSysfs(path string, val string) error {
	return os.WriteFile(path, []byte(val), 0644)
}

func (p *sysfsPWM) close() {
	writeSysfs(p.pwmPath+"/enable", "0")
	writeSysfs(p.pwmPath+"/duty_cycle", "0")
	writeSysfs(p.chipPath+"/unexport", strconv.Itoa(p.channel))
}

func RingBuzzer(buzzerTone int, buzzerTime time.Duration, freq int) {
	if buzzerPWM == nil {
		return
	}

	buzzerPWM.mu.Lock()
	defer buzzerPWM.mu.Unlock()

	const baseFrequency = 440.0
	const semitoneRatio = 1.0595

	var frequency float64
	if freq == 0 {
		frequency = baseFrequency * math.Pow(semitoneRatio, float64(buzzerTone))
	} else {
		frequency = float64(freq)
	}

	periodNs := int(1_000_000_000.0 / frequency)
	dutyNs := periodNs / 2

	writeSysfs(buzzerPWM.pwmPath+"/duty_cycle", "0")
	writeSysfs(buzzerPWM.pwmPath+"/period", strconv.Itoa(periodNs))
	writeSysfs(buzzerPWM.pwmPath+"/duty_cycle", strconv.Itoa(dutyNs))
	writeSysfs(buzzerPWM.pwmPath+"/enable", "1")

	time.Sleep(buzzerTime)

	writeSysfs(buzzerPWM.pwmPath+"/enable", "0")
	writeSysfs(buzzerPWM.pwmPath+"/duty_cycle", "0")
}

func ringBuzzerDirect(freq int, duration time.Duration) {
	RingBuzzer(0, duration, freq)
}
