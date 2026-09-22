//go:build pi4

package pi4

import (
	"math"
	"time"

	"github.com/stianeikeland/go-rpio/v4"
)

func RingBuzzer(buzzerTone int, buzzerTime time.Duration, freq int) {
	buzzer := rpio.Pin(PIN_BUZZER)
	buzzer.Mode(rpio.Pwm)

	const pwmMultiplier = 64
	const baseFrequency = 440.0
	const semitoneRatio = 1.0595

	var frequency int
	if freq == 0 {
		frequency = int(baseFrequency*math.Pow(semitoneRatio, float64(buzzerTone))) * pwmMultiplier
	} else {
		frequency = freq * pwmMultiplier
	}

	buzzer.Freq(frequency)
	buzzer.DutyCycle(16, 32)
	time.Sleep(buzzerTime)
	buzzer.DutyCycle(0, 32)
}

func playRebootMelody() {
	buzzer := rpio.Pin(PIN_BUZZER)
	buzzer.Mode(rpio.Pwm)

	notes := []int{1175, 1396, 1760}
	for _, freq := range notes {
		buzzer.Freq(freq * 64)
		buzzer.DutyCycle(16, 32)
		time.Sleep(500 * time.Millisecond)
		buzzer.DutyCycle(0, 32)
	}
}
