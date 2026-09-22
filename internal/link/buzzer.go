package link

import "time"

var ringBuzzerFn func(int, time.Duration, int)

func SetRingBuzzer(fn func(int, time.Duration, int)) {
	ringBuzzerFn = fn
}

func RingBuzzerAsync(tone int, d time.Duration, freq int) {
	if ringBuzzerFn != nil {
		go ringBuzzerFn(tone, d, freq)
	}
}

func RingBuzzerSync(tone int, d time.Duration, freq int) {
	if ringBuzzerFn != nil {
		ringBuzzerFn(tone, d, freq)
	}
}
