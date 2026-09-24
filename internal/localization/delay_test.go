package localization

import (
	"math"
	"math/rand"
	"testing"
	"time"
)

// 指令と実測のずれから、指令が効くまでの遅れを復元できる。
func TestActuationDelayEstimatorRecoversKnownDelay(t *testing.T) {
	const period = 8 * time.Millisecond
	// 合成データの遅れは周期の整数倍にしかならないので、期待値も整数倍で書く。
	for _, lagSamples := range []int{5, 11, 15} {
		want := time.Duration(lagSamples) * period
		est := NewActuationDelayEstimator(period, 4*time.Second, 200*time.Millisecond)
		hist := make([]float64, 0, 1024)
		rng := rand.New(rand.NewSource(5))

		for i := 0; i < 800; i++ {
			tt := float64(i) * period.Seconds()
			// 加減速がないと相関に山ができないので、複数の周波数で振る。
			cmd := 0.8*math.Sin(2*math.Pi*0.8*tt) + 0.4*math.Sin(2*math.Pi*2.1*tt+0.7)
			hist = append(hist, cmd)
			meas := 0.0
			if i >= lagSamples {
				meas = hist[i-lagSamples]
			}
			meas += rng.NormFloat64() * 0.01
			est.Observe(cmd, meas)
		}
		got, ok := est.Estimate(0.5)
		if !ok {
			t.Fatalf("want %v: estimator did not converge", want)
		}
		if diff := got - want; diff > 2*time.Millisecond || diff < -2*time.Millisecond {
			t.Fatalf("delay = %v, want %v (+-2 ms)", got, want)
		}
		peak, _ := est.Peak()
		t.Logf("true %v -> estimated %v (peak %.3f)", want, got, peak)
	}
}

// 加減速がないと遅れは決まらない (定速では不可観測)。
func TestActuationDelayEstimatorRejectsConstantInput(t *testing.T) {
	est := NewActuationDelayEstimator(8*time.Millisecond, 4*time.Second, 200*time.Millisecond)
	rng := rand.New(rand.NewSource(9))
	for i := 0; i < 800; i++ {
		est.Observe(1.0, 1.0+rng.NormFloat64()*0.01)
	}
	if _, ok := est.Estimate(0.5); ok {
		t.Fatal("a constant command should not produce a delay estimate")
	}
}

// **vision の定数遅延は、車輪という独立した速度源があれば測れる。**
//
// 計画 §5.3 は「片方向観測だけでは分離できない」とし、handoff §5.2 は
// 「速度域で残る誤差はその全部である」と結論づけた。その定数がここで埋まる。
func TestVisionDelayEstimatorRecoversTimestampBias(t *testing.T) {
	const wheelPeriod = 8 * time.Millisecond
	const visionPeriod = time.Second / 116

	for _, bias := range []time.Duration{
		0,
		10 * time.Millisecond,
		25 * time.Millisecond,
		-15 * time.Millisecond,
	} {
		est := NewVisionDelayEstimator(VisionDelayOptions{
			Period: wheelPeriod, Window: 5 * time.Second, MaxDelay: 60 * time.Millisecond,
		})
		rng := rand.New(rand.NewSource(17))

		speed := func(tt float64) float64 {
			return 0.9 + 0.8*math.Sin(2*math.Pi*0.6*tt) + 0.3*math.Sin(2*math.Pi*1.7*tt+1.2)
		}
		// 位置は速度の積分 (解析的に出す)。
		pos := func(tt float64) float64 {
			return 0.9*tt -
				0.8/(2*math.Pi*0.6)*(math.Cos(2*math.Pi*0.6*tt)-1) -
				0.3/(2*math.Pi*1.7)*(math.Cos(2*math.Pi*1.7*tt+1.2)-math.Cos(1.2))
		}

		const dur = 4.5
		for i := 0; float64(i)*wheelPeriod.Seconds() < dur; i++ {
			tt := float64(i) * wheelPeriod.Seconds()
			est.AddWheel(Stamp(time.Duration(tt*float64(time.Second))), speed(tt)+rng.NormFloat64()*0.01, 0)
		}
		for i := 0; float64(i)*visionPeriod.Seconds() < dur; i++ {
			tt := float64(i) * visionPeriod.Seconds()
			// vision の刻印は真の露光時刻より bias だけ後ろ (または前)。
			stamp := Stamp(time.Duration(tt*float64(time.Second))) + Stamp(bias)
			est.AddVision(VisionPose{
				Stamp: stamp,
				Pose:  Pose2{X: pos(tt) + rng.NormFloat64()*0.0004, Y: rng.NormFloat64() * 0.0004},
			}, 25*time.Millisecond)
		}

		got, ok := est.Estimate(0.7)
		if !ok {
			t.Fatalf("bias %v: estimator did not converge", bias)
		}
		if diff := got - bias; diff > 3*time.Millisecond || diff < -3*time.Millisecond {
			t.Fatalf("bias %v: estimated %v (+-3 ms required)", bias, got)
		}
		_, peak, _ := est.Last()
		t.Logf("true bias %6v -> estimated %6v (peak %.3f)", bias, got.Round(time.Millisecond), peak)
	}
}

// 静止していると遅延は決まらない。
func TestVisionDelayEstimatorRejectsStationary(t *testing.T) {
	est := NewVisionDelayEstimator(VisionDelayOptions{Window: 4 * time.Second})
	rng := rand.New(rand.NewSource(3))
	for i := 0; i < 600; i++ {
		tt := time.Duration(i) * 8 * time.Millisecond
		est.AddWheel(Stamp(tt), rng.NormFloat64()*0.002, rng.NormFloat64()*0.002)
	}
	for i := 0; i < 500; i++ {
		tt := time.Duration(i) * (time.Second / 116)
		est.AddVision(VisionPose{
			Stamp: Stamp(tt),
			Pose:  Pose2{X: rng.NormFloat64() * 0.0004, Y: rng.NormFloat64() * 0.0004},
		}, 25*time.Millisecond)
	}
	if d, ok := est.Estimate(0.7); ok {
		t.Fatalf("a stationary robot should not produce a delay estimate, got %v", d)
	}
}
