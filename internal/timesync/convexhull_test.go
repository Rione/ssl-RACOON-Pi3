package timesync

import (
	"math"
	"math/rand"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// 合成シナリオ。計画 §9 の検証項目 2 に対応する。
//
//	t_arrive = (1 + alpha)*t_capture + theta + delta,  delta >= 0
//
// delta は「最小片方向遅延 + 非負の裾」で作る。無線の再送はバースト的に
// 数十 ms 伸びるので、裾は指数分布に時折の大きな尖りを足したものにする。
type scenario struct {
	skewPpm     float64
	offset      time.Duration // theta
	minDelay    time.Duration
	jitterMean  time.Duration
	burstProb   float64
	burstDelay  time.Duration
	rate        time.Duration // パケット間隔
	tCaptureAt0 time.Duration // vision PC の起動からの経過 (大きな値にする)
}

func defaultScenario() scenario {
	return scenario{
		skewPpm:    50,
		offset:     3 * time.Millisecond,
		minDelay:   2 * time.Millisecond,
		jitterMean: 1500 * time.Microsecond,
		burstProb:  0.05,
		burstDelay: 40 * time.Millisecond,
		rate:       time.Second / 60,
		// SSL-Vision の t_capture は PC 起動からの秒。大きな値でも
		// 数値精度が落ちないことを一緒に確かめる。
		tCaptureAt0: 36 * time.Hour,
	}
}

// sample は i 番目のパケットの (t_capture, 真の到着時刻) を返す。
func (s scenario) sample(i int, rng *rand.Rand) (remote, arrival localization.Stamp, delay time.Duration) {
	capture := s.tCaptureAt0 + time.Duration(i)*s.rate
	delay = s.minDelay + time.Duration(rng.ExpFloat64()*float64(s.jitterMean))
	if rng.Float64() < s.burstProb {
		delay += time.Duration(rng.Float64() * float64(s.burstDelay))
	}
	alpha := s.skewPpm * 1e-6
	arriveNs := (1+alpha)*float64(capture) + float64(s.offset) + float64(delay)
	return localization.Stamp(capture), localization.Stamp(arriveNs), delay
}

// trueLocal は「遅延がゼロだったときの到着時刻」= 写像の真値。
func (s scenario) trueLocal(remote localization.Stamp) localization.Stamp {
	alpha := s.skewPpm * 1e-6
	return localization.Stamp((1+alpha)*float64(remote) + float64(s.offset))
}

func feed(c *ConvexHull, s scenario, n int, seed int64) {
	rng := rand.New(rand.NewSource(seed))
	for i := 0; i < n; i++ {
		remote, arrival, _ := s.sample(i, rng)
		c.Observe(remote, arrival)
	}
}

// 計画 §10-10: スキュー残差 <= 10 ppm。
func TestConvexHullRecoversSkew(t *testing.T) {
	for _, ppm := range []float64{0, 50, -50, 120, -120} {
		s := defaultScenario()
		s.skewPpm = ppm

		c := NewConvexHull(Config{})
		feed(c, s, 60*30, 1) // 30 秒ぶん

		st := c.Stats()
		if !st.Valid {
			t.Fatalf("ppm=%v: estimate never became valid (%+v)", ppm, st)
		}
		if d := math.Abs(st.SkewPpm - ppm); d > 10 {
			t.Errorf("ppm=%v: recovered %.2f ppm (off by %.2f, want <= 10)", ppm, st.SkewPpm, d)
		}
	}
}

// 計画 §10-10: オフセット変動 <= 2 ms。
//
// 片方向観測だけでは「クロックオフセット」と「最小片方向遅延」は分離できないので、
// 検証するのは theta + minDelay に対する残差。
func TestConvexHullMappingAccuracy(t *testing.T) {
	s := defaultScenario()
	c := NewConvexHull(Config{})
	feed(c, s, 60*30, 2)

	rng := rand.New(rand.NewSource(99))
	var maxErr, sumSq float64
	n := 0
	for i := 60 * 30; i < 60*40; i++ {
		remote, arrival, _ := s.sample(i, rng)
		c.Observe(remote, arrival)
		mapped, q := c.ToLocal(remote, arrival)
		if !q.Valid {
			t.Fatal("estimate went invalid mid-run")
		}
		// 分離できない最小片方向遅延ぶんだけ系統的にずれる。それを引いて評価する。
		want := s.trueLocal(remote) + localization.Stamp(s.minDelay)
		e := math.Abs(float64(mapped - want))
		maxErr = math.Max(maxErr, e)
		sumSq += e * e
		n++
	}
	rms := math.Sqrt(sumSq / float64(n))
	if rms > float64(2*time.Millisecond) {
		t.Errorf("mapping rms error = %v, want <= 2ms", time.Duration(rms))
	}
	t.Logf("mapping error: rms %v, max %v", time.Duration(rms), time.Duration(maxErr))
}

// これが凸包法を選んだ理由そのもの。バースト遅延の下で、線形回帰より
// 明確に良いことを固定する (計画 §13「凸包法 (EMA を使わない)」)。
func TestConvexHullBeatsLeastSquaresUnderBursts(t *testing.T) {
	s := defaultScenario()
	// 再送が頻発する状況。裾が重い。
	s.burstProb = 0.25
	s.burstDelay = 80 * time.Millisecond

	c := NewConvexHull(Config{})
	rng := rand.New(rand.NewSource(7))

	var sx, sy, sxx, sxy float64
	var x0, y0 float64
	n := 0
	for i := 0; i < 60*30; i++ {
		remote, arrival, _ := s.sample(i, rng)
		c.Observe(remote, arrival)

		// 同じ点群に素朴な最小二乗を当てる。
		x := float64(remote)
		y := float64(arrival - remote)
		if n == 0 {
			x0, y0 = x, y
		}
		px, py := x-x0, y-y0
		sx += px
		sy += py
		sxx += px * px
		sxy += px * py
		n++
	}
	den := float64(n)*sxx - sx*sx
	lsSkewPpm := (float64(n)*sxy - sx*sy) / den * 1e6

	hullErr := math.Abs(c.Stats().SkewPpm - s.skewPpm)
	lsErr := math.Abs(lsSkewPpm - s.skewPpm)

	t.Logf("skew error: convex hull %.2f ppm, least squares %.2f ppm", hullErr, lsErr)
	if hullErr > 10 {
		t.Errorf("convex hull skew error %.2f ppm exceeds the 10 ppm target", hullErr)
	}
	// 凸包は下端しか見ないのでバーストに動かされない。回帰は平均に引かれる。
	if hullErr > lsErr {
		t.Errorf("convex hull (%.2f ppm) did no better than least squares (%.2f ppm); "+
			"the whole reason for choosing it is robustness to the heavy upper tail", hullErr, lsErr)
	}
}

// AP ローミング相当。最小片方向遅延が急に縮んだら窓を捨てて組み直すこと。
func TestConvexHullDetectsPathChange(t *testing.T) {
	s := defaultScenario()
	c := NewConvexHull(Config{})
	rng := rand.New(rand.NewSource(11))

	for i := 0; i < 60*30; i++ {
		remote, arrival, _ := s.sample(i, rng)
		c.Observe(remote, arrival)
	}
	if !c.Stats().Valid {
		t.Fatal("estimate never became valid")
	}
	before := c.Stats()

	// 経路が変わって 20 ms 速くなった。
	faster := s
	faster.minDelay = s.minDelay - 20*time.Millisecond
	for i := 60 * 30; i < 60*31; i++ {
		remote, arrival, _ := faster.sample(i, rng)
		c.Observe(remote, arrival)
	}
	after := c.Stats()
	if after.Resets <= before.Resets {
		t.Errorf("the path change was not detected (resets %d -> %d)", before.Resets, after.Resets)
	}

	// 組み直した後、新しい経路でちゃんと収束すること。
	for i := 60 * 31; i < 60*70; i++ {
		remote, arrival, _ := faster.sample(i, rng)
		c.Observe(remote, arrival)
	}
	st := c.Stats()
	if !st.Valid {
		t.Fatalf("estimate did not recover after the path change (%+v)", st)
	}
	if d := math.Abs(st.SkewPpm - s.skewPpm); d > 10 {
		t.Errorf("after recovery skew = %.2f ppm, want %.2f (off by %.2f)", st.SkewPpm, s.skewPpm, d)
	}
}

// スキューが上限を超えたら凍結して直前の値を保持すること (計画 §5.5)。
func TestConvexHullFreezesOnRunawaySkew(t *testing.T) {
	s := defaultScenario()
	s.skewPpm = 20
	c := NewConvexHull(Config{MaxSkewPpm: 50})
	feed(c, s, 60*30, 3)

	good := c.Stats()
	if !good.Valid || good.Frozen {
		t.Fatalf("expected a healthy estimate first, got %+v", good)
	}

	// 経路変更の検出に引っかからないよう、遅延が伸びる向きに壊す。
	// 到着だけがどんどん遅れていくとスキューが巨大に見える。
	rng := rand.New(rand.NewSource(4))
	drifted := func(i int) (localization.Stamp, localization.Stamp) {
		remote, arrival, _ := s.sample(i, rng)
		drift := time.Duration(float64(i-60*30) * float64(s.rate) * 500e-6) // 500 ppm 相当
		return remote, arrival + localization.Stamp(drift)
	}

	// 閾値を跨いで凍結するまで進める。
	var frozenAt Stats
	for i := 60 * 30; i < 60*90; i++ {
		c.Observe(drifted(i))
		if st := c.Stats(); st.Frozen && frozenAt.Fits == 0 {
			frozenAt = st
		}
	}

	st := c.Stats()
	if st.Freezes == 0 || !st.Frozen {
		t.Fatalf("a 500 ppm drift did not trip the 50 ppm guard (%+v)", st)
	}
	// 凍結中も直前の値を保持していること。推定を落とすのではなく警報する。
	if !st.Valid {
		t.Error("a frozen estimate must stay usable; it holds the last good fit")
	}
	// 肝心なのは「閾値を超えた値を決して公表しない」こと。保持されるのは
	// 初期値ではなく、閾値を跨ぐ直前に受理された最後の値である。
	if math.Abs(st.SkewPpm) > 50 {
		t.Errorf("frozen skew = %.2f ppm, which is beyond the %v ppm guard", st.SkewPpm, 50.0)
	}
	if st.SkewPpm < good.SkewPpm {
		t.Errorf("frozen skew %.2f ppm is below the healthy value %.2f; "+
			"the held value should be the last accepted fit on the way up", st.SkewPpm, good.SkewPpm)
	}
	// 凍結後はドリフトが続いても値が動かないこと。
	if frozenAt.Fits != 0 && math.Abs(st.SkewPpm-frozenAt.SkewPpm) > 1e-9 {
		t.Errorf("skew moved after freezing: %.4f -> %.4f ppm", frozenAt.SkewPpm, st.SkewPpm)
	}
}

// 観測が足りない・広がりが無いうちは推定を出さないこと。
// ここを緩めると、短い窓の雑音を ppm オーダーのスキューと誤認する。
func TestConvexHullWithholdsEstimateUntilItCan(t *testing.T) {
	s := defaultScenario()
	c := NewConvexHull(Config{MinSamples: 20, MinSpan: 5 * time.Second})
	rng := rand.New(rand.NewSource(5))

	for i := 0; i < 10; i++ { // 観測が足りない
		remote, arrival, _ := s.sample(i, rng)
		c.Observe(remote, arrival)
		_, q := c.ToLocal(remote, arrival)
		if q.Valid {
			t.Fatalf("estimate became valid after only %d samples", i+1)
		}
	}
	// 60 Hz なので 20 サンプルでは 0.33 秒しか広がらない。まだ出してはいけない。
	for i := 10; i < 60; i++ {
		remote, arrival, _ := s.sample(i, rng)
		c.Observe(remote, arrival)
	}
	if c.Stats().Valid {
		t.Error("estimate became valid with only 1 second of span; MinSpan is not being enforced")
	}

	for i := 60; i < 60*8; i++ {
		remote, arrival, _ := s.sample(i, rng)
		c.Observe(remote, arrival)
	}
	if !c.Stats().Valid {
		t.Errorf("estimate still not valid after 8 seconds (%+v)", c.Stats())
	}
}

// 推定が立つ前は到着時刻へフォールバックし、Valid = false と正直に言うこと。
func TestConvexHullFallsBackToArrival(t *testing.T) {
	c := NewConvexHull(Config{})
	remote := localization.Stamp(36 * time.Hour)
	arrival := localization.Stamp(12345)

	c.Observe(remote, arrival)
	mapped, q := c.ToLocal(remote, arrival)
	if q.Valid {
		t.Error("a single observation must not produce a valid estimate")
	}
	if mapped != arrival {
		t.Errorf("fallback mapped to %v, want the arrival stamp %v", mapped, arrival)
	}
}

// 窓は古い観測を落とすこと。落とさないとメモリが伸び続け、
// 経路変更への追従も鈍る。
func TestConvexHullEvictsOldSamples(t *testing.T) {
	s := defaultScenario()
	c := NewConvexHull(Config{Window: 10 * time.Second})
	feed(c, s, 60*60, 6) // 60 秒ぶん投入

	st := c.Stats()
	if st.Samples > 60*11 {
		t.Errorf("window holds %d samples; a 10 s window at 60 Hz should hold about 600", st.Samples)
	}
	if st.SpanNs > int64(11*time.Second) {
		t.Errorf("window span = %v, want <= 10 s", time.Duration(st.SpanNs))
	}
}

// 順序逆転したパケットが来ても凸包が壊れないこと。
func TestConvexHullToleratesReordering(t *testing.T) {
	s := defaultScenario()
	c := NewConvexHull(Config{})
	rng := rand.New(rand.NewSource(8))

	for i := 0; i < 60*30; i++ {
		j := i
		if i%17 == 0 && i > 0 {
			j = i - 1 // 1 つ前のフレームが後から届く
		}
		remote, arrival, _ := s.sample(j, rng)
		c.Observe(remote, arrival)
	}
	st := c.Stats()
	if !st.Valid {
		t.Fatalf("reordering broke the estimate (%+v)", st)
	}
	if d := math.Abs(st.SkewPpm - s.skewPpm); d > 10 {
		t.Errorf("skew = %.2f ppm with reordering, want %.2f", st.SkewPpm, s.skewPpm)
	}
}

// 下側凸包の当てはめが「すべての点の下を通る」ことを直接確かめる。
func TestFitLowerHullStaysBelowEveryPoint(t *testing.T) {
	rng := rand.New(rand.NewSource(21))
	for trial := 0; trial < 200; trial++ {
		n := 20 + rng.Intn(200)
		pts := make([]point, n)
		for i := range pts {
			x := float64(i) * 1e7
			pts[i] = point{x: x, y: 5e-5*x + 1e6 + rng.ExpFloat64()*2e6}
		}
		skew, offset, ok := fitLowerHull(pts)
		if !ok {
			t.Fatalf("trial %d: fit failed", trial)
		}
		for i, p := range pts {
			if resid := p.y - (skew*p.x + offset); resid < -1e-6 {
				t.Fatalf("trial %d: point %d sits %v below the fitted line", trial, i, resid)
			}
		}
	}
}

func TestConvexHullResetClearsState(t *testing.T) {
	s := defaultScenario()
	c := NewConvexHull(Config{})
	feed(c, s, 60*30, 9)
	if !c.Stats().Valid {
		t.Fatal("estimate never became valid")
	}
	c.Reset()
	st := c.Stats()
	if st.Valid || st.Samples != 0 {
		t.Errorf("Reset left state behind: %+v", st)
	}
}

// timesync は time.Now() を呼ばない。同じ入力から必ず同じ出力になること
// (計画 §7.4)。オフラインチューニングと回帰テストの前提。
func TestConvexHullIsDeterministic(t *testing.T) {
	s := defaultScenario()
	run := func() Stats {
		c := NewConvexHull(Config{})
		feed(c, s, 60*30, 12)
		return c.Stats()
	}
	a, b := run(), run()
	if a != b {
		t.Errorf("two identical runs disagreed:\n  %+v\n  %+v", a, b)
	}
}
