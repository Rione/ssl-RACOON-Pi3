package localization

import "time"

// 制御が使う「指令が効く時刻」の推定 (docs/self-localization-research-20260923.md §4.4)。
//
// 推定 -> 指令 -> SPI -> STM -> モータ応答には遅れがある。現在時刻の状態で
// 制御すると位相が遅れて振動する。実機の実測では **指令が効くまで約 90 ms**
// かかり、しかも**電池電圧で変わる** (Trajectory POC Log §5-21: 電池 21-23 V に
// 合わせた先読み 85 ms が、満充電 24 V では効きすぎて 18-20 mm 先走った)。
//
// したがって「固定の先読み量」は実機では成立しない。ここでは
//
//   - PredictAhead(tau)  推定を tau 先へ進める (共分散も進める)
//   - ActuationDelayEstimator  tau を指令と実測の相関からオンラインで測る
//
// の 2 つを提供し、制御側が測った tau を使えるようにする。

// PredictAhead は現在の推定を d だけ未来へ進めたものを返す。
//
// **フィルタの状態は変えない。** 内部で保存・復元する。
// d は EstimatorOptions.MaxPredictAhead で頭打ちにする。超えた場合は
// Health を DEGRADED へ落として呼び出し側に伝える。
//
// 共分散も同じだけ進めるので、**先読みが長いほど不確かさが正しく増える**。
// 位置制御がゲインを落とす判断に使える。
func (e *Estimator) PredictAhead(d time.Duration) Estimate {
	est := e.Current()
	if d <= 0 || !e.started {
		return est
	}
	capped := false
	if d > e.opts.MaxPredictAhead {
		d = e.opts.MaxPredictAhead
		capped = true
	}

	// 状態と共分散だけ退避する。作業領域は使い捨てなので戻さなくてよい。
	savedX := e.f.x
	savedP := e.f.P

	e.f.predict(d.Seconds())

	est.Stamp = e.lastStamp.Add(d)
	est.Pose = e.f.x.Pose()
	est.VelBody = e.f.x.VelBody()
	est.YawRate = e.f.x.Omega
	est.CovPose = e.f.covPose()
	est.Slip = e.f.x.Slip
	if capped && est.Health == HealthOK {
		est.Health = HealthDegraded
	}

	e.f.x = savedX
	e.f.P = savedP
	return est
}

// ActuationDelayEstimator は「速度指令を出してから機体が実際にそう動くまでの
// 遅れ」をオンラインで測る。
//
// 推定器の状態には入れない。時定数 (電池の消耗は分オーダー) が姿勢推定と
// 4-5 桁違うのと、これは制御の都合の量で推定の精度には効かないため
// (計画 §5.3 が時計推定をフィルタから分けたのと同じ理由)。
//
// **実機では約 90 ms で、電池電圧で変わる** (Trajectory POC Log §5-21)。
// 固定値にできないので測り続ける。
//
// 単一の goroutine からのみ触ること。
type ActuationDelayEstimator struct {
	period time.Duration
	maxLag int

	cmd     []float64
	meas    []float64
	ok      []bool
	head    int
	full    bool
	lastLag float64
	lastPk  float64
	valid   bool

	// 相関を取るときに展開する作業領域 (古い順に並べ直したもの)。
	bufA []float64
	bufB []float64
	vA   []bool
	vB   []bool
}

// NewActuationDelayEstimator は遅れの推定器を作る。
// period は呼び出し周期 (SPI なら 8 ms)、window は相関を取る窓の長さ、
// maxDelay は探索する最大の遅れ。
func NewActuationDelayEstimator(period, window, maxDelay time.Duration) *ActuationDelayEstimator {
	if period <= 0 {
		period = 8 * time.Millisecond
	}
	n := int(window / period)
	if n < 64 {
		n = 64
	}
	maxLag := int(maxDelay / period)
	if maxLag < 1 {
		maxLag = 1
	}
	if maxLag > n/4 {
		maxLag = n / 4
	}
	return &ActuationDelayEstimator{
		period: period,
		maxLag: maxLag,
		cmd:    make([]float64, n),
		meas:   make([]float64, n),
		ok:     make([]bool, n),
		bufA:   make([]float64, n),
		bufB:   make([]float64, n),
		vA:     make([]bool, n),
		vB:     make([]bool, n),
	}
}

// Observe は 1 周期ぶんの「出した指令のある成分」と「推定したその成分」を入れる。
//
// 並進の大きさでも 1 軸でもよいが、**指令と実測で同じ量**にすること。
// 加減速のある区間でないと相関に山ができないので、定速走行では更新されない。
func (a *ActuationDelayEstimator) Observe(command, measured float64) {
	a.cmd[a.head] = command
	a.meas[a.head] = measured
	a.ok[a.head] = true
	a.head = (a.head + 1) % len(a.cmd)
	if a.head == 0 {
		a.full = true
	}
}

// Reset は窓を空にする。
func (a *ActuationDelayEstimator) Reset() {
	for i := range a.ok {
		a.ok[i] = false
	}
	a.head = 0
	a.full = false
	a.valid = false
}

// Estimate は遅れの推定値を返す。まだ決まらなければ ok = false。
//
// minPeak は採用する相関係数の下限 (0.5 程度)。
func (a *ActuationDelayEstimator) Estimate(minPeak float64) (delay time.Duration, ok bool) {
	if !a.full {
		return 0, false
	}
	n := len(a.cmd)
	for i := 0; i < n; i++ {
		j := (a.head + i) % n
		a.bufA[i] = a.cmd[j]
		a.bufB[i] = a.meas[j]
		a.vA[i] = a.ok[j]
		a.vB[i] = a.ok[j]
	}
	// 指令が実測より先行する = lag は非負。
	res := crossCorrelateLag(a.bufA, a.bufB, a.vA, a.vB, 0, a.maxLag)
	if !res.OK || res.Peak < minPeak {
		a.valid = false
		return 0, false
	}
	a.lastLag = res.LagSamples
	a.lastPk = res.Peak
	a.valid = true
	return time.Duration(res.LagSamples * float64(a.period)), true
}

// Peak は直近に求めた相関のピーク値を返す。
func (a *ActuationDelayEstimator) Peak() (float64, bool) { return a.lastPk, a.valid }
