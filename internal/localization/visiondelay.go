package localization

import (
	"math"
	"time"
)

// vision の定数遅延を、車輪を基準にして測る
// (docs/self-localization-research-20260923.md §4.3)。
//
// **計画 §5.3 は「片方向観測だけではクロックオフセットと最小片方向遅延を
// 分離できない」としている。これは正しいが、結論は「測れない」ではない。**
// 車輪という独立した速度源があれば、両者の速度の時間ずれとして直接測れる。
// PoC が実際にこの方法で「車輪を 8-12 ms 古いとすると最も合う」と測っており、
// timesync の写像に残る片道遅延は 0-4 ms しかないことが分かっている。
//
// handoff §5.2 は「速度域で残る誤差は未較正の定数遅延がその全部である」と
// 結論づけた。**その定数がここで埋まる。**
//
// 仕組みは素直で、
//
//  1. 車輪から復元した body 速度の大きさを、車輪の時刻格子に並べる
//  2. vision の位置を差分して速度の大きさを作り、同じ格子へ最近傍で置く
//  3. 2 つの列の相互相関のピーク位置が、そのまま時刻のずれになる
//
// 速度の**大きさ**を使うのは、座標系・スケール・向きの誤差に影響されないため。
// 正規化相関なのでスケールの違いも吸収される。
//
// **加減速が無いと山ができない。** 定速走行や静止では更新されない。
// これは Qin & Shen が時刻オフセットの可観測性について述べていることと同じで、
// 「v = 0 で不可観測、v 一定で位置バイアスと縮退する」性質の現れである。
//
// 単一の goroutine からのみ触ること。
type VisionDelayEstimator struct {
	period time.Duration
	maxLag int

	wheelSpeed []float64
	wheelOK    []bool
	visSpeed   []float64
	visOK      []bool

	origin Stamp
	hasOrg bool
	// filled は格子を何個埋めたか。
	filled int

	prevVision    VisionPose
	hasPrevVision bool

	last    time.Duration
	lastOK  bool
	lastCor float64
}

// VisionDelayOptions は推定器の設定。
type VisionDelayOptions struct {
	// Period は時刻格子の刻み。既定 8 ms (SPI の周期)。
	Period time.Duration
	// Window は相関を取る窓の長さ。既定 4 s。
	Window time.Duration
	// MaxDelay は探索する遅れの絶対値の上限。既定 100 ms。
	MaxDelay time.Duration
	// VisionBaseline は vision を差分するときの最小の基線。既定 25 ms。
	//
	// 連続フレームの差分だと雑音が支配する (116 Hz・0.4 mm で 66 mm/s)。
	// 25 ms なら 23 mm/s まで落ちる。長くすると今度は遅れの分解能が鈍る。
	VisionBaseline time.Duration
}

func (o *VisionDelayOptions) withDefaults() {
	if o.Period <= 0 {
		o.Period = 8 * time.Millisecond
	}
	if o.Window <= 0 {
		o.Window = 4 * time.Second
	}
	if o.MaxDelay <= 0 {
		o.MaxDelay = 100 * time.Millisecond
	}
	if o.VisionBaseline <= 0 {
		o.VisionBaseline = 25 * time.Millisecond
	}
}

// NewVisionDelayEstimator は vision の遅延推定器を作る。
func NewVisionDelayEstimator(opts VisionDelayOptions) *VisionDelayEstimator {
	opts.withDefaults()
	n := int(opts.Window / opts.Period)
	if n < 64 {
		n = 64
	}
	maxLag := int(opts.MaxDelay / opts.Period)
	if maxLag < 1 {
		maxLag = 1
	}
	if maxLag > n/4 {
		maxLag = n / 4
	}
	return &VisionDelayEstimator{
		period:     opts.Period,
		maxLag:     maxLag,
		wheelSpeed: make([]float64, n),
		wheelOK:    make([]bool, n),
		visSpeed:   make([]float64, n),
		visOK:      make([]bool, n),
		last:       0,
	}
}

// visionBaselineFor は差分の基線を返す。
func (v *VisionDelayEstimator) slot(s Stamp) (int, bool) {
	if !v.hasOrg {
		v.origin = s
		v.hasOrg = true
	}
	// **切り捨てではなく四捨五入で格子へ載せる。** 切り捨てにすると、
	// 車輪 (格子にちょうど乗る) に対して vision (乗らない) だけが平均
	// period/2 = 4 ms 早くずれ、それがそのまま遅延の推定値の偏りになる。
	idx := int((s.Sub(v.origin) + v.period/2) / v.period)
	if idx < 0 {
		return 0, false
	}
	if idx >= len(v.wheelSpeed) {
		return idx, false // 窓からあふれた。Reset して測り直す。
	}
	return idx, true
}

// AddWheel は車輪から復元した body 速度を取り込む。
//
// 引数は運動学で復元した速度 (BodyFromWheel の結果) でよい。
// **フィルタの推定値ではなく車輪そのものから作ること。**
// 推定値を使うと、推定に既に入っている vision の遅延が相関に混ざる。
func (v *VisionDelayEstimator) AddWheel(stamp Stamp, vx, vy float64) {
	idx, ok := v.slot(stamp)
	if !ok {
		if idx >= len(v.wheelSpeed) {
			v.Reset()
		}
		return
	}
	v.wheelSpeed[idx] = math.Hypot(vx, vy)
	if !v.wheelOK[idx] {
		v.wheelOK[idx] = true
		v.filled++
	}
}

// AddVision は vision の観測を取り込む。stamp は timesync で写した露光時刻。
func (v *VisionDelayEstimator) AddVision(pose VisionPose, baseline time.Duration) {
	if baseline <= 0 {
		baseline = 25 * time.Millisecond
	}
	if !v.hasPrevVision {
		v.prevVision = pose
		v.hasPrevVision = true
		return
	}
	dt := pose.Stamp.Sub(v.prevVision.Stamp)
	if dt < baseline {
		return
	}
	if dt > 10*baseline {
		// 長い途切れの後。基準点を張り直すだけ。
		v.prevVision = pose
		return
	}
	dx := pose.Pose.X - v.prevVision.Pose.X
	dy := pose.Pose.Y - v.prevVision.Pose.Y
	speed := math.Hypot(dx, dy) / dt.Seconds()
	// 差分は区間の中点の速度を表す。
	mid := v.prevVision.Stamp.Add(dt / 2)
	if idx, ok := v.slot(mid); ok {
		v.visSpeed[idx] = speed
		v.visOK[idx] = true
	} else if idx >= len(v.visSpeed) {
		v.Reset()
	}
	v.prevVision = pose
}

// Reset は窓を空にする。長い途切れや時刻同期の異常のあとに呼ぶ。
func (v *VisionDelayEstimator) Reset() {
	for i := range v.wheelSpeed {
		v.wheelSpeed[i] = 0
		v.wheelOK[i] = false
		v.visSpeed[i] = 0
		v.visOK[i] = false
	}
	v.hasOrg = false
	v.filled = 0
	v.hasPrevVision = false
}

// Estimate は「vision の時刻が車輪の時刻に対してどれだけ遅れているか」を返す。
//
// 正の値は **vision の刻印が実際の露光より遅い** ことを意味し、そのぶんを
// EstimatorOptions.VisionDelayComp に入れればよい。
//
// minPeak は採用する相関係数の下限 (0.7 程度)。加減速が足りないと山が立たない。
func (v *VisionDelayEstimator) Estimate(minPeak float64) (time.Duration, bool) {
	if v.filled < 64 {
		return 0, false
	}
	res := crossCorrelateLag(v.wheelSpeed, v.visSpeed, v.wheelOK, v.visOK, -v.maxLag, v.maxLag)
	if !res.OK || res.Peak < minPeak {
		v.lastOK = false
		return 0, false
	}
	d := time.Duration(res.LagSamples * float64(v.period))
	v.last = d
	v.lastOK = true
	v.lastCor = res.Peak
	return d, true
}

// Last は直近の推定値と相関のピークを返す。
func (v *VisionDelayEstimator) Last() (time.Duration, float64, bool) {
	return v.last, v.lastCor, v.lastOK
}
