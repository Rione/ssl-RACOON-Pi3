package timesync

import (
	"math"
	"sort"
	"sync"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// 下側凸包によるクロックオフセット・スキュー推定 (計画 §5.3)。
//
//	t_arrive = (1 + alpha)*t_capture + theta + delta,   delta >= 0
//
//	alpha … スキュー   theta … オフセット   delta … 片方向遅延
//
// delta は必ず非負で、無線の再送によりバースト的に伸びる。
// **分布は下側に硬い壁があり上側に長い裾を持つので、平均ではなく最小値が真値に近い。**
// 点群 (t_capture, t_arrive − t_capture) の下側凸包の傾きがスキュー、切片がオフセットになる。
// 線形回帰や EMA と違い、遅い配送の影響を原理的に受けない。
//
// 原理的な限界: 片方向観測だけでは「クロックオフセット」と「最小片方向遅延」は分離できない。
// 分離できるのはスキューと遅延の変動分で、残る定数分はオフラインで実測して定数として持つ。

// Config は凸包推定の設定。
type Config struct {
	// Window は凸包に使う観測の時間窓。既定 30 秒。
	//
	// 長いほどスキューの推定が安定するが、経路が変わったときの追従が遅れる。
	Window time.Duration

	// RefitInterval は凸包を組み直す間隔。既定 500 ms。
	//
	// クロックのドリフトは分オーダーなので毎パケット組み直す必要はない。
	// 経過は到着 Stamp で測るので、time.Now() は呼ばない (計画 §7.4)。
	RefitInterval time.Duration

	// MinSamples はこれ未満なら推定を出さない。既定 20。
	MinSamples int

	// MinSpan は観測の時間的な広がりの下限。既定 5 秒。
	//
	// スキューは「時間が経つほど開く差」なので、広がりが無いと傾きが決まらない。
	// ここを緩めると、短い窓の雑音を ppm オーダーのスキューと誤認する。
	MinSpan time.Duration

	// MaxSkewPpm を超えたら推定を凍結して直前の値を保持する。既定 200 ppm。
	//
	// 水晶の確度は ±20〜100 ppm。NTP のスルーイングを足しても ±500 ppm には届かない。
	// これを超えるのは推定の暴走である (計画 §5.5)。
	MaxSkewPpm float64

	// PathChangeNs は「最小片方向遅延が急に縮んだ」と判定する閾値。既定 5 ms。
	//
	// AP ローミングや経路変更で遅延が跳ぶと、古い凸包が現在の下端より上に残り、
	// 以降の写像が全部ずれる。下回った量がこれを超えたら窓を捨てる。
	PathChangeNs int64
}

func (c *Config) withDefaults() {
	if c.Window <= 0 {
		c.Window = 30 * time.Second
	}
	if c.RefitInterval <= 0 {
		c.RefitInterval = 500 * time.Millisecond
	}
	if c.MinSamples <= 0 {
		c.MinSamples = 20
	}
	if c.MinSpan <= 0 {
		c.MinSpan = 5 * time.Second
	}
	if c.MaxSkewPpm <= 0 {
		c.MaxSkewPpm = 200
	}
	if c.PathChangeNs <= 0 {
		c.PathChangeNs = int64(5 * time.Millisecond)
	}
}

// Stats は推定の状態。監視に使う (計画 §5.5)。
type Stats struct {
	// Samples は窓に入っている観測数。
	Samples int
	// SpanNs は窓に入っている観測の時間的な広がり。
	SpanNs int64
	// Fits は凸包を組み直した回数。
	Fits int64
	// Resets は経路変更を検出して窓を捨てた回数。
	Resets int64
	// Freezes はスキューが上限を超えて凍結した回数。
	Freezes int64
	// SkewPpm / OffsetNs は現在の推定値。
	SkewPpm  float64
	OffsetNs int64
	// Valid は推定が使える状態か。
	Valid bool
	// Frozen は暴走を検出して直前の値を保持しているか。
	Frozen bool
}

type point struct{ x, y float64 }

// ConvexHull は下側凸包によるクロック推定。単一のカメラ (単一の送信クロック) 用。
//
// 複数カメラは MultiSync で束ねる。カメラごとに露光から送信までの処理遅延が
// 違うので、混ぜると凸包の下端が一番速いカメラに引きずられる (計画 §5.3)。
type ConvexHull struct {
	cfg Config

	mu  sync.Mutex
	obs []point // 到着順 = x の昇順 (順序逆転があれば fit 時に整列する)

	// epoch は数値精度のための基準点。x も y もナノ秒で 1e14 になり得るので、
	// 差分に直してから当てはめる。Reset するまで動かさない。
	epoch    point
	hasEpoch bool

	lastFit localization.Stamp
	hasFit  bool

	skew     float64 // alpha (無次元)
	offset   float64 // theta [ns]、epoch 基準
	valid    bool
	frozen   bool
	unsorted bool

	fits    int64
	resets  int64
	freezes int64
}

// NewConvexHull は凸包推定器を作る。
func NewConvexHull(cfg Config) *ConvexHull {
	cfg.withDefaults()
	return &ConvexHull{cfg: cfg}
}

// Observe は 1 つの観測を取り込む。
func (c *ConvexHull) Observe(remote, arrival localization.Stamp) {
	c.mu.Lock()
	defer c.mu.Unlock()

	x := float64(remote)
	y := float64(arrival - remote)

	if !c.hasEpoch {
		c.epoch = point{x: x, y: y}
		c.hasEpoch = true
		c.lastFit = arrival
	}
	p := point{x: x - c.epoch.x, y: y - c.epoch.y}

	// 経路が変わって遅延が縮むと、古い凸包が現在の下端より上に取り残される。
	// 以降の写像が丸ごとずれるので、窓を捨てて組み直す。
	if c.valid {
		if below := (c.skew*p.x + c.offset) - p.y; below > float64(c.cfg.PathChangeNs) {
			c.resetLocked()
			c.epoch = point{x: x, y: y}
			c.hasEpoch = true
			c.lastFit = arrival
			c.resets++
			p = point{}
		}
	}

	if n := len(c.obs); n > 0 && p.x < c.obs[n-1].x {
		c.unsorted = true // 順序逆転。fit 時に整列する
	}
	c.obs = append(c.obs, p)
	c.evictLocked()

	if arrival-c.lastFit >= localization.Stamp(c.cfg.RefitInterval) || !c.hasFit {
		c.fitLocked()
		c.lastFit = arrival
	}
}

// ToLocal は remote 時刻を Rock5A の時間軸へ写す。
func (c *ConvexHull) ToLocal(remote, arrival localization.Stamp) (localization.Stamp, Quality) {
	c.mu.Lock()
	defer c.mu.Unlock()

	if !c.valid {
		// 推定が立っていない間は到着時刻で代用する。上位は Valid = false で
		// 「写像できていない」と判別でき、vision_meta に正直に出る。
		return arrival, Quality{Valid: false, Samples: len(c.obs)}
	}

	px := float64(remote) - c.epoch.x
	// y = alpha*x + theta は「到着 − 送信」なので、送信時刻に足すと
	// ローカル時間軸上の露光時刻になる。
	delta := c.skew*px + c.offset + c.epoch.y
	mapped := localization.Stamp(float64(remote) + delta)

	return mapped, Quality{
		Valid:   true,
		Frozen:  c.frozen,
		SkewPpm: c.skew * 1e6,
		// この観測を写すのに足した量。すなわち現時点の
		// 「クロックオフセット + 最小片方向遅延」である。
		//
		// 絶対時刻ゼロ点での切片は報告しない。t_capture は vision PC の
		// 起動からの秒なので、切片はスキューに掛かって数秒規模の値になり、
		// 物理的な意味を持たない。急なジャンプを監視したいのはこちらのほう
		// (計画 §5.5 の「最小片方向遅延が急にジャンプ」)。
		OffsetNs: int64(delta),
		Samples:  len(c.obs),
	}
}

// Reset は推定状態を捨てる。
func (c *ConvexHull) Reset() {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.resetLocked()
}

func (c *ConvexHull) resetLocked() {
	c.obs = c.obs[:0]
	c.hasEpoch = false
	c.hasFit = false
	c.valid = false
	c.frozen = false
	c.unsorted = false
	c.skew = 0
	c.offset = 0
}

// Stats は推定の状態を返す。
func (c *ConvexHull) Stats() Stats {
	c.mu.Lock()
	defer c.mu.Unlock()
	s := Stats{
		Samples: len(c.obs),
		Fits:    c.fits,
		Resets:  c.resets,
		Freezes: c.freezes,
		SkewPpm: c.skew * 1e6,
		Valid:   c.valid,
		Frozen:  c.frozen,
	}
	if n := len(c.obs); n > 0 {
		// 直近の観測時点での「オフセット + 最小片方向遅延」。
		s.OffsetNs = int64(c.skew*c.obs[n-1].x + c.offset + c.epoch.y)
		if n > 1 {
			s.SpanNs = int64(c.obs[n-1].x - c.obs[0].x)
		}
	}
	return s
}

func (c *ConvexHull) evictLocked() {
	if len(c.obs) == 0 {
		return
	}
	cutoff := c.obs[len(c.obs)-1].x - float64(c.cfg.Window)
	i := 0
	for i < len(c.obs) && c.obs[i].x < cutoff {
		i++
	}
	if i > 0 {
		c.obs = append(c.obs[:0], c.obs[i:]...)
	}
}

func (c *ConvexHull) fitLocked() {
	c.fits++

	if len(c.obs) < c.cfg.MinSamples {
		c.valid = c.hasFit && c.valid // 一度立った推定は保持する
		return
	}
	if c.unsorted {
		sort.Slice(c.obs, func(i, j int) bool {
			if c.obs[i].x != c.obs[j].x {
				return c.obs[i].x < c.obs[j].x
			}
			return c.obs[i].y < c.obs[j].y
		})
		c.unsorted = false
	}
	span := c.obs[len(c.obs)-1].x - c.obs[0].x
	if span < float64(c.cfg.MinSpan) {
		// 広がりが無いと傾きが決まらない。雑音を ppm と読み違えない。
		return
	}

	skew, offset, ok := fitLowerHull(c.obs)
	if !ok {
		return
	}

	if math.Abs(skew)*1e6 > c.cfg.MaxSkewPpm {
		// 暴走。直前の値を保持して警報する (計画 §5.5)。
		c.freezes++
		c.frozen = true
		return
	}

	c.skew = skew
	c.offset = offset
	c.valid = true
	c.frozen = false
	c.hasFit = true
}

// fitLowerHull は「すべての点の下を通る直線のうち、重心の x で最も高いもの」を返す。
//
// 目的は sum_i (y_i − (a·x_i + b)) の最小化、制約は a·x_i + b <= y_i。
// 目的関数は a·sum(x) + n·b なので、重心 x での直線の値を最大化することと等価になる。
// その最適解は**重心の x をまたぐ下側凸包の辺**そのものである。
// LP を解かずに済み、O(n log n)、数値的にも安定する。
func fitLowerHull(pts []point) (skew, offset float64, ok bool) {
	hull := lowerHull(pts)
	if len(hull) < 2 {
		return 0, 0, false
	}

	var sum float64
	for _, p := range pts {
		sum += p.x
	}
	mean := sum / float64(len(pts))

	for i := 0; i+1 < len(hull); i++ {
		a, b := hull[i], hull[i+1]
		if mean < a.x || mean > b.x {
			continue
		}
		dx := b.x - a.x
		if dx <= 0 {
			continue
		}
		skew = (b.y - a.y) / dx
		return skew, a.y - skew*a.x, true
	}
	return 0, 0, false
}

// lowerHull は x 昇順に整列済みの点群の下側凸包を返す (Andrew の monotone chain)。
func lowerHull(pts []point) []point {
	hull := make([]point, 0, len(pts))
	for _, p := range pts {
		for len(hull) >= 2 && cross(hull[len(hull)-2], hull[len(hull)-1], p) <= 0 {
			hull = hull[:len(hull)-1]
		}
		hull = append(hull, p)
	}
	return hull
}

func cross(o, a, b point) float64 {
	return (a.x-o.x)*(b.y-o.y) - (a.y-o.y)*(b.x-o.x)
}
