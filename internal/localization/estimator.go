package localization

import (
	"math"
	"time"
)

// Estimator は自己位置推定の公開 API。
//
// **単一の goroutine からのみ触ること** (計画 §7.2)。リンク goroutine が
// SPI トランザクション直後に AddWheel を呼び、vision は別 goroutine から
// チャネル経由で渡して同じ goroutine で AddVision を呼ぶ。
//
// このパッケージは time.Now() を呼ばない。時刻はすべて Stamp で受ける。
// 同じ入力からは必ず同じ出力になる (計画 §7.4)。
type Estimator struct {
	f   *eskf
	buf ringBuffer

	started   bool
	lastStamp Stamp

	hasVision  bool
	lastVision Stamp

	stats EstimatorStats
	opts  EstimatorOptions
}

// EstimatorStats は推定器の動作統計。
type EstimatorStats struct {
	// WheelUpdates / VisionUpdates は適用した観測の数。
	WheelUpdates  int64
	VisionUpdates int64
	// VisionTooOld はバッファより古くて捨てた vision 観測の数。
	//
	// 無理に取り込むと共分散が壊れるので捨てる。破棄率はメトリクスに出す
	// (計画 §4.6)。ここが増え続けるなら遅延がバッファ長を超えている。
	VisionTooOld int64
	// VisionFuture は MaxVisionLead を超えて未来の vision 観測を捨てた数。
	//
	// 車輪 1 周期ぶん先行するのは正常なのでここには数えない。
	// ここが増えるのは時刻同期が壊れているとき。
	VisionFuture int64
	// Replays は再フィルタで巻き戻した周期の総数。
	Replays int64
	// Resets は発散を検出して初期化し直した回数。
	Resets int64
	// HuberDownweights は Huber で重みを落とした観測の数。
	HuberDownweights int64
}

// EstimatorOptions は推定器の追加設定。
type EstimatorOptions struct {
	// BufferSize は OOSM のリングバッファの長さ [周期]。
	// 既定 25 (8 ms x 25 = 200 ms、計画 §4.6)。
	BufferSize int
	// VisionTimeout はこれを超えて vision が来なければ DEGRADED にする。
	// 既定 200 ms。**未計測のしきい値**。
	VisionTimeout time.Duration
	// MaxPosStdDev はこれを超えたら DEGRADED にする [m]。既定 0.3。**未計測**。
	MaxPosStdDev float64

	// VisionDelayComp は vision の時刻から引く定数 [s]。
	//
	// timesync が分離できない最小片方向遅延そのもの。**実測値を入れること**。
	// ゼロのままだと、位置誤差が速度に比例して残り、共分散がそれを表現できない。
	// 0 のときは補償しない (P1 の実測が入るまでの既定)。
	VisionDelayComp time.Duration

	// MaxVisionLead はこれを超えて未来の vision 観測を異常として捨てる。
	// 既定 50 ms。車輪 1 周期ぶん先行するのは正常なので、それより十分大きく取る。
	MaxVisionLead time.Duration
}

func (o *EstimatorOptions) withDefaults() {
	if o.BufferSize <= 0 {
		o.BufferSize = 25
	}
	if o.VisionTimeout <= 0 {
		o.VisionTimeout = 200 * time.Millisecond
	}
	if o.MaxPosStdDev <= 0 {
		o.MaxPosStdDev = 0.3
	}
	if o.MaxVisionLead <= 0 {
		o.MaxVisionLead = 50 * time.Millisecond
	}
}

// NewEstimator は推定器を作る。
func NewEstimator(cfg Config, opts EstimatorOptions) (*Estimator, error) {
	opts.withDefaults()
	f, err := newESKF(cfg)
	if err != nil {
		return nil, err
	}
	return &Estimator{f: f, buf: newRingBuffer(opts.BufferSize), opts: opts}, nil
}

// AddWheel は 4 輪の観測を取り込む。
//
// 入力は **SPI フレーム上の並び** (FL, BL, BR, FR)。論理輪番号への並べ替えは
// 設定に従ってここで行う。実機で Pi が受け取るのがスロット順なので、
// 呼び出し側が並べ替えを気にしなくて済むようにしてある。
func (e *Estimator) AddWheel(s WheelSample) UpdateInfo {
	if !e.started {
		// 最初のサンプルは時刻の基準を決めるだけ。dt が無いと予測できない。
		e.started = true
		e.lastStamp = s.Stamp
		e.pushEntry(s.Stamp, 0, e.f.kin.SlotsToLogical(s.Omega), true)
		return UpdateInfo{}
	}
	dt := s.Stamp.Sub(e.lastStamp).Seconds()
	if dt < 0 {
		// 時刻が戻った。SPI の順序が崩れることは無いはずなので、捨てる。
		return UpdateInfo{}
	}
	e.f.predict(dt)

	logical := e.f.kin.SlotsToLogical(s.Omega)
	info := e.f.updateWheels(logical)
	if info.Applied {
		e.stats.WheelUpdates++
		if info.HuberWeight < 1 {
			e.stats.HuberDownweights++
		}
	}
	e.lastStamp = s.Stamp
	e.pushEntry(s.Stamp, dt, logical, true)
	e.guardDivergence()
	return info
}

// AddImu は IMU サンプルを取り込む。
//
// **現状は何もしない。** 計画 §3.5 の通り STM 側に IMU の実装が存在せず、
// 観測が届かない。載った時点でジャイロを予測の入力に、加速度計を速度予測に
// 使うように書き換える (計画 P5)。呼び出し側を後から変えずに済むよう、
// 入り口だけ先に決めてある。
func (e *Estimator) AddImu(ImuSample) {}

// AddVision は vision の絶対姿勢を取り込む。
//
// v.Stamp は timesync でロボットの時間軸へ写した露光時刻でなければならない。
// 到着時刻を渡すと、片道遅延ぶんの系統誤差がそのまま位置に乗る。
func (e *Estimator) AddVision(v VisionPose) UpdateInfo {
	// timesync が原理的に分離できない定数分を引く (計画 §5.3)。
	//
	// 片方向観測だけでは「クロックオフセット」と「最小片方向遅延」を分離できず、
	// 写像された時刻は真の露光時刻より**常に最小片方向遅延ぶん後ろ**になる。
	// これは雑音ではなく系統誤差なので、共分散には一切現れない。放置すると
	// 位置誤差が速度に比例して増え、**NEES が破綻する**。
	// オフラインで実測して定数として与えること (計画 §12-D3)。
	v.Stamp -= Stamp(e.opts.VisionDelayComp)

	if !e.started {
		// まだ車輪が来ていない。位置だけ初期化しておく。
		e.initializeFrom(v)
		return UpdateInfo{}
	}
	if !e.hasVision {
		e.initializeFrom(v)
		e.syncBufferTail()
		return UpdateInfo{Applied: true, Dim: 3}
	}

	if lead := v.Stamp.Sub(e.lastStamp); lead > 0 {
		// **これは異常ではなく通常起きる。** vision は 60 Hz、車輪は 125 Hz
		// なので、直近の車輪サンプルより後の瞬間を写した観測が普通に届く。
		//
		// 現在時刻でそのまま当てると、その差のぶん (最大 1 車輪周期) だけ
		// 位置がずれる。2 m/s なら 16 mm。観測時刻まで予測を進めてから当てる。
		if lead > e.opts.MaxVisionLead {
			// さすがに先すぎる。時刻同期が壊れている。
			e.stats.VisionFuture++
			return UpdateInfo{}
		}
		return e.applyVisionAhead(v, lead.Seconds())
	}
	return e.applyVisionRetro(v)
}

// applyVisionAhead は観測時刻まで予測を進めてから当てる。
func (e *Estimator) applyVisionAhead(v VisionPose, dt float64) UpdateInfo {
	e.f.predict(dt)
	info := e.f.updateVision(v.Pose)
	e.lastStamp = v.Stamp
	// 状態と時刻が食い違わないよう、観測時刻でエントリを積む。
	// 車輪は無いので hasWheels は false。
	e.pushEntry(v.Stamp, dt, [NumWheels]float64{}, false)
	e.afterVision(v, info)
	e.guardDivergence()
	return info
}

// applyVisionRetro は観測時刻まで巻き戻して更新し、現在時刻まで再フィルタする。
func (e *Estimator) applyVisionRetro(v VisionPose) UpdateInfo {
	i := e.buf.findAtOrBefore(v.Stamp)
	if i < 0 {
		// バッファより古い。無理に取り込むと共分散が壊れるので捨てる。
		e.stats.VisionTooOld++
		return UpdateInfo{}
	}

	// バッファの i 番目の事後状態まで巻き戻す。
	entry := e.buf.at(i)
	e.f.x = entry.x
	e.f.P = entry.p

	// 観測時刻ちょうどまで進めてから当てる。
	if gap := v.Stamp.Sub(entry.stamp).Seconds(); gap > 0 {
		e.f.predict(gap)
	}
	info := e.f.updateVision(v.Pose)

	// 現在時刻まで、保存してある車輪観測で再フィルタする。
	prev := v.Stamp
	for j := i + 1; j < e.buf.len(); j++ {
		en := e.buf.at(j)
		if dt := en.stamp.Sub(prev).Seconds(); dt > 0 {
			e.f.predict(dt)
		}
		if en.hasWheels {
			e.f.updateWheels(en.wheels)
		}
		en.x = e.f.x
		en.p = e.f.P
		prev = en.stamp
		e.stats.Replays++
	}

	e.afterVision(v, info)
	e.guardDivergence()
	return info
}

func (e *Estimator) afterVision(v VisionPose, info UpdateInfo) {
	if !info.Applied {
		return
	}
	e.stats.VisionUpdates++
	e.hasVision = true
	if v.Stamp > e.lastVision {
		e.lastVision = v.Stamp
	}
	if info.HuberWeight < 1 {
		e.stats.HuberDownweights++
	}
}

// initializeFrom は最初の vision で位置と姿勢を立ち上げる。
//
// これをやらないと、初期共分散の大きさに応じた巨大な過渡が最初に出る。
func (e *Estimator) initializeFrom(v VisionPose) {
	e.f.setPose(v.Pose)
	n := e.f.cfg.Noise
	e.f.P[idxPx][idxPx] = n.VisionPosNoise * n.VisionPosNoise
	e.f.P[idxPy][idxPy] = n.VisionPosNoise * n.VisionPosNoise
	e.f.P[idxPhi][idxPhi] = n.VisionAngNoise * n.VisionAngNoise
	for i := 0; i < stateDim; i++ {
		for _, j := range []int{idxPx, idxPy, idxPhi} {
			if i != j {
				e.f.P[i][j] = 0
				e.f.P[j][i] = 0
			}
		}
	}
	e.hasVision = true
	if v.Stamp > e.lastVision {
		e.lastVision = v.Stamp
	}
	e.stats.VisionUpdates++
}

// syncBufferTail は現在状態をバッファの末尾へ書き戻す。
// 現在時刻で更新を当てた場合に、次の巻き戻しが古い状態を掴まないようにする。
func (e *Estimator) syncBufferTail() {
	if n := e.buf.len(); n > 0 {
		en := e.buf.at(n - 1)
		en.x = e.f.x
		en.p = e.f.P
	}
}

func (e *Estimator) pushEntry(stamp Stamp, dt float64, wheels [NumWheels]float64, has bool) {
	e.buf.push(bufferEntry{
		stamp:     stamp,
		x:         e.f.x,
		p:         e.f.P,
		dt:        dt,
		wheels:    wheels,
		hasWheels: has,
	})
}

// guardDivergence は NaN や負の分散を検出したら初期化し直す。
//
// 推定が壊れても走行機能は落としてはならない (計画 §9 の検証項目 8)。
// 黙って NaN を出し続けるより、共分散を開いて作り直すほうが復帰できる。
func (e *Estimator) guardDivergence() {
	if e.f.finite() {
		return
	}
	// 直前の健全な姿勢を探す。
	//
	// 現在の公称状態から拾うだけでは足りない。NaN は速度から位置へ 1 周期で
	// 伝播するので、検出した時点では姿勢まで汚染されていることが多い。
	// リングバッファには汚染前の状態が残っているので、そこから復元する。
	// ゼロへ飛ばすと位置制御の P 項に段差が入って指令が跳ねる。
	pose, ok := e.lastFinitePose()

	e.f.x = nominal{}
	e.f.resetCovariance()
	if ok {
		e.f.setPose(pose)
	}
	e.buf.reset()
	e.hasVision = false
	e.stats.Resets++
}

// lastFinitePose はバッファに残っている最も新しい健全な姿勢を返す。
func (e *Estimator) lastFinitePose() (Pose2, bool) {
	if p := e.f.x.Pose(); !hasNaN(p) {
		return p, true
	}
	for i := e.buf.len() - 1; i >= 0; i-- {
		if p := e.buf.at(i).x.Pose(); !hasNaN(p) {
			return p, true
		}
	}
	return Pose2{}, false
}

func hasNaN(p Pose2) bool {
	return p.X != p.X || p.Y != p.Y || p.Theta != p.Theta ||
		math.IsInf(p.X, 0) || math.IsInf(p.Y, 0) || math.IsInf(p.Theta, 0)
}

// Current は現在時刻の推定を返す。
func (e *Estimator) Current() Estimate {
	est := Estimate{
		Stamp:   e.lastStamp,
		Pose:    e.f.x.Pose(),
		VelBody: e.f.x.VelBody(),
		YawRate: e.f.x.Omega,
		CovPose: e.f.covPose(),
		Slip:    e.f.x.Slip,
		Health:  e.health(),
	}
	if e.hasVision {
		est.SinceVision = e.lastStamp.Sub(e.lastVision)
		if est.SinceVision < 0 {
			est.SinceVision = 0
		}
	}
	return est
}

// Stats は動作統計を返す。
func (e *Estimator) Stats() EstimatorStats { return e.stats }

func (e *Estimator) health() Health {
	if !e.f.finite() {
		return HealthInvalid
	}
	if !e.started || !e.hasVision {
		return HealthDegraded
	}
	if e.lastStamp.Sub(e.lastVision) > e.opts.VisionTimeout {
		return HealthDegraded
	}
	maxVar := e.opts.MaxPosStdDev * e.opts.MaxPosStdDev
	if e.f.P[idxPx][idxPx] > maxVar || e.f.P[idxPy][idxPy] > maxVar {
		return HealthDegraded
	}
	return HealthOK
}
