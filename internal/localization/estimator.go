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

	// 冗長残差によるスリップ監視 (redundancy.go)。推定に依存しない検査。
	red  *Redundancy
	slip *SlipMonitor

	// vision の見かけの速度 [m/s]。ZUPT の判定に使う。
	visionSpeed   float64
	prevVision    VisionPose
	hasPrevVision bool
	// zuptStreak は停止条件が連続して成立した周期数。
	zuptStreak int

	stats EstimatorStats
	opts  EstimatorOptions
}

// EstimatorStats は推定器の動作統計。
type EstimatorStats struct {
	// WheelUpdates / VisionUpdates は適用した観測の数。
	WheelUpdates  int64
	VisionUpdates int64
	// WheelTooOld はバッファより古くて捨てた車輪観測の数。
	//
	// **ここが増えるのは異常。** 車輪と vision の時刻の関係が壊れている
	// (VisionDelayComp が大きすぎる、車輪の刻印が遅れている) ことを示す。
	WheelTooOld int64
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
	// ZuptUpdates は停止の疑似観測を当てた回数。
	ZuptUpdates int64
	// GyroUpdates はジャイロで更新した回数。IMU が無い機体では 0 のまま。
	GyroUpdates int64
	// Collisions は加速度計が衝突を検出した回数。
	Collisions int64
	// SlipDetections は冗長残差がスリップと判定した周期の数。
	//
	// **これが全体の半分を超え続けるなら、滑っているのではなく幾何が間違っている。**
	// 残差の雑音は平均ゼロなので、いつも閾値を超えるのは運動学の設定が
	// 実機と合っていないときだけである (研究 §2.4)。
	SlipDetections int64
	// ParamFrozenCycles は機体パラメータの補正を凍結した周期の数。
	ParamFrozenCycles int64
	// HuberDownweights は Huber で重みを落とした観測の数。
	HuberDownweights int64

	// VisionNISSum / VisionNISCount は vision のイノベーションの正規化二乗和。
	//
	// **真値が要らない唯一の整合性の指標** (Bar-Shalom の NIS)。
	// 平均が観測の次元 (vision なら 3) になっていれば、Q と R が実機と合っている。
	// 3 より大きければ Q か R が小さすぎ、小さければ大きすぎる。
	//
	// NEES は真値が要るので実機では疑似真値に頼るしかなく、その疑似真値は
	// 同じモデルで作るため循環する。**実機で Q を決めるのはこちらを使うこと。**
	VisionNISSum   float64
	VisionNISCount int64
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

	// ParamFreezeAfter は vision が途切れてから機体パラメータの補正を止めるまで。
	// 既定 150 ms。
	//
	// 車輪観測だけでは速度と倍率が積としてしか現れず不可観測になるので、
	// そこで補正を許すと公称値が不可観測な尾根に沿って流れる
	// (Mozzarelli ほか arXiv:2403.13452)。共分散は育て続ける。
	ParamFreezeAfter time.Duration

	// MaxPredictAhead は PredictAhead のホライズンの上限。既定 150 ms。
	//
	// 実機の「指令が効くまでの遅れ」は約 90 ms で、電池電圧で変わる
	// (Trajectory POC Log §5-21)。上限はその 1.5 倍程度に取る。
	MaxPredictAhead time.Duration

	// ZuptMinCycles は停止条件が何周期連続したら ZUPT を当てるか。既定 5 (40 ms)。
	ZuptMinCycles int
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
	if o.ParamFreezeAfter <= 0 {
		o.ParamFreezeAfter = 150 * time.Millisecond
	}
	if o.MaxPredictAhead <= 0 {
		o.MaxPredictAhead = 150 * time.Millisecond
	}
	if o.ZuptMinCycles <= 0 {
		o.ZuptMinCycles = 5
	}
}

// NewEstimator は推定器を作る。
func NewEstimator(cfg Config, opts EstimatorOptions) (*Estimator, error) {
	opts.withDefaults()
	f, err := newESKF(cfg)
	if err != nil {
		return nil, err
	}
	red, err := NewRedundancy(f.kin)
	if err != nil {
		return nil, err
	}
	e := &Estimator{f: f, buf: newRingBuffer(opts.BufferSize), opts: opts, red: red}
	// しきい値は速度に依存する車輪雑音まで含めて計算する。定数の sigma で
	// 割ると、速く回っているときに「いつも滑っている」と誤判定する。
	e.slip = NewSlipMonitor(red, cfg.Noise.WheelNoise, cfg.Noise.WheelNoiseSpeedCoef, 0)
	return e, nil
}

// Redundancy は冗長残差の計算器を返す。診断と較正で使う。
func (e *Estimator) Redundancy() *Redundancy { return e.red }

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
		e.pushEntry(s.Stamp, 0, e.f.kin.SlotsToLogical(s.Omega), true, false)
		return UpdateInfo{}
	}
	logical := e.f.kin.SlotsToLogical(s.Omega)

	dt := s.Stamp.Sub(e.lastStamp).Seconds()
	if dt < 0 {
		// **時刻が戻った。捨てない。**
		//
		// SPI の順序が崩れることは無いが、vision の観測時刻が車輪より先に
		// なることはある (VisionDelayComp が負のとき、あるいは車輪の刻印が
		// 遅れているとき)。そのとき applyVisionAhead が lastStamp を未来へ
		// 進めるので、次の車輪が「過去」になる。
		//
		// 以前はここで黙って捨てていた。実機のログを流したら**車輪の更新が
		// 710 回中 0 回**になっていて気づいた。vision の観測だけで回るので
		// 一見動いてしまうのが質が悪い。
		return e.applyWheelRetro(s.Stamp, logical)
	}
	e.f.predict(dt)

	// **推定に依存しないスリップ検査。** 冗長残差 n^T w は滑りが無ければ
	// 厳密にゼロなので、速度推定も vision も要らない (研究 §2.4)。
	slipping := e.slip.Observe(logical)
	if slipping {
		e.stats.SlipDetections++
	}

	// 機体パラメータの補正は vision が新しいときだけ許す。
	e.f.paramsFrozen = !e.hasVision || s.Stamp.Sub(e.lastVision) > e.opts.ParamFreezeAfter
	if e.f.paramsFrozen {
		e.stats.ParamFrozenCycles++
	}

	info := e.f.updateWheels(logical)
	if info.Applied {
		e.stats.WheelUpdates++
		if info.HuberWeight < 1 {
			e.stats.HuberDownweights++
		}
	}

	zupt := e.zuptDetect(s.Stamp, logical, slipping)
	if zupt {
		if z := e.f.updateZupt(); z.Applied {
			e.stats.ZuptUpdates++
		}
	}

	e.lastStamp = s.Stamp
	e.pushEntry(s.Stamp, dt, logical, true, zupt)
	e.guardDivergence()
	return info
}

// applyWheelRetro は過去の時刻の車輪観測を、バッファを巻き戻して取り込む。
// applyVisionRetro と同じ仕掛け。
func (e *Estimator) applyWheelRetro(stamp Stamp, logical [NumWheels]float64) UpdateInfo {
	i := e.buf.findAtOrBefore(stamp)
	if i < 0 {
		e.stats.WheelTooOld++
		return UpdateInfo{}
	}
	entry := e.buf.at(i)
	e.f.x = entry.x
	e.f.P = entry.p
	if gap := stamp.Sub(entry.stamp).Seconds(); gap > 0 {
		e.f.predict(gap)
	}
	info := e.f.updateWheels(logical)
	if info.Applied {
		e.stats.WheelUpdates++
	}

	e.replayFrom(i, stamp)
	e.guardDivergence()
	return info
}

// zuptDetect は停止の疑似観測を当ててよいかを判定する (研究 §4.5)。
//
// **3 条件の AND + 連続回数**で判定する。誤検出は致命的 (動いているのに
// 位置が固まる) なので、条件は厳しめにしてある。
//
//	(1) 4 輪の角速度がすべて閾値以下
//	(2) vision が新しく、その見かけの速度が閾値以下
//	(3) 冗長残差がスリップと言っていない
//
// (2) を入れるのは、車輪の読みが壊れて 0 のまま機体が動く事故
// (Trajectory POC Log §5-18) で推定を固めないため。
func (e *Estimator) zuptDetect(now Stamp, logical [NumWheels]float64, slipping bool) bool {
	n := e.f.cfg.Noise
	if !n.EnableZupt {
		e.zuptStreak = 0
		return false
	}
	ok := !slipping
	if ok {
		for i := 0; i < NumWheels; i++ {
			if math.Abs(logical[i]) > n.ZuptWheelThreshold {
				ok = false
				break
			}
		}
	}
	if ok {
		fresh := e.hasVision && now.Sub(e.lastVision) <= e.opts.VisionTimeout
		if !fresh || e.visionSpeed > n.ZuptVisionSpeedThreshold {
			ok = false
		}
	}
	if !ok {
		e.zuptStreak = 0
		return false
	}
	e.zuptStreak++
	return e.zuptStreak >= e.opts.ZuptMinCycles
}

// AddImu は IMU サンプルを取り込む。
//
// **ジャイロは omega の観測**として入れる (理由は NoiseConfig.EnableGyro の
// コメント)。バイアス b_g を同時に推定し、停止中の疑似観測 (ZARU) で
// 較正し直す。
//
// **加速度計は速度の予測に使わない。** RoboTeam Twente の実測で走行中の
// 標準偏差が 2.5 m/s^2 (静止時の 100 倍) あり、頼れる精度ではない。
// 閾値を超えたときだけ衝突とみなし、しばらくプロセス雑音を膨らませる。
//
// STM がまだ IMU を送ってこない機体では HasGyro / HasAccel が false になり、
// この関数は何もしない。**その状態でも推定はそのまま動く。**
func (e *Estimator) AddImu(s ImuSample) UpdateInfo {
	n := e.f.cfg.Noise
	if s.HasAccel && n.AccelCollisionThreshold > 0 {
		if math.Hypot(s.Accel.X, s.Accel.Y) > n.AccelCollisionThreshold {
			// 8 ms 周期で 25 周期 = 200 ms ぶん膨らませる。
			e.f.noteCollision(25)
			e.stats.Collisions++
		}
	}
	if !s.HasGyro || !n.EnableGyro {
		return UpdateInfo{}
	}
	if !e.started {
		e.started = true
		e.lastStamp = s.Stamp
		e.pushEntry(s.Stamp, 0, [NumWheels]float64{}, false, false)
		return UpdateInfo{}
	}
	dt := s.Stamp.Sub(e.lastStamp).Seconds()
	if dt < 0 {
		return UpdateInfo{}
	}
	e.f.predict(dt)
	info := e.f.updateGyro(s.GyroZ)
	if info.Applied {
		e.stats.GyroUpdates++
	}
	e.lastStamp = s.Stamp
	e.pushGyroEntry(s.Stamp, dt, s.GyroZ)
	e.guardDivergence()
	return info
}

// GyroBias は推定中のジャイロバイアス [rad/s] を返す。
func (e *Estimator) GyroBias() float64 { return e.f.x.GyroBias }

func (e *Estimator) pushGyroEntry(stamp Stamp, dt float64, gyro float64) {
	e.buf.push(bufferEntry{
		stamp:   stamp,
		x:       e.f.x,
		p:       e.f.P,
		dt:      dt,
		gyroZ:   gyro,
		hasGyro: true,
	})
}

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
	// 車輪は無いので hasWheels は false。**vision は保存する** (再フィルタ用)。
	e.pushVisionEntry(v.Stamp, dt, v.Pose)
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

	// **この観測もバッファへ残す。** さらに古い観測が後から来たときに
	// もう一度当て直せるようにする。
	if info.Applied {
		e.recordVisionAt(i, v)
	}

	// 現在時刻まで、保存してある観測で再フィルタする。
	e.replayFrom(i, v.Stamp)

	e.afterVision(v, info)
	e.guardDivergence()
	return info
}

// recordVisionAt は、巻き戻して当てた vision をバッファの区間へ覚えさせる。
//
// エントリ i の直後 (時刻順で v.Stamp の位置) に置きたいが、リングバッファへ
// 挿入はできないので、**v.Stamp 以降で最初のエントリに相乗りさせる**。
// 再フィルタの順序は「予測 -> 車輪 -> ジャイロ -> vision -> ZUPT」なので、
// 同じ時刻に複数の観測があっても結果は変わらない。
func (e *Estimator) recordVisionAt(i int, v VisionPose) {
	for j := i; j < e.buf.len(); j++ {
		en := e.buf.at(j)
		if en.stamp < v.Stamp {
			continue
		}
		if en.hasVision {
			return // 既にこの区間の vision を持っている。
		}
		en.vision = v.Pose
		// **観測の本当の時刻を残す。** エントリの時刻で代用すると、
		// 再フィルタのたびに最大 1 周期ぶん遅れて当たる。
		en.visionStamp = v.Stamp
		en.hasVision = true
		return
	}
}

func (e *Estimator) afterVision(v VisionPose, info UpdateInfo) {
	if !info.Applied {
		return
	}
	e.trackVisionSpeed(v)
	// Huber で重みを落とした観測は NIS の統計から外す (外れ値なので)。
	if info.HuberWeight >= 1 {
		e.stats.VisionNISSum += info.Normalized * info.Normalized
		e.stats.VisionNISCount++
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

// visionSpeedBaseline は見かけの速度を測る基線の最小長。
//
// **連続フレームの差分で速度を作ってはいけない。** 116 Hz・雑音 0.4 mm では
// 見かけの速度が 0.4mm*sqrt(2)/8.6ms = 66 mm/s になり、停止判定の閾値
// (20 mm/s) を常に超えてしまう。基線を 100 ms 取れば雑音は 5.7 mm/s まで落ちる。
const visionSpeedBaseline = 100 * time.Millisecond

// trackVisionSpeed は vision フレームの差分から見かけの速度を出す。
//
// **フィルタの推定速度ではなく観測の差分を使う**のが肝。ZUPT の判定に
// 推定を混ぜると、推定が固まっているときに「止まっている」と誤判定して
// そのまま固まり続ける自己確認ループになる。
func (e *Estimator) trackVisionSpeed(v VisionPose) {
	if !e.hasPrevVision {
		e.prevVision = v
		e.hasPrevVision = true
		return
	}
	dt := v.Stamp.Sub(e.prevVision.Stamp)
	if dt < visionSpeedBaseline {
		return // 基線が足りない。基準点は据え置く。
	}
	if dt > 2*visionSpeedBaseline+e.opts.VisionTimeout {
		// 長く途切れた後。基準点を張り直すだけにする。
		e.prevVision = v
		return
	}
	dx := v.Pose.X - e.prevVision.Pose.X
	dy := v.Pose.Y - e.prevVision.Pose.Y
	speed := math.Hypot(dx, dy) / dt.Seconds()
	// 1 区間の雑音で跳ねないよう軽く平滑化する。
	const alpha = 0.5
	e.visionSpeed += alpha * (speed - e.visionSpeed)
	e.prevVision = v
}

// VisionSpeed は連続する vision フレームから測った見かけの速度 [m/s] を返す。
func (e *Estimator) VisionSpeed() float64 { return e.visionSpeed }

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
	e.trackVisionSpeed(v)
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

func (e *Estimator) pushEntry(stamp Stamp, dt float64, wheels [NumWheels]float64, has, zupt bool) {
	e.buf.push(bufferEntry{
		stamp:     stamp,
		x:         e.f.x,
		p:         e.f.P,
		dt:        dt,
		wheels:    wheels,
		hasWheels: has,
		zupt:      zupt,
	})
}

// pushVisionEntry は vision を当てた時刻のエントリを積む。
// 再フィルタで同じ観測をもう一度当てられるよう、姿勢も持たせる。
func (e *Estimator) pushVisionEntry(stamp Stamp, dt float64, pose Pose2) {
	e.buf.push(bufferEntry{
		stamp:       stamp,
		x:           e.f.x,
		p:           e.f.P,
		dt:          dt,
		vision:      pose,
		visionStamp: stamp,
		hasVision:   true,
	})
}

// replayFrom は i より後のバッファを、保存してある観測で再フィルタする。
//
// **観測の時刻を守ること。** vision はエントリの時刻とは別の時刻を持つので、
// エントリより前ならそこまで予測してから当てる。ここを雑にすると、
// 再フィルタのたびに最大 1 周期ぶん遅れて当たり、系統的な遅れになる。
func (e *Estimator) replayFrom(i int, from Stamp) {
	prev := from
	for j := i + 1; j < e.buf.len(); j++ {
		en := e.buf.at(j)
		if en.hasVision && en.visionStamp < en.stamp {
			if dt := en.visionStamp.Sub(prev).Seconds(); dt > 0 {
				e.f.predict(dt)
				prev = en.visionStamp
			}
			e.f.updateVision(en.vision)
		}
		if dt := en.stamp.Sub(prev).Seconds(); dt > 0 {
			e.f.predict(dt)
		}
		if en.hasWheels {
			e.f.updateWheels(en.wheels)
		}
		if en.hasGyro {
			e.f.updateGyro(en.gyroZ)
		}
		if en.hasVision && en.visionStamp >= en.stamp {
			e.f.updateVision(en.vision)
		}
		if en.zupt {
			e.f.updateZupt()
		}
		en.x = e.f.x
		en.p = e.f.P
		prev = en.stamp
		e.stats.Replays++
	}
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

	e.f.resetNominal()
	e.f.resetCovariance()
	e.f.resetAdaptiveR()
	if ok {
		e.f.setPose(pose)
	}
	e.buf.reset()
	e.hasVision = false
	e.hasPrevVision = false
	e.visionSpeed = 0
	e.zuptStreak = 0
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
		Params: KinematicParams{
			TransScale: e.f.x.Kv,
			RotScale:   e.f.x.Kw,
			AngleBias:  e.f.x.Ka,
			Frozen:     e.f.paramsFrozen,
		},
		WheelResidual: e.slip.Residual(),
		Stationary:    e.zuptStreak >= e.opts.ZuptMinCycles,
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

// VisionNIS は vision のイノベーションの正規化二乗の平均を返す。
//
// **観測の次元 (3) になっているのが正しい。** 大きければ Q か R が小さすぎ、
// 小さければ大きすぎる。真値が要らないので実機でそのまま使える。
func (e *Estimator) VisionNIS() (float64, bool) {
	if e.stats.VisionNISCount < 30 {
		return 0, false
	}
	return e.stats.VisionNISSum / float64(e.stats.VisionNISCount), true
}

// SlipRate は冗長残差がスリップと判定した周期の割合を返す。
// 0.5 を超え続けるなら幾何を疑うこと (研究 §2.4)。
func (e *Estimator) SlipRate() float64 { return e.slip.SlipRate() }

// WheelNoiseScaleFactor は「実測の残差の広がり / 雑音モデルの予測」を返す。
// 1.0 なら WheelNoise と WheelNoiseSpeedCoef が実機と合っている。
func (e *Estimator) WheelNoiseScaleFactor() (float64, bool) { return e.slip.NoiseScaleFactor() }

// WheelResidualBias は滑っていない区間の冗長残差の平均 [rad/s] を返す。
// **ゼロから離れているなら幾何が間違っている。**
func (e *Estimator) WheelResidualBias() (float64, bool) { return e.slip.ResidualBias() }

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
