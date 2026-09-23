package locadapter

import (
	"fmt"
	"math"
	"sync/atomic"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi3/internal/loclog"
)

// LiveEstimator は実機で推定器を回す。
//
// **単一の goroutine (SPI ループ) からのみ推定器を触る** (計画 §7.2)。
// vision は別 goroutine が受けてチャンネルへ置き、ここで吸い上げる。
// こうするとロックが要らず、ホットパスでアロケートしない性質も保てる。
//
// 記録と同じく、**推定が壊れても走行機能は巻き込まない**。
// ObserveSPI は link.NotifySPI が recover で包んでいる。
type LiveEstimator struct {
	rec    *SPIRecorder
	est    *localization.Estimator
	vision <-chan localization.VisionPose
	writer *loclog.Writer

	// latest は制御・監視がロックなしで読む最新の推定。
	latest atomic.Pointer[localization.Estimate]

	visionApplied atomic.Int64
	cycles        atomic.Int64
	loopNs        atomic.Int64

	// logEvery は MCAP へ /est/state を書く間引き。0 で毎周期。
	logEvery int
	// actuation は指令が効くまでの遅れの推定器。
	actuation *localization.ActuationDelayEstimator
	actDelay  atomic.Int64 // [ns]、まだ決まらないうちは 0
}

// LiveEstimatorConfig は起動時の設定。
type LiveEstimatorConfig struct {
	Config  localization.Config
	Options localization.EstimatorOptions
	// LogEvery は /est/state を何周期に 1 回書くか。0 なら毎周期 (125 Hz)。
	LogEvery int
}

// NewLiveEstimator は実機用の推定器を作る。
func NewLiveEstimator(rec *SPIRecorder, vision <-chan localization.VisionPose,
	writer *loclog.Writer, cfg LiveEstimatorConfig) (*LiveEstimator, error) {
	if rec == nil {
		return nil, fmt.Errorf("live estimator: SPI recorder is required")
	}
	est, err := localization.NewEstimator(cfg.Config, cfg.Options)
	if err != nil {
		return nil, err
	}
	if cfg.LogEvery <= 0 {
		cfg.LogEvery = 1
	}
	return &LiveEstimator{
		rec:      rec,
		est:      est,
		vision:   vision,
		writer:   writer,
		logEvery: cfg.LogEvery,
		// 指令が効くまでの遅れは電池電圧で変わるので測り続ける
		// (Trajectory POC Log §5-21)。
		actuation: localization.NewActuationDelayEstimator(
			8*time.Millisecond, 4*time.Second, 200*time.Millisecond),
	}, nil
}

// ObserveSPI は link.SPIObserver を満たす。SPI ループから毎周期呼ばれる。
func (e *LiveEstimator) ObserveSPI(tx, rx []byte, before, after time.Time) {
	start := time.Now()
	s := e.rec.Record(tx, rx, before, after)
	if !s.Valid {
		return
	}

	// **vision を先に吸い上げる。** 溜まっているぶんをすべて入れてから
	// 車輪を当てる。時刻の前後は推定器の OOSM が面倒を見る。
	for {
		select {
		case v := <-e.vision:
			e.est.AddVision(v)
			e.visionApplied.Add(1)
			continue
		default:
		}
		break
	}

	if s.IMU.HasGyro || s.IMU.HasAccel {
		e.est.AddImu(s.IMU)
	}
	info := e.est.AddWheel(s.Wheel)

	cur := e.est.Current()
	e.latest.Store(&cur)

	// 指令が効くまでの遅れ: 出した並進指令の大きさと、推定した速度の大きさ。
	cmd := math.Hypot(float64(s.Command.VelXMmS), float64(s.Command.VelYMmS)) / 1000
	meas := math.Hypot(cur.VelBody.X, cur.VelBody.Y)
	e.actuation.Observe(cmd, meas)

	n := e.cycles.Add(1)
	e.loopNs.Store(int64(time.Since(start)))

	if e.writer != nil && int(n)%e.logEvery == 0 {
		e.writeState(cur, info)
	}
	// 4 秒ぶん溜まったら遅れを測り直す。相関が立たなければ前の値のまま。
	if n%500 == 0 {
		if d, ok := e.actuation.Estimate(0.5); ok {
			e.actDelay.Store(int64(d))
		}
	}
}

// Record は推定を記録・表示用のレコードへ直す。HTTP からも使う。
func (e *LiveEstimator) Record(cur localization.Estimate) loclog.EstimateRecord {
	return loclog.EstimateRecord{
		StampNs:           cur.Stamp.Nanoseconds(),
		XMm:               cur.Pose.X * 1000,
		YMm:               cur.Pose.Y * 1000,
		ThetaRad:          cur.Pose.Theta,
		VxMmS:             cur.VelBody.X * 1000,
		VyMmS:             cur.VelBody.Y * 1000,
		OmegaRadS:         cur.YawRate,
		SlipXMmS:          cur.Slip.X * 1000,
		SlipYMmS:          cur.Slip.Y * 1000,
		GyroBiasRadS:      e.est.GyroBias(),
		SigmaXMm:          math.Sqrt(math.Max(cur.CovPose[0][0], 0)) * 1000,
		SigmaYMm:          math.Sqrt(math.Max(cur.CovPose[1][1], 0)) * 1000,
		SigmaThetaRad:     math.Sqrt(math.Max(cur.CovPose[2][2], 0)),
		TransScale:        cur.Params.TransScale,
		RotScale:          cur.Params.RotScale,
		AngleBiasRad:      cur.Params.AngleBias,
		ParamsFrozen:      cur.Params.Frozen,
		WheelResidualRadS: cur.WheelResidual,
		Stationary:        cur.Stationary,
		SinceVisionMs:     float64(cur.SinceVision) / float64(time.Millisecond),
		Health:            cur.Health.String(),
	}
}

func (e *LiveEstimator) writeState(cur localization.Estimate, info localization.UpdateInfo) {
	e.writer.LogJSON(loclog.ChEstState, cur.Stamp, e.Record(cur))

	if info.Dim > 0 {
		e.writer.LogJSON(loclog.ChEstInnovation, cur.Stamp, loclog.InnovationRecord{
			StampNs:     cur.Stamp.Nanoseconds(),
			Kind:        "wheel",
			Dim:         info.Dim,
			Innovation:  info.Innovation[:info.Dim],
			Normalized:  info.Normalized,
			HuberWeight: info.HuberWeight,
			RScale:      info.RScale,
			Applied:     info.Applied,
		})
	}
}

// Latest は最新の推定を返す。ロックなしで読める。
func (e *LiveEstimator) Latest() (localization.Estimate, bool) {
	p := e.latest.Load()
	if p == nil {
		return localization.Estimate{}, false
	}
	return *p, true
}

// PredictAhead は「指令が効く時刻」の推定を返す。
//
// **遅れは測った値を使う。** 電池電圧で変わるので固定値にはできない
// (Trajectory POC Log §5-21: 午前 85 ms に合わせた先読みが満充電では効きすぎた)。
// まだ測れていないうちは fallback を使う。
func (e *LiveEstimator) PredictAhead(fallback time.Duration) localization.Estimate {
	d := time.Duration(e.actDelay.Load())
	if d <= 0 {
		d = fallback
	}
	return e.est.PredictAhead(d)
}

// ActuationDelay は測った「指令が効くまでの遅れ」を返す。
func (e *LiveEstimator) ActuationDelay() (time.Duration, bool) {
	d := time.Duration(e.actDelay.Load())
	return d, d > 0
}

// Stats は推定器の累積統計を返す。
func (e *LiveEstimator) Stats() loclog.EstimatorStatsRecord {
	st := e.est.Stats()
	out := loclog.EstimatorStatsRecord{
		WheelUpdates:      st.WheelUpdates,
		VisionUpdates:     st.VisionUpdates,
		GyroUpdates:       st.GyroUpdates,
		ZuptUpdates:       st.ZuptUpdates,
		WheelTooOld:       st.WheelTooOld,
		VisionTooOld:      st.VisionTooOld,
		VisionFuture:      st.VisionFuture,
		Replays:           st.Replays,
		Resets:            st.Resets,
		Collisions:        st.Collisions,
		SlipDetections:    st.SlipDetections,
		ParamFrozenCycles: st.ParamFrozenCycles,
		HuberDownweights:  st.HuberDownweights,
		SlipRate:          e.est.SlipRate(),
		LoopNs:            e.loopNs.Load(),
	}
	if v, ok := e.est.VisionNIS(); ok {
		out.VisionNIS = v
	}
	if v, ok := e.est.WheelNoiseScaleFactor(); ok {
		out.WheelNoiseScale = v
	}
	return out
}

// Status は人が 1 行で読む要約を返す。
//
// **「静かに壊れる」失敗を見えるようにするための行**である。
// 車輪の更新が止まる、幾何が合っていない、vision が来ていない、といった
// 事故は数字を出していないと気づけない (実機で実際に起きた)。
func (e *LiveEstimator) Status() string {
	cur, ok := e.Latest()
	if !ok {
		return "[EST] no estimate yet"
	}
	st := e.Stats()
	delay := "delay n/a"
	if d, ok := e.ActuationDelay(); ok {
		delay = fmt.Sprintf("act %dms", d.Milliseconds())
	}
	return fmt.Sprintf(
		"[EST] %s pos %.0f,%.0f mm th %.1f deg | v %.0f,%.0f mm/s w %.2f rad/s | "+
			"sigma %.1f mm | vision %.0f ms ago | k_v %.3f k_w %.3f%s | "+
			"NIS %.2f slip %.2f noise x%.2f | wheel %d vision %d zupt %d reset %d | %s | %.0f us",
		cur.Health,
		cur.Pose.X*1000, cur.Pose.Y*1000, cur.Pose.Theta*180/math.Pi,
		cur.VelBody.X*1000, cur.VelBody.Y*1000, cur.YawRate,
		math.Sqrt(math.Max(cur.CovPose[0][0], 0))*1000,
		float64(cur.SinceVision)/float64(time.Millisecond),
		cur.Params.TransScale, cur.Params.RotScale, frozenMark(cur.Params.Frozen),
		st.VisionNIS, st.SlipRate, st.WheelNoiseScale,
		st.WheelUpdates, st.VisionUpdates, st.ZuptUpdates, st.Resets,
		delay, float64(st.LoopNs)/1000,
	)
}

func frozenMark(frozen bool) string {
	if frozen {
		return " (frozen)"
	}
	return ""
}

// Warnings は「見たらすぐ手を打つべき」状態を返す。空なら異常なし。
//
// 実機で実際に起きた失敗をそのまま項目にしてある。
func (e *LiveEstimator) Warnings() []string {
	var out []string
	st := e.Stats()
	cur, ok := e.Latest()
	if !ok {
		return []string{"no estimate yet (no valid SPI frames?)"}
	}
	if st.WheelUpdates == 0 && st.VisionUpdates > 50 {
		out = append(out, "no wheel updates at all -- the wheel and vision timestamps disagree")
	}
	if st.WheelTooOld > 0 {
		out = append(out, fmt.Sprintf("%d wheel samples fell out of the buffer", st.WheelTooOld))
	}
	if st.SlipRate > 0.5 {
		out = append(out, fmt.Sprintf("slip rate %.2f -- the geometry is probably wrong, not the floor", st.SlipRate))
	}
	if st.WheelNoiseScale > 1.5 || (st.WheelNoiseScale > 0 && st.WheelNoiseScale < 0.6) {
		out = append(out, fmt.Sprintf("wheel noise model is off by x%.2f", st.WheelNoiseScale))
	}
	if st.Resets > 0 {
		out = append(out, fmt.Sprintf("the filter reset %d times (divergence)", st.Resets))
	}
	if cur.Health != localization.HealthOK {
		out = append(out, "health is "+cur.Health.String())
	}
	return out
}

// RunStatusLogger は一定間隔で 1 行出し続ける。done が閉じたら止まる。
func RunStatusLogger(done <-chan struct{}, e *LiveEstimator, writer *loclog.Writer,
	every time.Duration, logf func(string, ...any)) {
	if every <= 0 {
		every = time.Second
	}
	t := time.NewTicker(every)
	defer t.Stop()
	for {
		select {
		case <-done:
			return
		case <-t.C:
			logf("%s", e.Status())
			for _, w := range e.Warnings() {
				logf("[EST] WARNING: %s", w)
			}
			if writer != nil {
				if cur, ok := e.Latest(); ok {
					writer.LogJSON(loclog.ChEstTiming, cur.Stamp, e.Stats())
				}
			}
		}
	}
}
