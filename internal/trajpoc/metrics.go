package trajpoc

import (
	"fmt"
	"io"
	"math"
	"sort"
	"strings"

	"github.com/Rione/ssl-RACOON-Pi3/internal/control"
	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// Metrics は 1 回の走行の追従の良し悪し。距離は mm、角度は度、時間は ms で持つ
// (人が読む数字なので)。
type Metrics struct {
	Frames int // 評価に使った vision のフレーム数 (同じ撮影を重複して数えない)
	Held   int // vision が古くて 0 を出した周期の数

	PosRMS, PosMax         float64 // 参照位置との距離
	ContourRMS, ContourMax float64 // 経路 (時刻を無視した折れ線) との距離
	AlongMean              float64 // 進行方向の誤差の平均。負 = 参照より遅れている
	LagMs                  float64 // AlongMean を速度で割った「時間の遅れ」の中央値
	CrossRMS               float64 // 進行方向に直交する誤差
	HeadRMS, HeadMax       float64
	FinalPos, FinalHead    float64 // Settle の最後の誤差 (到着の精度)
	CmdAccelRMS            float64 // 指令の変化の激しさ [m/s^2]
	AgeMedianMs            float64 // vision の古さの中央値
	SettleHeadSwing        float64 // 軌道が終わった後の向きの誤差の振れ幅の半分 [deg] (振動の大きさ)
	SettleCmdMax           float64 // 軌道が終わった後の並進の指令の最大 [mm/s] (静止摩擦で機体を回す小さな補正)
}

// minMovingSpeed より遅い区間は進行方向が定まらないので along/cross に入れない。
const minMovingSpeed = 0.05

// ComputeMetrics は記録から指標を出す。path は貼り付け後の点列 (輪郭誤差用)。
func ComputeMetrics(samples []Sample, path []Knot) Metrics {
	var m Metrics
	var pos2, cont2, cross2, head2, acc2 float64
	var alongSum float64
	var alongN, accN int
	var lags, ages []float64
	seen := map[localization.Stamp]bool{}
	settleMin, settleMax := math.Inf(1), math.Inf(-1)
	for _, s := range samples {
		if s.Held || s.T <= 0 || s.Phase != control.Finished {
			continue
		}
		m.SettleCmdMax = math.Max(m.SettleCmdMax, math.Hypot(s.CmdWorld.X, s.CmdWorld.Y)*1000)
		if s.TV > 0 && s.Phase == control.Finished {
			h := localization.AngleDiff(s.Pose.Theta, s.Ref.Pose.Theta) * 180 / math.Pi
			settleMin, settleMax = math.Min(settleMin, h), math.Max(settleMax, h)
		}
	}
	if settleMax >= settleMin {
		m.SettleHeadSwing = (settleMax - settleMin) / 2
	}
	for i, s := range samples {
		if s.Held {
			m.Held++
			continue
		}
		if i > 0 && !samples[i-1].Held {
			dt := s.T - samples[i-1].T
			if dt > 1e-4 {
				dv := math.Hypot(s.CmdWorld.X-samples[i-1].CmdWorld.X, s.CmdWorld.Y-samples[i-1].CmdWorld.Y) / dt
				acc2 += dv * dv
				accN++
			}
		}
		if s.TV < 0 || seen[s.Capture] {
			continue
		}
		seen[s.Capture] = true
		m.Frames++
		ages = append(ages, s.Age*1000)
		ex, ey := s.Pose.X-s.Ref.Pose.X, s.Pose.Y-s.Ref.Pose.Y
		e := math.Hypot(ex, ey) * 1000
		pos2 += e * e
		m.PosMax = math.Max(m.PosMax, e)
		c := DistanceToPath(path, localization.Vec2{X: s.Pose.X, Y: s.Pose.Y}) * 1000
		cont2 += c * c
		m.ContourMax = math.Max(m.ContourMax, c)
		h := math.Abs(localization.AngleDiff(s.Pose.Theta, s.Ref.Pose.Theta)) * 180 / math.Pi
		head2 += h * h
		m.HeadMax = math.Max(m.HeadMax, h)
		if v := math.Hypot(s.Ref.VelWorld.X, s.Ref.VelWorld.Y); s.Phase == control.Tracking && v > minMovingSpeed {
			ux, uy := s.Ref.VelWorld.X/v, s.Ref.VelWorld.Y/v
			along := (ex*ux + ey*uy) * 1000
			cross := (-ex*uy + ey*ux) * 1000
			alongSum += along
			cross2 += cross * cross
			alongN++
			lags = append(lags, -along/(v*1000)*1000)
		}
		m.FinalPos, m.FinalHead = e, h
	}
	if m.Frames > 0 {
		n := float64(m.Frames)
		m.PosRMS, m.ContourRMS, m.HeadRMS = math.Sqrt(pos2/n), math.Sqrt(cont2/n), math.Sqrt(head2/n)
		m.AgeMedianMs = median(ages)
	}
	if alongN > 0 {
		m.AlongMean = alongSum / float64(alongN)
		m.CrossRMS = math.Sqrt(cross2 / float64(alongN))
		m.LagMs = median(lags)
	}
	if accN > 0 {
		m.CmdAccelRMS = math.Sqrt(acc2 / float64(accN))
	}
	return m
}

func median(v []float64) float64 {
	if len(v) == 0 {
		return 0
	}
	s := append([]float64(nil), v...)
	sort.Float64s(s)
	return s[len(s)/2]
}

// Format は人が読む要約を返す。
func (m Metrics) Format(cfg Config, state State, reason string) string {
	var b strings.Builder
	fmt.Fprintf(&b, "=== trajpoc result: method=%s Kp=%.2f Kth=%.2f lead=%.0fms ===\n",
		cfg.Method, cfg.Kp, cfg.Kth, cfg.Lead*1000)
	fmt.Fprintf(&b, "end: %s (%s)\n", state, reason)
	fmt.Fprintf(&b, "vision frames %d, held cycles %d, vision age median %.0f ms\n", m.Frames, m.Held, m.AgeMedianMs)
	fmt.Fprintf(&b, "position error     RMS %6.1f mm   max %6.1f mm\n", m.PosRMS, m.PosMax)
	fmt.Fprintf(&b, "contour error      RMS %6.1f mm   max %6.1f mm   (distance to the path, time ignored)\n", m.ContourRMS, m.ContourMax)
	fmt.Fprintf(&b, "along-track mean   %+6.1f mm   lag %.0f ms   (negative mean = behind the reference)\n", m.AlongMean, m.LagMs)
	fmt.Fprintf(&b, "cross-track        RMS %6.1f mm\n", m.CrossRMS)
	fmt.Fprintf(&b, "heading error      RMS %6.2f deg  max %6.2f deg\n", m.HeadRMS, m.HeadMax)
	fmt.Fprintf(&b, "final (arrival)    %6.1f mm  %6.2f deg\n", m.FinalPos, m.FinalHead)
	fmt.Fprintf(&b, "command accel RMS  %6.2f m/s^2  (how jerky the command was)\n", m.CmdAccelRMS)
	fmt.Fprintf(&b, "after the end      heading swing ±%.2f deg, max translation command %.0f mm/s  (near-goal=%v)\n",
		m.SettleHeadSwing, m.SettleCmdMax, cfg.NearGoal.Enabled)
	return b.String()
}

// WriteCSV は記録を CSV で書く (単位は列名に書く)。
func WriteCSV(w io.Writer, samples []Sample) error {
	if _, err := fmt.Fprintln(w, "t_s,tv_s,vision_age_ms,held,pose_x_mm,pose_y_mm,pose_theta_rad,"+
		"ref_x_mm,ref_y_mm,ref_theta_rad,ref_vx_mm_s,ref_vy_mm_s,ref_omega_rad_s,"+
		"cmd_world_vx_mm_s,cmd_world_vy_mm_s,cmd_omega_rad_s,cmd_body_vx_mm_s,cmd_body_vy_mm_s,"+
		"near_goal,pred_x_mm,pred_y_mm,pred_theta_rad,wheel_fl_rad_s,wheel_bl_rad_s,wheel_br_rad_s,wheel_fr_rad_s,"+
		"imu_valid,imu_yaw_rate_rad_s,imu_accel_x_m_s2,imu_accel_y_m_s2"); err != nil {
		return err
	}
	for _, s := range samples {
		held, ng, imu := 0, 0, 0
		if s.ImuValid {
			imu = 1
		}
		if s.Held {
			held = 1
		}
		if s.NearGoal {
			ng = 1
		}
		if _, err := fmt.Fprintf(w, "%.4f,%.4f,%.1f,%d,%.1f,%.1f,%.4f,%.1f,%.1f,%.4f,%.1f,%.1f,%.4f,%.1f,%.1f,%.4f,%.1f,%.1f,%d,%.1f,%.1f,%.4f,%.2f,%.2f,%.2f,%.2f,%d,%.4f,%.3f,%.3f\n",
			s.T, s.TV, s.Age*1000, held, s.Pose.X*1000, s.Pose.Y*1000, s.Pose.Theta,
			s.Ref.Pose.X*1000, s.Ref.Pose.Y*1000, s.Ref.Pose.Theta, s.Ref.VelWorld.X*1000, s.Ref.VelWorld.Y*1000, s.Ref.YawRate,
			s.CmdWorld.X*1000, s.CmdWorld.Y*1000, s.CmdOmega, s.CmdBody.X*1000, s.CmdBody.Y*1000,
			ng, s.Pred.X*1000, s.Pred.Y*1000, s.Pred.Theta,
			s.Wheels[0], s.Wheels[1], s.Wheels[2], s.Wheels[3],
			imu, s.ImuYawRate, s.ImuAccelX, s.ImuAccelY); err != nil {
			return err
		}
	}
	return nil
}

// DistanceToPath は点 p から、点列を結んだ折れ線までの最短距離 [m]。
// 輪郭誤差 (時刻を無視して、経路からどれだけ外れたか) に使う。
func DistanceToPath(knots []Knot, p localization.Vec2) float64 {
	best := math.Inf(1)
	for i := 1; i < len(knots); i++ {
		a := localization.Vec2{X: knots[i-1].Pose.X, Y: knots[i-1].Pose.Y}
		b := localization.Vec2{X: knots[i].Pose.X, Y: knots[i].Pose.Y}
		best = math.Min(best, pointSegment(p, a, b))
	}
	if len(knots) == 1 {
		best = math.Hypot(p.X-knots[0].Pose.X, p.Y-knots[0].Pose.Y)
	}
	return best
}

func pointSegment(p, a, b localization.Vec2) float64 {
	dx, dy := b.X-a.X, b.Y-a.Y
	l2 := dx*dx + dy*dy
	if l2 < 1e-18 {
		return math.Hypot(p.X-a.X, p.Y-a.Y)
	}
	u := ((p.X-a.X)*dx + (p.Y-a.Y)*dy) / l2
	u = math.Max(0, math.Min(1, u))
	return math.Hypot(p.X-(a.X+u*dx), p.Y-(a.Y+u*dy))
}
