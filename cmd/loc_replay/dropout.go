package main

import (
	"fmt"
	"math"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// vision を人為的に落として、推定がどこまで持つかを測る。
//
// **PoC が `traj_locreplay` で回したのと同じ手順**を再現している
// (trajpoc-dataset/reports/localization-dropout.txt)。比較できるように、
// 窓の位置も比較対象も揃えてある:
//
//	窓        0.80-1.10 / 1.80-2.10 / 2.80-3.10 s (軌道時刻)
//	比較対象  (1) 最後の vision のまま  (2) 直前 40 ms の速度で等速外挿
//
// **真値は「落とした vision そのもの」**である。推定器には渡さず、
// 答え合わせにだけ使う。実機に真値が無い以上、これがいちばん素直な測り方になる。

// dropWindow は落とす区間 (軌道時刻)。
type dropWindow struct {
	start, end time.Duration
}

var defaultDropWindows = []dropWindow{
	{800 * time.Millisecond, 1100 * time.Millisecond},
	{1800 * time.Millisecond, 2100 * time.Millisecond},
	{2800 * time.Millisecond, 3100 * time.Millisecond},
}

type dropResult struct {
	window  dropWindow
	frames  int
	estMax  float64
	estLast float64
	// holdMax / holdLast は「最後の vision のまま」。
	holdMax, holdLast float64
	// extraMax / extraLast は等速外挿。
	extraMax, extraLast float64
	headLast            float64
}

// reportDropout は 1 本の走りについて vision 欠落の耐性を測る。
func reportDropout(cfg localization.Config, ws []sample, vs []visionSample, delay time.Duration) error {
	fmt.Printf("\n=== vision dropout (same protocol as trajpoc-dataset/reports/localization-dropout.txt) ===\n")

	// vision がある間の「届く前の推定 と vision の差」も出す。
	base, err := replayWithDrops(cfg, ws, vs, delay, nil)
	if err != nil {
		return err
	}
	fmt.Printf("with vision: prediction vs the next frame  %.2f mm  %.3f deg  (%d frames)\n",
		base.predRMSm*1000, base.predRMSrad*180/math.Pi, base.predCount)

	fmt.Printf("%-18s %6s %12s %12s %12s %10s\n",
		"window", "frames", "estimator", "hold last", "const vel", "heading")
	for _, w := range defaultDropWindows {
		r, err := dropOne(cfg, ws, vs, delay, w)
		if err != nil {
			return err
		}
		if r.frames == 0 {
			continue
		}
		fmt.Printf("%5.2f-%5.2f s    %6d  %5.1f/%5.1f  %5.1f/%5.1f  %5.1f/%5.1f  %+7.2f deg\n",
			w.start.Seconds(), w.end.Seconds(), r.frames,
			r.estMax*1000, r.estLast*1000,
			r.holdMax*1000, r.holdLast*1000,
			r.extraMax*1000, r.extraLast*1000,
			r.headLast*180/math.Pi)
	}
	fmt.Println("(max/last error in mm; smaller is better)")
	return nil
}

type dropStats struct {
	predRMSm   float64
	predRMSrad float64
	predCount  int
	res        dropResult
}

// dropOne は 1 つの窓について測る。
func dropOne(cfg localization.Config, ws []sample, vs []visionSample,
	delay time.Duration, w dropWindow) (dropResult, error) {
	st, err := replayWithDrops(cfg, ws, vs, delay, &w)
	if err != nil {
		return dropResult{}, err
	}
	st.res.window = w
	return st.res, nil
}

// replayWithDrops は drop が非 nil ならその窓の vision を推定器へ渡さず、
// 落とした枚数ぶん答え合わせをする。
func replayWithDrops(cfg localization.Config, ws []sample, vs []visionSample,
	delay time.Duration, drop *dropWindow) (dropStats, error) {
	opts := localization.EstimatorOptions{VisionDelayComp: delay}
	est, err := localization.NewEstimator(cfg, opts)
	if err != nil {
		return dropStats{}, err
	}

	var out dropStats
	var predSum, predHead float64

	// 落とす直前の vision 2 枚 (等速外挿の基準)。
	//
	// **窓に入った瞬間に固定する。** 最後まで更新し続けると、答え合わせのとき
	// 走行の終端の姿勢を基準にしてしまう (実際それで hold の誤差が 102 mm の
	// はずが 328 mm になっていた)。
	var lastKept, prevKept visionSample
	var haveKept, havePrev bool
	var refKept, refPrev visionSample
	var haveRef, haveRefPrev bool

	// 推定の軌跡 (答え合わせの内挿に使う)。
	track := make([]poseAt, 0, len(ws))

	dropped := make([]visionSample, 0, 64)
	vi := 0
	for _, wsample := range ws {
		for vi < len(vs) && vs[vi].arrival <= wsample.wheelStamp {
			v := vs[vi]
			vi++
			inDrop := drop != nil &&
				v.stamp >= localization.Stamp(drop.start) && v.stamp < localization.Stamp(drop.end)
			if inDrop {
				if !haveRef && haveKept {
					refKept, haveRef = lastKept, true
					refPrev, haveRefPrev = prevKept, havePrev
				}
				dropped = append(dropped, v)
				continue
			}
			// 届く前の推定と、これから入れる観測の差 (vision がある間の指標)。
			if cur := est.Current(); cur.Health != localization.HealthInvalid && len(track) > 0 {
				p := interpPose(track, v.stamp)
				dx := p.X - v.pose.X
				dy := p.Y - v.pose.Y
				predSum += dx*dx + dy*dy
				d := localization.AngleDiff(p.Theta, v.pose.Theta)
				predHead += d * d
				out.predCount++
			}
			est.AddVision(localization.VisionPose{
				Stamp: v.stamp, Arrival: v.arrival, Pose: v.pose, Mapped: true,
			})
			prevKept, havePrev = lastKept, haveKept
			lastKept, haveKept = v, true
		}
		est.AddWheel(localization.WheelSample{Stamp: wsample.wheelStamp, Omega: wsample.wheels})
		cur := est.Current()
		track = append(track, poseAt{stamp: cur.Stamp, pose: cur.Pose})
	}
	if out.predCount > 0 {
		out.predRMSm = math.Sqrt(predSum / float64(out.predCount))
		out.predRMSrad = math.Sqrt(predHead / float64(out.predCount))
	}
	if drop == nil || len(dropped) == 0 || !haveRef {
		return out, nil
	}

	// 等速外挿の速度 (窓に入る直前の 2 枚から)。
	var vx, vy float64
	if haveRefPrev {
		if dt := refKept.stamp.Sub(refPrev.stamp).Seconds(); dt > 0 {
			vx = (refKept.pose.X - refPrev.pose.X) / dt
			vy = (refKept.pose.Y - refPrev.pose.Y) / dt
		}
	}

	out.res.frames = len(dropped)
	for i, v := range dropped {
		p := interpPose(track, v.stamp)
		e := math.Hypot(p.X-v.pose.X, p.Y-v.pose.Y)
		hold := math.Hypot(refKept.pose.X-v.pose.X, refKept.pose.Y-v.pose.Y)
		dt := v.stamp.Sub(refKept.stamp).Seconds()
		ex := math.Hypot(refKept.pose.X+vx*dt-v.pose.X, refKept.pose.Y+vy*dt-v.pose.Y)

		out.res.estMax = math.Max(out.res.estMax, e)
		out.res.holdMax = math.Max(out.res.holdMax, hold)
		out.res.extraMax = math.Max(out.res.extraMax, ex)
		if i == len(dropped)-1 {
			out.res.estLast = e
			out.res.holdLast = hold
			out.res.extraLast = ex
			out.res.headLast = localization.AngleDiff(p.Theta, v.pose.Theta)
		}
	}
	return out, nil
}

// poseAt は推定の軌跡の 1 点。
type poseAt struct {
	stamp localization.Stamp
	pose  localization.Pose2
}

// interpPose は推定の軌跡から stamp の姿勢を線形内挿する。
func interpPose(track []poseAt, stamp localization.Stamp) localization.Pose2 {
	if len(track) == 0 {
		return localization.Pose2{}
	}
	lo, hi := 0, len(track)-1
	if stamp <= track[0].stamp {
		return track[0].pose
	}
	if stamp >= track[hi].stamp {
		return track[hi].pose
	}
	for lo+1 < hi {
		mid := (lo + hi) / 2
		if track[mid].stamp <= stamp {
			lo = mid
		} else {
			hi = mid
		}
	}
	a, b := track[lo], track[hi]
	span := b.stamp.Sub(a.stamp).Seconds()
	if span <= 0 {
		return a.pose
	}
	t := stamp.Sub(a.stamp).Seconds() / span
	return localization.Pose2{
		X:     a.pose.X + t*(b.pose.X-a.pose.X),
		Y:     a.pose.Y + t*(b.pose.Y-a.pose.Y),
		Theta: localization.WrapAngle(a.pose.Theta + t*localization.AngleDiff(b.pose.Theta, a.pose.Theta)),
	}
}
