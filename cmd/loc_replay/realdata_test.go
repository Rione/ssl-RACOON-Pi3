package main

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// datasetGlob は実機データの場所。無ければテストは skip する。
const datasetGlob = "../../trajpoc-dataset/poc/*.csv"

func loadRealRuns(t *testing.T) []*poCRun {
	t.Helper()
	if _, err := os.Stat(filepath.Dir(datasetGlob)); err != nil {
		t.Skip("trajpoc-dataset is not present")
	}
	runs, err := loadPoCRuns(datasetGlob)
	if err != nil {
		t.Skipf("cannot read the dataset: %v", err)
	}
	var out []*poCRun
	for _, r := range runs {
		if !r.hasWheels || len(r.wheels) < 300 || len(r.vision) < 100 {
			continue
		}
		// STM 無応答で車輪がゼロのままの走りを除く (index.csv の valid=no)。
		var peak float64
		for _, w := range r.wheels {
			for _, v := range w.wheels {
				peak = math.Max(peak, math.Abs(v))
			}
		}
		if peak < 0.5 {
			continue
		}
		out = append(out, r)
	}
	if len(out) < 4 {
		t.Skipf("only %d usable runs", len(out))
	}
	return out
}

// deadReckonError は**フィルタを通さず車輪だけ**で 0.3 s 積分し、
// vision の変位とのずれ [m] を窓ごとに返す。
//
// 幾何そのものの良し悪しを測る。フィルタ・スムーザ・時刻同期・雑音設定が
// 一切入らないので、幾何の比較にはこれがいちばん素直である。
func deadReckonError(t *testing.T, runs []*poCRun, g localization.GeometryConfig) []float64 {
	t.Helper()
	kin, err := localization.NewKinematics(g)
	if err != nil {
		t.Fatal(err)
	}
	windows := [][2]float64{{0.8, 1.1}, {1.8, 2.1}, {2.8, 3.1}}
	var out []float64
	for _, r := range runs {
		for _, w := range windows {
			start, end := localization.Stamp(w[0]*1e9), localization.Stamp(w[1]*1e9)
			var dx, dy float64
			var first, last localization.Pose2
			var have bool
			var prev localization.Stamp
			for i, ws := range r.wheels {
				if ws.wheelStamp < start || ws.wheelStamp > end {
					continue
				}
				pose := visionPoseNear(r, ws.wheelStamp)
				if !have {
					first, have = pose, true
					prev = ws.wheelStamp
					continue
				}
				dt := ws.wheelStamp.Sub(prev).Seconds()
				prev = ws.wheelStamp
				last = pose
				if dt <= 0 || dt > 0.05 {
					continue
				}
				var logical [localization.NumWheels]float64
				for slot := 0; slot < localization.NumWheels; slot++ {
					logical[g.WheelSlotOrder[slot]] = r.wheels[i].wheels[slot]
				}
				vx, vy, _ := kin.BodyFromWheel(logical)
				c, s := math.Cos(pose.Theta), math.Sin(pose.Theta)
				dx += (c*vx - s*vy) * dt
				dy += (s*vx + c*vy) * dt
			}
			if !have {
				continue
			}
			out = append(out, math.Hypot(dx-(last.X-first.X), dy-(last.Y-first.Y)))
		}
	}
	return out
}

func visionPoseNear(r *poCRun, stamp localization.Stamp) localization.Pose2 {
	lo, hi := 0, len(r.vision)-1
	if hi < 0 {
		return localization.Pose2{}
	}
	for lo+1 < hi {
		mid := (lo + hi) / 2
		if r.vision[mid].stamp <= stamp {
			lo = mid
		} else {
			hi = mid
		}
	}
	return r.vision[lo].pose
}

func meanOf(v []float64) float64 {
	if len(v) == 0 {
		return 0
	}
	var s float64
	for _, x := range v {
		s += x
	}
	return s / float64(len(v))
}

// **CAD の幾何をそのままオドメトリに使ってはいけない、という事実を実データで固定する。**
//
// 3D モデルの取付角は厳密に +-60 / +-135 度で、全機体が同じ設計である。
// それでも**実機の車輪は「有効取付角 55-56 度・有効アーム 70-74 mm」のほうが
// よく合う**。オムニ車輪のローラのコンプライアンスと滑りによる効果で、
// 文献でいう effective kinematic parameters がこれにあたる。
//
// このテストが落ちたら、**CAD を読んで既定値を「直した」可能性を疑うこと。**
func TestEffectiveGeometryBeatsCADGeometryOnRealData(t *testing.T) {
	runs := loadRealRuns(t)

	eff := meanOf(deadReckonError(t, runs, localization.DefaultGeometry()))
	cad := meanOf(deadReckonError(t, runs, localization.CADGeometry()))
	t.Logf("wheel-only dead reckoning over 0.3 s windows (%d runs): "+
		"effective %.2f mm, CAD geometry %.2f mm", len(runs), eff*1000, cad*1000)

	if eff >= cad {
		t.Errorf("the effective geometry (%.2f mm) is not better than the CAD geometry (%.2f mm); "+
			"either the default was changed to the CAD values, or the robot changed", eff*1000, cad*1000)
	}
}

// 有効アームは幾何 (78.45 mm) より短い。
//
// アームは**角速度にしか効かない**ので、向きで測る。誤差そのものを足すと
// 直進の走りの雑音に埋もれるので、**「車輪から積分した向きの変化」対
// 「vision の向きの変化」の傾き**を見る。アームはこの傾きをそのまま決めるので、
// 傾きが 1 に近いほど良い。
func TestEffectiveMomentArmIsShorterThanCAD(t *testing.T) {
	runs := loadRealRuns(t)
	short, nShort := headingScale(t, runs, 0.074)
	long, _ := headingScale(t, runs, 0.07845)
	if nShort < 4 {
		t.Skipf("only %d windows with enough rotation", nShort)
	}
	t.Logf("wheel heading change / vision heading change (%d windows): "+
		"arm 74.00 mm -> %.4f, 78.45 mm (CAD) -> %.4f  (1.0 is correct)", nShort, short, long)
	if math.Abs(short-1) > math.Abs(long-1) {
		t.Errorf("the CAD moment arm gave a better scale (%.4f vs %.4f); "+
			"re-measure before changing the default", long, short)
	}
}

// headingScale は「車輪から積分した向きの変化」対「vision の向きの変化」の
// 最小二乗の傾きと、使った窓の数を返す。
func headingScale(t *testing.T, runs []*poCRun, arm float64) (float64, int) {
	t.Helper()
	g := localization.DefaultGeometry()
	g.MomentArmM = arm
	kin, err := localization.NewKinematics(g)
	if err != nil {
		t.Fatal(err)
	}
	windows := [][2]float64{{0.8, 1.1}, {1.8, 2.1}, {2.8, 3.1}}
	var num, den float64
	var n int
	for _, r := range runs {
		for _, w := range windows {
			start, end := localization.Stamp(w[0]*1e9), localization.Stamp(w[1]*1e9)
			var dth float64
			var first, last localization.Pose2
			var have bool
			var prev localization.Stamp
			for i, ws := range r.wheels {
				if ws.wheelStamp < start || ws.wheelStamp > end {
					continue
				}
				pose := visionPoseNear(r, ws.wheelStamp)
				if !have {
					first, have = pose, true
					prev = ws.wheelStamp
					continue
				}
				dt := ws.wheelStamp.Sub(prev).Seconds()
				prev = ws.wheelStamp
				last = pose
				if dt <= 0 || dt > 0.05 {
					continue
				}
				var logical [localization.NumWheels]float64
				for slot := 0; slot < localization.NumWheels; slot++ {
					logical[g.WheelSlotOrder[slot]] = r.wheels[i].wheels[slot]
				}
				_, _, omega := kin.BodyFromWheel(logical)
				dth += omega * dt
			}
			if !have {
				continue
			}
			ref := localization.AngleDiff(last.Theta, first.Theta)
			// 回っていない窓は傾きを決められないので外す (3 度 = 0.052 rad)。
			if math.Abs(ref) < 0.052 {
				continue
			}
			num += ref * dth
			den += ref * ref
			n++
		}
	}
	if den == 0 {
		return 0, 0
	}
	return num / den, n
}
