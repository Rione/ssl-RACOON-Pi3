package locsim

import (
	"fmt"
	"math"
	"sort"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// 効果を数値で示すための評価 (計画 §9)。
//
// RAVEN の EKF は「効果が測れなかった」ため不採用になった。
// 比較対象を固定する ——「vision 生値の RMSE」対「融合値の RMSE」—— のが肝で、
// これを単体テストの合格条件にすることで、改善していなければ CI が落ちる。

// Errors は真値に対する誤差の要約。
type Errors struct {
	// PositionRMSEm / PositionMaxM は位置誤差 [m]。
	PositionRMSEm float64
	PositionMaxM  float64
	// HeadingRMSErad / HeadingMaxRad は角度誤差 [rad]。
	HeadingRMSErad float64
	HeadingMaxRad  float64
	// Samples は評価に使ったサンプル数。
	Samples int
}

// String は人が読む 1 行を返す。
func (e Errors) String() string {
	return fmt.Sprintf("pos rmse %.2f mm (max %.2f) | heading rmse %.3f deg (max %.3f) | n=%d",
		e.PositionRMSEm*1000, e.PositionMaxM*1000,
		e.HeadingRMSErad*180/math.Pi, e.HeadingMaxRad*180/math.Pi, e.Samples)
}

// PoseLookup は時刻を受けて推定姿勢を返す。
type PoseLookup func(localization.Stamp) (localization.Pose2, bool)

// EvaluatePoses は与えた時刻列で推定姿勢を真値と比べる。
func EvaluatePoses(s *Sensors, times []localization.Stamp, lookup PoseLookup) Errors {
	var e Errors
	var sumPos, sumHead float64
	for _, t := range times {
		truth, ok := s.TruthAt(t)
		if !ok {
			continue
		}
		pose, ok := lookup(t)
		if !ok {
			continue
		}
		dx := pose.X - truth.Pose.X
		dy := pose.Y - truth.Pose.Y
		dp := math.Hypot(dx, dy)
		dh := math.Abs(localization.AngleDiff(pose.Theta, truth.Pose.Theta))

		sumPos += dp * dp
		sumHead += dh * dh
		e.PositionMaxM = math.Max(e.PositionMaxM, dp)
		e.HeadingMaxRad = math.Max(e.HeadingMaxRad, dh)
		e.Samples++
	}
	if e.Samples > 0 {
		e.PositionRMSEm = math.Sqrt(sumPos / float64(e.Samples))
		e.HeadingRMSErad = math.Sqrt(sumHead / float64(e.Samples))
	}
	return e
}

// EvaluateEstimates は推定器の出力列を真値と比べる。
func EvaluateEstimates(s *Sensors, est []localization.Estimate) Errors {
	times := make([]localization.Stamp, len(est))
	index := make(map[localization.Stamp]localization.Pose2, len(est))
	for i, e := range est {
		times[i] = e.Stamp
		index[e.Stamp] = e.Pose
	}
	return EvaluatePoses(s, times, func(t localization.Stamp) (localization.Pose2, bool) {
		p, ok := index[t]
		return p, ok
	})
}

// RawVisionHold は「最後に届いた vision 観測をそのまま現在位置として使う」
// 参照実装を返す。フィルタを入れなかった場合の比較対象である。
//
// 保持には 2 つの誤差が乗る: vision の観測雑音そのものと、
// 次のフレームが来るまで古い値を持ち続けることによる遅れ。
// 実機で RAVEN が自機に対してやっていることに相当する。
func RawVisionHold(s *Sensors) PoseLookup {
	obs := append([]localization.VisionPose(nil), s.Vision...)
	sort.Slice(obs, func(i, j int) bool { return obs[i].Stamp < obs[j].Stamp })

	return func(t localization.Stamp) (localization.Pose2, bool) {
		// t 以下で最大の Stamp を持つ観測。
		i := sort.Search(len(obs), func(i int) bool { return obs[i].Stamp > t }) - 1
		if i < 0 {
			return localization.Pose2{}, false
		}
		return obs[i].Pose, true
	}
}

// EvaluateVisionRaw は vision 生値を、推定器と同じ時刻列で評価する。
func EvaluateVisionRaw(s *Sensors, times []localization.Stamp) Errors {
	return EvaluatePoses(s, times, RawVisionHold(s))
}

// Improvement は位置 RMSE の削減率を返す。0.5 なら 50% 改善。
// 悪化していれば負になる。
func Improvement(raw, fused Errors) float64 {
	if raw.PositionRMSEm == 0 {
		return 0
	}
	return 1 - fused.PositionRMSEm/raw.PositionRMSEm
}

// --- 一貫性 (NEES) ------------------------------------------------------

// カイ二乗分布 (自由度 3) の 95% 区間。
//
// 共分散が誤差を正しく表現していれば、NEES はこの区間に 95% の割合で入る。
// 位置制御が共分散を見てゲインを落とす判断をする以上、ここが合っていないと
// 使えない (計画 §9 / §10-8)。
const (
	Chi2Df3Lower = 0.2158
	Chi2Df3Upper = 9.3484
)

// Consistency は NEES の検定結果。
type Consistency struct {
	// Samples は検定に使ったサンプル数。
	Samples int
	// InInterval は 95% 区間に入ったサンプル数。
	InInterval int
	// MeanNEES は NEES の平均。自由度 3 なので理想は 3.0。
	MeanNEES float64
	// Optimistic / Pessimistic は区間から外れた向きの内訳。
	//
	// Optimistic は「共分散が実際の誤差より小さい」= 推定が過信している状態で、
	// こちらのほうが危険。制御が信じてはいけない値を信じる。
	Optimistic  int
	Pessimistic int
	// Singular は共分散が反転できなかった数。
	Singular int
}

// Ratio は 95% 区間に入った割合を返す。
func (c Consistency) Ratio() float64 {
	if c.Samples == 0 {
		return 0
	}
	return float64(c.InInterval) / float64(c.Samples)
}

func (c Consistency) String() string {
	return fmt.Sprintf("NEES mean %.2f (ideal 3.0) | in 95%% interval %.1f%% | optimistic %d, pessimistic %d, singular %d | n=%d",
		c.MeanNEES, c.Ratio()*100, c.Optimistic, c.Pessimistic, c.Singular, c.Samples)
}

// EvaluateConsistency は推定の共分散が誤差を正しく表現しているかを検定する。
func EvaluateConsistency(s *Sensors, est []localization.Estimate) Consistency {
	var c Consistency
	var sum float64
	for _, e := range est {
		truth, ok := s.TruthAt(e.Stamp)
		if !ok {
			continue
		}
		err := [3]float64{
			e.Pose.X - truth.Pose.X,
			e.Pose.Y - truth.Pose.Y,
			localization.AngleDiff(e.Pose.Theta, truth.Pose.Theta),
		}
		inv, ok := invert3(e.CovPose)
		if !ok {
			c.Singular++
			c.Samples++
			continue
		}

		var nees float64
		for i := 0; i < 3; i++ {
			for j := 0; j < 3; j++ {
				nees += err[i] * inv[i][j] * err[j]
			}
		}
		c.Samples++
		sum += nees
		switch {
		case nees < Chi2Df3Lower:
			c.Pessimistic++
		case nees > Chi2Df3Upper:
			c.Optimistic++
		default:
			c.InInterval++
		}
	}
	if n := c.Samples - c.Singular; n > 0 {
		c.MeanNEES = sum / float64(n)
	}
	return c
}

// invert3 は 3x3 の逆行列を返す。特異なら false。
func invert3(m localization.Mat3) (localization.Mat3, bool) {
	a := m
	det := a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1]) -
		a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0]) +
		a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0])
	if det == 0 || math.IsNaN(det) || math.IsInf(det, 0) {
		return localization.Mat3{}, false
	}
	id := 1 / det
	var out localization.Mat3
	out[0][0] = (a[1][1]*a[2][2] - a[1][2]*a[2][1]) * id
	out[0][1] = (a[0][2]*a[2][1] - a[0][1]*a[2][2]) * id
	out[0][2] = (a[0][1]*a[1][2] - a[0][2]*a[1][1]) * id
	out[1][0] = (a[1][2]*a[2][0] - a[1][0]*a[2][2]) * id
	out[1][1] = (a[0][0]*a[2][2] - a[0][2]*a[2][0]) * id
	out[1][2] = (a[0][2]*a[1][0] - a[0][0]*a[1][2]) * id
	out[2][0] = (a[1][0]*a[2][1] - a[1][1]*a[2][0]) * id
	out[2][1] = (a[0][1]*a[2][0] - a[0][0]*a[2][1]) * id
	out[2][2] = (a[0][0]*a[1][1] - a[0][1]*a[1][0]) * id
	return out, true
}

// Report は 1 シナリオの評価結果をまとめたもの。
type Report struct {
	Scenario    string
	Raw         Errors
	Fused       Errors
	Improvement float64
	Consistency Consistency
}

func (r Report) String() string {
	return fmt.Sprintf("%s\n  raw vision : %s\n  fused      : %s\n  improvement: %+.1f%%\n  %s",
		r.Scenario, r.Raw, r.Fused, r.Improvement*100, r.Consistency)
}
