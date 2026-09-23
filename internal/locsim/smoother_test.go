package locsim

import (
	"math"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// SmootherInputs はセンサ列を RTS スムーザの入力へ変換する。
//
// vision の時刻は timesync の残差ぶんを引いてから渡す。オンラインの
// EstimatorOptions.VisionDelayComp と同じ補正である。
func SmootherInputs(s *Sensors, visionDelayComp time.Duration) []localization.SmootherInput {
	out := make([]localization.SmootherInput, 0, len(s.Wheels)+len(s.Vision))
	for _, w := range s.Wheels {
		var logical [localization.NumWheels]float64
		for slot := 0; slot < localization.NumWheels; slot++ {
			logical[s.Config.Geometry.WheelSlotOrder[slot]] = w.Omega[slot]
		}
		out = append(out, localization.SmootherInput{
			Stamp: w.Stamp, Wheels: logical, HasWheels: true,
		})
	}
	for _, v := range s.Vision {
		out = append(out, localization.SmootherInput{
			Stamp: v.Stamp - localization.Stamp(visionDelayComp), Vision: v.Pose, HasVision: true,
		})
	}
	return out
}

// **平滑化は、モデルが合っていれば欠落区間でオンライン推定に明確に勝つ。**
//
// 実機には真値が無いので、平滑化の結果を疑似真値に使う (計画 §8)。
// ただし**疑似真値はモデルの仮定をそのまま受け継ぐ**ので、その性質を
// テストで固定しておく:
//
//   - **滑りが実際にある** (= スリップ状態の仮定が正しい) とき、欠落区間で
//     平滑化がはっきり勝つ。未来の観測で欠落区間を引き戻せるため。
//   - **滑りが無い**とき、スリップ状態は「使われない自由度」になり、
//     平滑化は欠落区間の変位を v と s に配分してしまう。位置は v からしか
//     積まれないので、配分を誤ると位置が悪くなる。オンライン推定は因果なので
//     この自由度を使えず、かえって悪化しない。
//
// **だから欠落区間の評価に疑似真値を使うときは、その走りに滑りがあるかを
// 意識すること。** 迷うなら `EnableSlip = false` の設定でも作って比べる。
func TestSmootherBeatsOnlineFilterWhenTheModelFits(t *testing.T) {
	cfg := DefaultConfig()
	cfg.VisionOutages = []Outage{{Start: 5 * time.Second, End: 6 * time.Second}}
	cfg.Slip = ConstantSlip(localization.Vec2{X: 0.08, Y: -0.05})
	tr := FigureEight(localization.Vec2{}, 1.2, 4*time.Second,
		HeadingParams{Mode: HeadingSpin, SpinRate: 2.0}, 10*time.Second)
	s, err := Generate(tr, cfg, 33)
	if err != nil {
		t.Fatal(err)
	}

	filterCfg := localization.DefaultConfig()
	opts := localization.EstimatorOptions{VisionDelayComp: s.Config.VisionTimeBias}
	online, _, err := RunFilter(s, filterCfg, opts)
	if err != nil {
		t.Fatal(err)
	}
	smoothed, err := localization.Smooth(SmootherInputs(s, s.Config.VisionTimeBias), filterCfg)
	if err != nil {
		t.Fatal(err)
	}

	outage := &cfg.VisionOutages[0]
	onOut := poseRMSE(t, s, outage, func(yield func(localization.Stamp, localization.Pose2)) {
		for _, e := range online {
			yield(e.Stamp, e.Pose)
		}
	})
	smOut := poseRMSE(t, s, outage, func(yield func(localization.Stamp, localization.Pose2)) {
		for _, e := range smoothed {
			yield(e.Stamp, e.Pose)
		}
	})
	t.Logf("during the 1 s vision outage (with real slip): online %.2f mm, smoothed %.2f mm (%.0f%% better)",
		onOut*1000, smOut*1000, 100*(1-smOut/onOut))
	if smOut >= onOut*0.8 {
		t.Errorf("the smoother gave %.2f mm against the online %.2f mm; it should be clearly better "+
			"during an outage when the model fits", smOut*1000, onOut*1000)
	}
}

// 滑りが無い走りでは、平滑化とオンライン推定はほぼ並ぶ。
//
// vision が 116 Hz・0.4 mm で来ている間はオンライン推定がすでにほぼ最適で、
// 未来の観測を足しても伸びしろが無い。**これは不具合ではない** (上のコメント)。
func TestSmootherMatchesOnlineFilterWithoutSlip(t *testing.T) {
	cfg := DefaultConfig()
	cfg.VisionOutages = []Outage{{Start: 5 * time.Second, End: 6 * time.Second}}
	tr := FigureEight(localization.Vec2{}, 1.2, 4*time.Second,
		HeadingParams{Mode: HeadingSpin, SpinRate: 2.0}, 10*time.Second)
	s, err := Generate(tr, cfg, 33)
	if err != nil {
		t.Fatal(err)
	}
	filterCfg := localization.DefaultConfig()
	opts := localization.EstimatorOptions{VisionDelayComp: s.Config.VisionTimeBias}
	online, _, err := RunFilter(s, filterCfg, opts)
	if err != nil {
		t.Fatal(err)
	}
	smoothed, err := localization.Smooth(SmootherInputs(s, s.Config.VisionTimeBias), filterCfg)
	if err != nil {
		t.Fatal(err)
	}
	onErr := poseRMSE(t, s, nil, func(yield func(localization.Stamp, localization.Pose2)) {
		for _, e := range online {
			yield(e.Stamp, e.Pose)
		}
	})
	smErr := poseRMSE(t, s, nil, func(yield func(localization.Stamp, localization.Pose2)) {
		for _, e := range smoothed {
			yield(e.Stamp, e.Pose)
		}
	})
	t.Logf("overall position rmse without slip: online %.3f mm, smoothed %.3f mm", onErr*1000, smErr*1000)
	if smErr > onErr*1.2 {
		t.Errorf("the smoother is %.0f%% worse overall (%.3f vs %.3f mm); that is more than the "+
			"unused slip freedom should cost", 100*(smErr/onErr-1), smErr*1000, onErr*1000)
	}
}

// poseRMSE は推定列と真値の位置 RMSE を返す。
// window が非 nil ならその区間だけで測る。
func poseRMSE(t *testing.T, s *Sensors, window *Outage, iter func(func(localization.Stamp, localization.Pose2))) float64 {
	t.Helper()
	var sum float64
	var n int
	iter(func(stamp localization.Stamp, pose localization.Pose2) {
		if window != nil {
			if stamp < localization.Stamp(window.Start) || stamp > localization.Stamp(window.End) {
				return
			}
		} else if stamp < localization.Stamp(time.Second) {
			// 立ち上がりは除く (最初の vision が来るまで位置は未知)。
			return
		}
		truth, ok := s.TruthAt(stamp)
		if !ok {
			return
		}
		dx := pose.X - truth.Pose.X
		dy := pose.Y - truth.Pose.Y
		sum += dx*dx + dy*dy
		n++
	})
	if n == 0 {
		t.Fatal("no samples to compare")
	}
	return math.Sqrt(sum / float64(n))
}
