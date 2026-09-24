package locsim

import (
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// プロセス雑音 Q の感度を地図にする (handoff §6.2 の 4 番目)。
//
// **合成データに合わせ込むためではない。** 実測値が来たときに当てはめる先を
// 用意しておくのと、「今どちら側に外しているか」を可視化するためにある。
//
// 2026-09-23 現在、vision の雑音を実測値 (0.4 mm) に下げた結果、
// **NEES が 0.2 前後まで落ちた (理想 3.0)**。フィルタは実際の誤差より
// 3〜4 倍大きい不確かさを申告している。安全側だが、位置制御が共分散を見て
// ゲインを決める以上、過度に弱気なのも困る。
//
// AccelNoise を下げれば NEES は 3 に近づくが、急加減速への追従が落ちる。
// **実機のログで決めること。** ここでは選ばない。
func TestProcessNoiseSensitivityMap(t *testing.T) {
	cfg := DefaultConfig()
	opts := localization.EstimatorOptions{VisionDelayComp: cfg.VisionTimeBias}

	type scen struct {
		name string
		tr   Trajectory
		cfg  Config
	}
	outage := cfg
	outage.VisionOutages = []Outage{{Start: 5 * time.Second, End: 6 * time.Second}}

	scens := []scen{
		{"figure eight", FigureEight(localization.Vec2{}, 1.5, 4*time.Second,
			HeadingParams{Mode: HeadingSpin, SpinRate: 2.0}, 10*time.Second), cfg},
		{"hard accel and stop", StepAccel(localization.Vec2{}, 0.3, 4.0, 2.5,
			2*time.Second, HeadingParams{Mode: HeadingFixed}, 10*time.Second), cfg},
		{"1 s vision outage", Line(localization.Vec2{}, localization.Vec2{X: 2, Y: 0},
			localization.Vec2{}, HeadingParams{Mode: HeadingSpin, SpinRate: 2.0}, 10*time.Second), outage},
	}

	accels := []float64{0.5, 1.0, 2.0, 4.0, 8.0}
	for _, sc := range scens {
		s, err := Generate(sc.tr, sc.cfg, 51)
		if err != nil {
			t.Fatal(err)
		}
		t.Logf("--- %s ---", sc.name)
		t.Logf("  %10s %12s %12s %10s %12s", "accelNoise", "pos RMSE", "pos max", "NEES", "optimistic")
		for _, a := range accels {
			fc := localization.DefaultConfig()
			fc.Noise.AccelNoise = a
			rep, err := Evaluate(sc.name, s, fc, opts)
			if err != nil {
				t.Fatal(err)
			}
			t.Logf("  %10.1f %9.2f mm %9.2f mm %10.2f %12d",
				a, rep.Fused.PositionRMSEm*1000, rep.Fused.PositionMaxM*1000,
				rep.Consistency.MeanNEES, rep.Consistency.Optimistic)
		}
	}
}
