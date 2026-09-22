package locsim

import (
	"fmt"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/localization"
)

// スリップのプロセス雑音と時定数を掃引して既定値を選ぶ (2026-09-13 の決定)。
//
// 見るのは 2 つ:
//
//	RMSE  スリップが無いときに真の速度を吸い取っていないか
//	NEES  スリップがあるときに過信していないか
//
// **過信 (NEES が大きい) のほうが危険である。** 位置制御は共分散を見て
// ゲインを決めるので、ずれているのに自信満々な状態が一番事故る。
func TestSlipParameterSweep(t *testing.T) {
	tr := Line(localization.Vec2{}, localization.Vec2{X: 2, Y: 0}, localization.Vec2{},
		HeadingParams{Mode: HeadingFixed}, 10*time.Second)

	type scenario struct {
		name string
		cfg  Config
	}
	scs := []scenario{
		{"no slip", DefaultConfig()},
		{"burst 0.5 m/s 150 ms", func() Config {
			c := DefaultConfig()
			c.Slip = BurstSlip(localization.Vec2{X: 0.5, Y: -0.3},
				3*time.Second, 3*time.Second+150*time.Millisecond)
			return c
		}()},
		{"burst + 1 s outage", func() Config {
			c := DefaultConfig()
			c.Slip = BurstSlip(localization.Vec2{X: 0.5, Y: -0.3},
				3*time.Second, 3*time.Second+150*time.Millisecond)
			c.VisionOutages = []Outage{{Start: 3 * time.Second, End: 4 * time.Second}}
			return c
		}()},
		{"constant 0.2 m/s + 2 s outage", func() Config {
			c := DefaultConfig()
			c.Slip = ConstantSlip(localization.Vec2{X: 0.2, Y: 0})
			c.VisionOutages = []Outage{{Start: 4 * time.Second, End: 6 * time.Second}}
			return c
		}()},
	}

	type setting struct {
		label string
		apply func(*localization.NoiseConfig)
	}
	settings := []setting{
		{"OFF", func(n *localization.NoiseConfig) { n.EnableSlip = false }},
	}
	for _, tau := range []float64{0.05, 0.1, 0.3} {
		for _, q := range []float64{0.1, 0.3, 0.5, 1.0} {
			tau, q := tau, q
			settings = append(settings, setting{
				label: fmt.Sprintf("tau %.2f q %.1f", tau, q),
				apply: func(n *localization.NoiseConfig) {
					n.EnableSlip = true
					n.SlipTau = tau
					n.SlipNoise = q
				},
			})
		}
	}

	// scenario -> setting -> 結果
	for _, sc := range scs {
		s, err := Generate(tr, sc.cfg, 11)
		if err != nil {
			t.Fatal(err)
		}
		t.Logf("--- %s ---", sc.name)
		for _, st := range settings {
			cfg := filterConfig()
			st.apply(&cfg.Noise)
			rep, err := Evaluate(sc.name, s, cfg, calibratedOpts(s))
			if err != nil {
				t.Fatal(err)
			}
			t.Logf("  %-14s rmse %7.2f mm  max %7.2f  NEES %8.2f  worst-case optimistic %d",
				st.label, rep.Fused.PositionRMSEm*1000, rep.Fused.PositionMaxM*1000,
				rep.Consistency.MeanNEES, rep.Consistency.Optimistic)
		}
	}
}
