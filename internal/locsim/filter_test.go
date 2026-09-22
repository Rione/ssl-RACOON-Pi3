package locsim

import (
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/localization"
)

// フィルタの効果を測る。合格条件は 2 段階 (2026-09-13 合意):
//
//	ハード  vision 生値より悪ければ失敗。改善していなければ CI が落ちる。
//	ソフト  改善率はログに出して推移を見る。合成データの数字を実機の保証と
//	        取り違えないため、ここは合格条件にしない。

func filterConfig() localization.Config {
	cfg := localization.DefaultConfig()
	// センサ合成側と同じ幾何を使う。**符号やホイール順序を取り違えた場合は
	// 別のテストで扱う。**
	cfg.Geometry = DefaultConfig().Geometry
	return cfg
}

// calibratedOpts は「片道遅延の定数分を実測済み」の設定を返す。
//
// timesync は原理的にこの定数を分離できないので、オフラインで測って与えるしかない
// (計画 §5.3 / §12-D3)。**これが入っていないと、位置誤差が速度に比例して残り、
// しかも共分散はそれを一切表現できない。**
func calibratedOpts(s *Sensors) localization.EstimatorOptions {
	return localization.EstimatorOptions{VisionDelayComp: s.Config.VisionTimeBias}
}

func scenarios() []struct {
	name string
	tr   Trajectory
	cfg  Config
} {
	base := DefaultConfig()

	outage := base
	outage.VisionOutages = []Outage{{Start: 4 * time.Second, End: 4*time.Second + 500*time.Millisecond}}

	longOutage := base
	longOutage.VisionOutages = []Outage{{Start: 4 * time.Second, End: 6 * time.Second}}

	slip := base
	slip.Slip = BurstSlip(localization.Vec2{X: 0.5, Y: -0.3},
		3*time.Second, 3*time.Second+150*time.Millisecond)

	return []struct {
		name string
		tr   Trajectory
		cfg  Config
	}{
		{"straight 2 m/s", Line(localization.Vec2{}, localization.Vec2{X: 2, Y: 0},
			localization.Vec2{}, HeadingParams{Mode: HeadingFixed}, 10*time.Second), base},
		{"lateral 1.5 m/s (fixed heading)", Line(localization.Vec2{}, localization.Vec2{X: 0, Y: 1.5},
			localization.Vec2{}, HeadingParams{Mode: HeadingFixed}, 10*time.Second), base},
		{"circle 2 m/s tangent", Circle(localization.Vec2{}, 1.5, 2.0,
			HeadingParams{Mode: HeadingTangent}, 10*time.Second), base},
		{"circle + spin", Circle(localization.Vec2{}, 1.5, 2.0,
			HeadingParams{Mode: HeadingSpin, SpinRate: 3.0}, 10*time.Second), base},
		{"figure eight", FigureEight(localization.Vec2{}, 1.5, 4*time.Second,
			HeadingParams{Mode: HeadingSpin, SpinRate: 2.0}, 12*time.Second), base},
		{"hard accel and stop", StepAccel(localization.Vec2{}, 0.3, 4.0, 2.5,
			2*time.Second, HeadingParams{Mode: HeadingFixed}, 10*time.Second), base},
		{"stationary", Stationary(localization.Vec2{X: 1, Y: 1},
			HeadingParams{Mode: HeadingFixed, Theta0: 0.4}, 10*time.Second), base},
		{"vision outage 0.5 s", Line(localization.Vec2{}, localization.Vec2{X: 2, Y: 0},
			localization.Vec2{}, HeadingParams{Mode: HeadingFixed}, 10*time.Second), outage},
		{"vision outage 2.0 s", Line(localization.Vec2{}, localization.Vec2{X: 2, Y: 0},
			localization.Vec2{}, HeadingParams{Mode: HeadingFixed}, 10*time.Second), longOutage},
		{"slip burst", Line(localization.Vec2{}, localization.Vec2{X: 2, Y: 0},
			localization.Vec2{}, HeadingParams{Mode: HeadingFixed}, 10*time.Second), slip},
	}
}

// **本命の回帰テスト。** vision 生値より悪ければ失敗する。
func TestFilterBeatsRawVision(t *testing.T) {
	for _, sc := range scenarios() {
		t.Run(sc.name, func(t *testing.T) {
			s, err := Generate(sc.tr, sc.cfg, 1)
			if err != nil {
				t.Fatal(err)
			}
			rep, err := Evaluate(sc.name, s, filterConfig(), calibratedOpts(s))
			if err != nil {
				t.Fatal(err)
			}
			t.Log("\n" + rep.String())

			if rep.Fused.PositionRMSEm > rep.Raw.PositionRMSEm {
				t.Errorf("fused position rmse %.2f mm is WORSE than raw vision %.2f mm",
					rep.Fused.PositionRMSEm*1000, rep.Raw.PositionRMSEm*1000)
			}
			if rep.Fused.HeadingRMSErad > rep.Raw.HeadingRMSErad {
				t.Errorf("fused heading rmse %.4f deg is WORSE than raw vision %.4f deg",
					rep.Fused.HeadingRMSErad*57.2958, rep.Raw.HeadingRMSErad*57.2958)
			}
		})
	}
}

// **符号を取り違えたら精度が悪化することを固定する。**
//
// もしフィルタが符号の誤りを検出できないなら、それ自体が重大な発見である
// (= 実機で符号が逆でも気づけない)。
func TestWrongSignDegradesFilter(t *testing.T) {
	tr := FigureEight(localization.Vec2{}, 1.5, 4*time.Second,
		HeadingParams{Mode: HeadingSpin, SpinRate: 2.0}, 12*time.Second)
	s, err := Generate(tr, DefaultConfig(), 2)
	if err != nil {
		t.Fatal(err)
	}

	good, err := Evaluate("correct geometry", s, filterConfig(), calibratedOpts(s))
	if err != nil {
		t.Fatal(err)
	}
	t.Log("\n" + good.String())

	cases := []struct {
		name   string
		mutate func(*localization.GeometryConfig)
	}{
		{"sign flipped (A-5)", func(g *localization.GeometryConfig) {
			for i := range g.WheelSigns {
				g.WheelSigns[i] = -1
			}
		}},
		{"FL/FR swapped (A-4)", func(g *localization.GeometryConfig) {
			g.WheelSlotOrder = [localization.NumWheels]int{
				localization.WheelFR, localization.WheelBL, localization.WheelBR, localization.WheelFL,
			}
		}},
	}

	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			cfg := filterConfig()
			c.mutate(&cfg.Geometry)
			bad, err := Evaluate(c.name, s, cfg, calibratedOpts(s))
			if err != nil {
				t.Fatal(err)
			}
			t.Log("\n" + bad.String())

			if bad.Fused.PositionRMSEm <= good.Fused.PositionRMSEm*2 {
				t.Errorf("a wrong geometry gave %.2f mm vs %.2f mm with the correct one; "+
					"the filter cannot tell the difference, so this mistake would be invisible on the robot",
					bad.Fused.PositionRMSEm*1000, good.Fused.PositionRMSEm*1000)
			}
			// イノベーションにも出るべき。Huber が働いた回数で見る。
			t.Logf("innovation health: correct %+v", good.Consistency)
		})
	}
}

// スリップ状態を入れた場合と入れない場合を同じログで比べる。
// **既定値はこの数字を見てから決める** (2026-09-13 の決定)。
func TestSlipStateComparison(t *testing.T) {
	scs := []struct {
		name string
		cfg  Config
	}{
		{"no slip injected", DefaultConfig()},
		{"slip burst 0.5 m/s for 150 ms", func() Config {
			c := DefaultConfig()
			c.Slip = BurstSlip(localization.Vec2{X: 0.5, Y: -0.3},
				3*time.Second, 3*time.Second+150*time.Millisecond)
			return c
		}()},
		{"slip burst + vision outage", func() Config {
			c := DefaultConfig()
			c.Slip = BurstSlip(localization.Vec2{X: 0.5, Y: -0.3},
				3*time.Second, 3*time.Second+150*time.Millisecond)
			c.VisionOutages = []Outage{{Start: 3 * time.Second, End: 4 * time.Second}}
			return c
		}()},
		{"constant slip 0.2 m/s + vision outage", func() Config {
			c := DefaultConfig()
			c.Slip = ConstantSlip(localization.Vec2{X: 0.2, Y: 0})
			c.VisionOutages = []Outage{{Start: 4 * time.Second, End: 6 * time.Second}}
			return c
		}()},
	}

	tr := Line(localization.Vec2{}, localization.Vec2{X: 2, Y: 0}, localization.Vec2{},
		HeadingParams{Mode: HeadingFixed}, 10*time.Second)

	for _, sc := range scs {
		s, err := Generate(tr, sc.cfg, 3)
		if err != nil {
			t.Fatal(err)
		}
		for _, enable := range []bool{false, true} {
			cfg := filterConfig()
			cfg.Noise.EnableSlip = enable
			rep, err := Evaluate(sc.name, s, cfg, calibratedOpts(s))
			if err != nil {
				t.Fatal(err)
			}
			label := "slip OFF"
			if enable {
				label = "slip ON "
			}
			t.Logf("%-38s %s | pos rmse %6.2f mm (max %7.2f) | improvement %+5.1f%% | NEES %.2f",
				sc.name, label, rep.Fused.PositionRMSEm*1000, rep.Fused.PositionMaxM*1000,
				rep.Improvement*100, rep.Consistency.MeanNEES)
		}
	}
}

// vision が長時間落ちても発散しないこと (計画 §9 の検証項目 8)。
func TestFilterSurvivesLongVisionOutage(t *testing.T) {
	cfg := DefaultConfig()
	cfg.VisionOutages = []Outage{{Start: 2 * time.Second, End: 7 * time.Second}}
	tr := Line(localization.Vec2{}, localization.Vec2{X: 2, Y: 0}, localization.Vec2{},
		HeadingParams{Mode: HeadingFixed}, 10*time.Second)
	s, err := Generate(tr, cfg, 4)
	if err != nil {
		t.Fatal(err)
	}
	est, estimator, err := RunFilter(s, filterConfig(), calibratedOpts(s))
	if err != nil {
		t.Fatal(err)
	}

	var sawDegraded bool
	for _, e := range est {
		if e.Health == localization.HealthInvalid {
			t.Fatalf("the filter reported INVALID at %v", time.Duration(e.Stamp))
		}
		if e.Health == localization.HealthDegraded {
			sawDegraded = true
		}
		if hasNaNEstimate(e) {
			t.Fatalf("NaN in the estimate at %v", time.Duration(e.Stamp))
		}
	}
	if !sawDegraded {
		t.Error("a 5 second vision outage never raised DEGRADED; RAVEN would not know to fall back")
	}
	if s := estimator.Stats(); s.Resets != 0 {
		t.Errorf("the filter reset %d times during a plain outage", s.Resets)
	}
	t.Logf("stats: %+v", estimator.Stats())
}

func hasNaNEstimate(e localization.Estimate) bool {
	v := []float64{e.Pose.X, e.Pose.Y, e.Pose.Theta, e.VelBody.X, e.VelBody.Y, e.YawRate}
	for _, x := range v {
		if x != x {
			return true
		}
	}
	return false
}

// 同じ入力からは必ず同じ出力になること (計画 §7.4)。
func TestFilterIsDeterministic(t *testing.T) {
	s, err := Generate(FigureEight(localization.Vec2{}, 1.5, 4*time.Second,
		HeadingParams{Mode: HeadingSpin, SpinRate: 2.0}, 8*time.Second), DefaultConfig(), 5)
	if err != nil {
		t.Fatal(err)
	}
	a, _, err := RunFilter(s, filterConfig(), calibratedOpts(s))
	if err != nil {
		t.Fatal(err)
	}
	b, _, err := RunFilter(s, filterConfig(), calibratedOpts(s))
	if err != nil {
		t.Fatal(err)
	}
	if len(a) != len(b) {
		t.Fatalf("lengths differ: %d vs %d", len(a), len(b))
	}
	for i := range a {
		if a[i] != b[i] {
			t.Fatalf("estimate %d differs:\n  %+v\n  %+v", i, a[i], b[i])
		}
	}
}

// 未較正の定数遅延が、速度域で残る誤差のほぼ全部であることを固定する。
//
// **この事実が D-3 (片道遅延の実測) を「あると良い」から「最優先」へ引き上げる。**
func TestUncalibratedVisionDelayDominatesError(t *testing.T) {
	var calMin, calMax float64
	for _, speed := range []float64{0.5, 1.0, 2.0, 3.0} {
		tr := Line(localization.Vec2{}, localization.Vec2{X: speed, Y: 0}, localization.Vec2{},
			HeadingParams{Mode: HeadingFixed}, 10*time.Second)
		s, err := Generate(tr, DefaultConfig(), 7)
		if err != nil {
			t.Fatal(err)
		}

		cal, err := Evaluate("calibrated", s, filterConfig(), calibratedOpts(s))
		if err != nil {
			t.Fatal(err)
		}
		unc, err := Evaluate("uncalibrated", s, filterConfig(), localization.EstimatorOptions{})
		if err != nil {
			t.Fatal(err)
		}

		predicted := speed * s.Config.VisionTimeBias.Seconds() * 1000
		t.Logf("%.1f m/s: calibrated %5.2f mm (NEES %6.2f) | uncalibrated %6.2f mm (NEES %7.2f) | delay bias predicts %5.2f mm",
			speed, cal.Fused.PositionRMSEm*1000, cal.Consistency.MeanNEES,
			unc.Fused.PositionRMSEm*1000, unc.Consistency.MeanNEES, predicted)

		// 未較正の誤差は、遅延バイアスの予測値で説明しきれること。
		got := unc.Fused.PositionRMSEm * 1000
		if got < predicted*0.8 || got > predicted*1.3 {
			t.Errorf("%.1f m/s: uncalibrated error %.2f mm does not match the %.2f mm predicted by the delay bias; "+
				"something other than the constant delay is contributing", speed, got, predicted)
		}

		c := cal.Fused.PositionRMSEm * 1000
		if calMin == 0 || c < calMin {
			calMin = c
		}
		if c > calMax {
			calMax = c
		}
	}

	// **これが主張の核心。** 較正すると誤差が速度に依存しなくなる。
	//
	// 未較正では誤差が速度に比例して増える (0.5 m/s で 10 mm、3 m/s で 60 mm)。
	// 定数遅延を引けば、残るのは速度に依らないフィルタ本来の下限だけになる。
	if calMax > calMin*1.3 {
		t.Errorf("calibrated error still varies with speed (%.2f to %.2f mm); "+
			"a speed-dependent term other than the constant delay remains", calMin, calMax)
	}
	t.Logf("calibrated error is speed-independent: %.2f to %.2f mm across 0.5-3.0 m/s", calMin, calMax)
}

// 計画 §10 の精度目標に対する現在位置。**合成データでの値であり、実機の保証ではない。**
func TestAccuracyTargetsOnSyntheticData(t *testing.T) {
	tr := Line(localization.Vec2{}, localization.Vec2{X: 2, Y: 0}, localization.Vec2{},
		HeadingParams{Mode: HeadingFixed}, 10*time.Second)
	s, err := Generate(tr, DefaultConfig(), 8)
	if err != nil {
		t.Fatal(err)
	}
	rep, err := Evaluate("2 m/s straight", s, filterConfig(), calibratedOpts(s))
	if err != nil {
		t.Fatal(err)
	}
	t.Log("\n" + rep.String())

	// 10-1: 位置 RMSE <= 10 mm、角度 RMSE <= 1.0 度
	if rep.Fused.PositionRMSEm*1000 > 10 {
		t.Errorf("10-1: position rmse %.2f mm exceeds the 10 mm target", rep.Fused.PositionRMSEm*1000)
	}
	if rep.Fused.HeadingRMSErad*57.2958 > 1.0 {
		t.Errorf("10-1: heading rmse %.3f deg exceeds the 1.0 deg target", rep.Fused.HeadingRMSErad*57.2958)
	}
	// 10-2: vision 生値に対して 50% 以上の削減
	if rep.Improvement < 0.5 {
		t.Errorf("10-2: improvement %+.1f%% falls short of the 50%% target", rep.Improvement*100)
	}
	// 10-8: NEES。
	//
	// **過信 (Optimistic) だけをハードな合格条件にする。** 位置制御は共分散を見て
	// ゲインを落とすので、「ずれているのに自信満々」が唯一の危険な向きである。
	// 弱気側 (Pessimistic) は制御が慎重になるだけで事故らない。
	optimisticRatio := float64(rep.Consistency.Optimistic) / float64(rep.Consistency.Samples)
	if optimisticRatio > 0.01 {
		t.Errorf("10-8: %.1f%% of samples are overconfident; the controller would trust a wrong estimate",
			optimisticRatio*100)
	}
	// 両側の 90% は現状**未達**。Q が未チューニングで保守的 (NEES の平均が
	// 3.0 より小さい) ため、外れるぶんはすべて弱気側に出る。
	// 合成データの雑音に Q を合わせ込むのは禁じてあるので、ここは実測待ちである。
	if r := rep.Consistency.Ratio(); r < 0.90 {
		t.Logf("10-8 NOT MET (known gap): %.1f%% in the 95%% interval, target 90%%. "+
			"mean NEES %.2f < 3.0 means Q is conservative; all misses are on the safe side "+
			"(%d pessimistic, %d optimistic). Retune once the real noise is measured.",
			r*100, rep.Consistency.MeanNEES, rep.Consistency.Pessimistic, rep.Consistency.Optimistic)
	}
}
