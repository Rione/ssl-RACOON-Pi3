package locsim

import (
	"math"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// scaledRadius は真値の車輪半径だけを factor 倍した設定を返す。
//
// **これがオンライン較正の狙いどころ**である。車輪半径は機体ごと・摩耗ごとに
// 違い、出典によって 26 / 27 / 28.6 / 30 mm と 15% も割れている (計画 §3.3)。
func scaledRadius(g localization.GeometryConfig, factor float64) localization.GeometryConfig {
	for i := range g.WheelRadiusM {
		g.WheelRadiusM[i] *= factor
	}
	return g
}

// **車輪半径の食い違いをオンラインで吸収できることを固定する。**
//
// 真値の半径を公称より 6% 大きくし、フィルタには公称値を持たせる。
// 倍率 Kv は 1/1.06 = 0.943 へ、Kw も同じ側へ寄るはずで、vision が切れた
// ときの誤差がはっきり減ること。
//
// Mozzarelli ほか (arXiv:2403.13452) が車輪半径を状態にして、130 秒の位置喪失で
// 累積誤差を 5.3 m -> 0.35 m にしたのと同じ仕掛けである。
func TestParameterEstimationAbsorbsWheelRadiusError(t *testing.T) {
	cfg := DefaultConfig()
	const factor = 1.06
	cfg.Geometry = scaledRadius(localization.DefaultGeometry(), factor)
	cfg.VisionOutages = []Outage{{Start: 8 * time.Second, End: 10 * time.Second}}

	// 較正には並進・横・回転すべての加振が要る。
	tr := FigureEight(localization.Vec2{}, 1.2, 4*time.Second,
		HeadingParams{Mode: HeadingSpin, SpinRate: 2.0}, 12*time.Second)
	s, err := Generate(tr, cfg, 21)
	if err != nil {
		t.Fatal(err)
	}

	base := localization.DefaultConfig() // 公称の半径を持つ = 6% 小さい
	opts := localization.EstimatorOptions{VisionDelayComp: s.Config.VisionTimeBias}

	off := base
	off.Noise.EnableParamEstimation = false
	estOff, _, err := RunFilter(s, off, opts)
	if err != nil {
		t.Fatal(err)
	}

	on := base
	on.Noise.EnableParamEstimation = true
	estOn, filter, err := RunFilter(s, on, opts)
	if err != nil {
		t.Fatal(err)
	}

	errOff := outageError(t, s, estOff, cfg.VisionOutages[0])
	errOn := outageError(t, s, estOn, cfg.VisionOutages[0])
	final := estOn[len(estOn)-1].Params

	t.Logf("converged scale: trans %.4f (expected ~%.4f), rot %.4f, angle %+.2f deg",
		final.TransScale, 1/factor, final.RotScale, final.AngleBias*180/math.Pi)
	t.Logf("vision outage worst error: params off %.1f mm, params on %.1f mm", errOff*1000, errOn*1000)
	t.Logf("slip rate %.3f (geometry error indicator), stats %+v", filter.SlipRate(), filter.Stats())

	if errOn > errOff*0.7 {
		t.Errorf("parameter estimation did not help during the vision outage: %.1f mm -> %.1f mm",
			errOff*1000, errOn*1000)
	}
	if math.Abs(final.TransScale-1/factor) > 0.03 {
		t.Errorf("translation scale converged to %.4f, want %.4f +-0.03", final.TransScale, 1/factor)
	}
}

// **既知の限界: 輪ごとに取付角が違う食い違いは吸収できない。**
//
// CAD (60/135/-135/-60) と PoC の同定値 (55.4/136.1/-136.3/-57.4) の差は
// +4.6 / -1.1 / +1.3 / -2.6 度で、**符号が揃っていない**。共通の角度補正
// 1 つでは表せず、倍率でも表せない。最小二乗で当てはめても車輪残差は
// 1.40 -> 1.29 rad/s にしか下がらない (輪ごとの角度を自由にすれば厳密に 0)。
//
// **だから CAD と同定値の食い違いはオンライン較正では解決しない。**
// 研究 §2.5 の計測 (車輪ログだけで角度比を出す) で決めるしかない。
// このテストはその限界を固定して、後から「効かないのはバグでは」と
// 疑われないようにするためにある。
func TestParameterEstimationCannotAbsorbPerWheelAngleError(t *testing.T) {
	cfg := DefaultConfig()
	cfg.Geometry = localization.CADGeometry()
	cfg.VisionOutages = []Outage{{Start: 8 * time.Second, End: 10 * time.Second}}

	tr := FigureEight(localization.Vec2{}, 1.2, 4*time.Second,
		HeadingParams{Mode: HeadingSpin, SpinRate: 2.0}, 12*time.Second)
	s, err := Generate(tr, cfg, 21)
	if err != nil {
		t.Fatal(err)
	}

	base := localization.DefaultConfig() // 同定値 = 輪ごとに角度が違う
	opts := localization.EstimatorOptions{VisionDelayComp: s.Config.VisionTimeBias}

	on := base
	on.Noise.EnableParamEstimation = true
	est, filter, err := RunFilter(s, on, opts)
	if err != nil {
		t.Fatal(err)
	}
	errOn := outageError(t, s, est, cfg.VisionOutages[0])
	t.Logf("per-wheel angle mismatch: outage error %.0f mm even with online calibration", errOn*1000)
	t.Logf("slip rate %.3f, residual bias (see below)", filter.SlipRate())

	// 周期ごとの検定では捕まらない。幾何の誤りは**平均**に出る系統誤差で、
	// 1 周期あたりの大きさは速度に比例する車輪雑音に埋もれるためである。
	bias, ok := filter.WheelResidualBias()
	if !ok {
		t.Fatal("residual bias was not available")
	}
	t.Logf("residual bias %.4f rad/s (zero if the geometry is right)", bias)

	// **正しい捕まえ方はこちら。** 車輪ログから左零ベクトルを当てはめ直し、
	// 設定の値と比べる (研究 §2.5)。vision も時刻合わせも使わない。
	logical := make([][]float64, 0)
	_ = logical
	samples := wheelLogLogical(s)
	fit, err := localization.FitNullVector(samples)
	if err != nil {
		t.Fatal(err)
	}
	cadN := normalizeNull(localization.ClosedFormNullVector(localization.CADGeometry()))
	cfgN := normalizeNull(localization.ClosedFormNullVector(localization.DefaultGeometry()))
	fitN := normalizeNull(fit.N)

	dCAD := nullDistance(fitN, cadN)
	dCfg := nullDistance(fitN, cfgN)
	t.Logf("fitted null vector %v (identifiability %.1f, residual %.3f rad/s)",
		roundVec(fitN), fit.Identifiability, fit.ResidualRMS)
	t.Logf("distance to CAD geometry        : %.4f", dCAD)
	t.Logf("distance to configured geometry : %.4f", dCfg)

	if dCAD >= dCfg {
		t.Errorf("the wheel-only fit did not point at the true (CAD) geometry: "+
			"distance to CAD %.4f vs to configured %.4f", dCAD, dCfg)
	}
}

// wheelLogLogical はセンサ列の車輪を論理輪番号の順に並べ直して返す。
func wheelLogLogical(s *Sensors) [][localization.NumWheels]float64 {
	out := make([][localization.NumWheels]float64, 0, len(s.Wheels))
	for _, w := range s.Wheels {
		var v [localization.NumWheels]float64
		for slot := 0; slot < localization.NumWheels; slot++ {
			v[s.Config.Geometry.WheelSlotOrder[slot]] = w.Omega[slot]
		}
		out = append(out, v)
	}
	return out
}

func normalizeNull(v [localization.NumWheels]float64) [localization.NumWheels]float64 {
	maxAbs := 0.0
	for _, x := range v {
		if a := math.Abs(x); a > maxAbs {
			maxAbs = a
		}
	}
	if maxAbs == 0 {
		return v
	}
	var out [localization.NumWheels]float64
	for i, x := range v {
		out[i] = x / maxAbs
	}
	for i := range out {
		if math.Abs(out[i]) > 1e-9 {
			if out[i] < 0 {
				for j := range out {
					out[j] = -out[j]
				}
			}
			break
		}
	}
	return out
}

func nullDistance(a, b [localization.NumWheels]float64) float64 {
	var d float64
	for i := range a {
		d += (a[i] - b[i]) * (a[i] - b[i])
	}
	return math.Sqrt(d)
}

func roundVec(v [localization.NumWheels]float64) []float64 {
	out := make([]float64, len(v))
	for i, x := range v {
		out[i] = math.Round(x*10000) / 10000
	}
	return out
}

// outageError は vision 欠落区間のあいだの最悪の位置誤差を返す。
func outageError(t *testing.T, s *Sensors, est []localization.Estimate, o Outage) float64 {
	t.Helper()
	target := localization.Stamp(o.End) - localization.Stamp(2*time.Millisecond)
	var worst float64
	for _, e := range est {
		if e.Stamp < localization.Stamp(o.Start) || e.Stamp > target {
			continue
		}
		truth, ok := s.TruthAt(e.Stamp)
		if !ok {
			continue
		}
		d := math.Hypot(e.Pose.X-truth.Pose.X, e.Pose.Y-truth.Pose.Y)
		if d > worst {
			worst = d
		}
	}
	return worst
}

// vision が切れているあいだは倍率が凍結されること。
//
// 車輪だけでは速度と倍率が積としてしか現れず不可観測なので、そこで
// 補正を許すと公称値が不可観測な尾根に沿って流れる (Mozzarelli ほか)。
func TestParametersFreezeWithoutVision(t *testing.T) {
	cfg := DefaultConfig()
	cfg.Geometry = scaledRadius(localization.DefaultGeometry(), 1.06)
	cfg.VisionOutages = []Outage{{Start: 5 * time.Second, End: 9 * time.Second}}

	tr := FigureEight(localization.Vec2{}, 1.2, 4*time.Second,
		HeadingParams{Mode: HeadingSpin, SpinRate: 2.0}, 10*time.Second)
	s, err := Generate(tr, cfg, 22)
	if err != nil {
		t.Fatal(err)
	}
	est, _, err := RunFilter(s, localization.DefaultConfig(),
		localization.EstimatorOptions{VisionDelayComp: s.Config.VisionTimeBias})
	if err != nil {
		t.Fatal(err)
	}

	// 凍結は ParamFreezeAfter (既定 150 ms) 経ってから効く。その手前は
	// まだ vision が新しい扱いなので、比較はそこを過ぎてから始める。
	const frozenFrom = 5300 * time.Millisecond
	var atStart, atEnd localization.KinematicParams
	var sawFrozen bool
	for _, e := range est {
		if e.Stamp <= localization.Stamp(frozenFrom) {
			atStart = e.Params
		}
		if e.Stamp <= localization.Stamp(9*time.Second) {
			atEnd = e.Params
		}
		if e.Params.Frozen && e.Stamp > localization.Stamp(frozenFrom) &&
			e.Stamp < localization.Stamp(9*time.Second) {
			sawFrozen = true
		}
	}
	if !sawFrozen {
		t.Fatal("parameters were never reported as frozen during a 4 s vision outage")
	}
	if d := math.Abs(atEnd.TransScale - atStart.TransScale); d > 1e-9 {
		t.Errorf("translation scale moved by %.3g while vision was gone; it must stay frozen", d)
	}
	if d := math.Abs(atEnd.RotScale - atStart.RotScale); d > 1e-9 {
		t.Errorf("rotation scale moved by %.3g while vision was gone", d)
	}
	t.Logf("frozen across the outage at trans %.4f / rot %.4f", atEnd.TransScale, atEnd.RotScale)
}
