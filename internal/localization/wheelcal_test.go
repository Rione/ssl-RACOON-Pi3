package localization

import (
	"math"
	"math/rand"
	"testing"
)

// excitedWheelLog は 3 自由度を加振した車輪ログを合成する。
func excitedWheelLog(t *testing.T, g GeometryConfig, n int, sigma float64, seed int64) [][NumWheels]float64 {
	t.Helper()
	k, err := NewKinematics(g)
	if err != nil {
		t.Fatalf("NewKinematics: %v", err)
	}
	rng := rand.New(rand.NewSource(seed))
	out := make([][NumWheels]float64, n)
	for i := 0; i < n; i++ {
		ph := float64(i) * 0.02
		// 3 軸を別々の周波数で振る (どれか 1 軸だけだと零方向が決まらない)。
		vx := 0.6 * math.Sin(ph)
		vy := 0.5 * math.Sin(ph*0.61+1.1)
		w := 3.0 * math.Sin(ph*0.37+2.3)
		wheels := k.WheelFromBody(vx, vy, w)
		for j := range wheels {
			wheels[j] += rng.NormFloat64() * sigma
		}
		out[i] = wheels
	}
	return out
}

// 車輪のログだけから左零ベクトルが復元できる。
func TestFitNullVectorRecoversGeometry(t *testing.T) {
	for name, g := range map[string]GeometryConfig{
		"cad60":    cadGeometry(),
		"stm55":    stmGeometry(),
		"unequalR": unequalRadiusGeometry(),
	} {
		samples := excitedWheelLog(t, g, 3000, 0.02, 11)
		fit, err := FitNullVector(samples)
		if err != nil {
			t.Fatalf("%s: %v", name, err)
		}
		want := normalizeMaxAbs(ClosedFormNullVector(g))
		got := normalizeMaxAbs(fit.N)
		for i := 0; i < NumWheels; i++ {
			if math.Abs(got[i]-want[i]) > 0.01 {
				t.Fatalf("%s: null vector %v, want %v (identifiability %.1f)", name, got, want, fit.Identifiability)
			}
		}
		if fit.Identifiability < 5 {
			t.Fatalf("%s: identifiability %.2f is too low for a well excited log", name, fit.Identifiability)
		}
		if fit.MeasuredWheelNoise < 0.016 || fit.MeasuredWheelNoise > 0.025 {
			t.Fatalf("%s: measured wheel noise %.4f rad/s, want ~0.020", name, fit.MeasuredWheelNoise)
		}
	}
}

// **55 度説と 60 度説を車輪ログだけで区別できる。**
// これが docs/self-localization-research-20260923.md §2.5 の計測の中身。
func TestWheelLogDistinguishes55From60(t *testing.T) {
	cases := []struct {
		name string
		g    GeometryConfig
		want float64
	}{
		{"cad 60deg", cadGeometry(), 60},
		{"stm 55deg", stmGeometry(), 55},
	}
	for _, c := range cases {
		samples := excitedWheelLog(t, c.g, 3000, 0.02, 23)
		fit, err := FitNullVector(samples)
		if err != nil {
			t.Fatalf("%s: %v", c.name, err)
		}
		ang, err := AnglesFromNullVector(fit.N, c.g.WheelSigns, 135)
		if err != nil {
			t.Fatalf("%s: %v", c.name, err)
		}
		if math.Abs(ang.PhiDeg-c.want) > 0.5 {
			t.Fatalf("%s: phi = %.2f deg, want %.1f", c.name, ang.PhiDeg, c.want)
		}
		if math.Abs(ang.FrontImbalance) > 0.02 || math.Abs(ang.RearImbalance) > 0.02 {
			t.Fatalf("%s: unexpected imbalance front=%.3f rear=%.3f", c.name, ang.FrontImbalance, ang.RearImbalance)
		}
		t.Logf("%s -> phi=%.2f deg, |n_FL|=%.4f (predicted %.4f), residual %.4f rad/s",
			c.name, ang.PhiDeg, math.Abs(normalizeMaxAbs(fit.N)[WheelFL]),
			PredictedNullVectorMagnitude(c.want, 135), fit.ResidualRMS)
	}
}

// 加振が足りないと零方向が決まらないことを、Identifiability が知らせる。
func TestFitNullVectorReportsPoorExcitation(t *testing.T) {
	g := cadGeometry()
	k, err := NewKinematics(g)
	if err != nil {
		t.Fatal(err)
	}
	rng := rand.New(rand.NewSource(5))
	samples := make([][NumWheels]float64, 3000)
	for i := range samples {
		// 前進しかしていないログ。
		wheels := k.WheelFromBody(0.5*math.Sin(float64(i)*0.02), 0, 0)
		for j := range wheels {
			wheels[j] += rng.NormFloat64() * 0.02
		}
		samples[i] = wheels
	}
	fit, err := FitNullVector(samples)
	if err != nil {
		t.Fatal(err)
	}
	if fit.Identifiability > 5 {
		t.Fatalf("straight-line-only log reported identifiability %.2f; it should be low", fit.Identifiability)
	}
	t.Logf("straight-line only: identifiability %.2f (eigenvalues %v)", fit.Identifiability, fit.Eigenvalues)
}

// 車輪ログから決まるのは **sinθ/sinφ の比**であって、角度そのものではない。
// θ を 135 度と仮定して読むと、実機の θ が 136.2 度なら φ は 2 度ほど大きく出る。
// この感度を明示しておく (docs/self-localization-research-20260923.md §2.5 の限界)。
func TestOnlyTheAngleRatioIsIdentifiable(t *testing.T) {
	g := identifiedGeometry() // 55.4 / 136.1 / -136.3 / -57.4
	samples := excitedWheelLog(t, g, 4000, 0.02, 31)
	fit, err := FitNullVector(samples)
	if err != nil {
		t.Fatal(err)
	}

	// 真の比。前後とも左右平均を代表値にする。
	phiTrue := 0.5 * (g.WheelAnglesDeg[WheelFL] - g.WheelAnglesDeg[WheelFR])
	thetaTrue := 0.5 * (g.WheelAnglesDeg[WheelBL] - g.WheelAnglesDeg[WheelBR])

	// 正しい θ を与えれば φ が復元できる。
	ang, err := AnglesFromNullVector(fit.N, g.WheelSigns, thetaTrue)
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(ang.PhiDeg-phiTrue) > 0.5 {
		t.Fatalf("with the true theta=%.2f, phi=%.2f deg, want %.2f", thetaTrue, ang.PhiDeg, phiTrue)
	}

	// θ を 135 度と誤って仮定すると φ がずれる。ずれの向きと大きさを固定しておく。
	wrong, err := AnglesFromNullVector(fit.N, g.WheelSigns, 135)
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(wrong.PhiDeg-ang.PhiDeg) < 1.0 {
		t.Fatalf("assuming theta=135 should move phi by more than 1 deg, got %.2f -> %.2f", ang.PhiDeg, wrong.PhiDeg)
	}
	// それでも CAD の 60 度とははっきり分かれる。
	if math.Abs(wrong.PhiDeg-60) < 1.0 {
		t.Fatalf("identified geometry is indistinguishable from 60 deg (phi=%.2f)", wrong.PhiDeg)
	}
	t.Logf("identified geometry: theta=%.2f -> phi=%.2f;  assuming theta=135 -> phi=%.2f",
		thetaTrue, ang.PhiDeg, wrong.PhiDeg)
}

// **取付角を CAD に固定したまま、車輪ログだけで半径の比が出る。**
//
// これが 2026-09-23 に切り分けを作り直した結果 (研究 §2.6)。
// 角度は機械加工で決まるので CAD を信用し、ゴム・摩耗・荷重で変わる半径だけを
// データから決める。
func TestRadiiFromNullVectorRecoversUnequalRadii(t *testing.T) {
	truth := cadGeometry()
	truth.WheelRadiusM = [NumWheels]float64{0.02874, 0.02798, 0.02807, 0.02961}

	samples := excitedWheelLog(t, truth, 4000, 0.02, 71)
	base := cadGeometry() // 角度・アーム・符号は CAD、半径は公称のまま
	got, fit, err := CalibrateRadii(samples, base, meanOf(truth.WheelRadiusM))
	if err != nil {
		t.Fatalf("CalibrateRadii: %v (identifiability %.1f)", err, fit.Identifiability)
	}
	for i := 0; i < NumWheels; i++ {
		if d := math.Abs(got.WheelRadiusM[i] - truth.WheelRadiusM[i]); d > 0.0002 {
			t.Errorf("wheel %d radius %.5f m, want %.5f (off by %.2f mm)",
				i, got.WheelRadiusM[i], truth.WheelRadiusM[i], d*1000)
		}
	}
	t.Logf("recovered radii [mm]: %.2f %.2f %.2f %.2f (truth %.2f %.2f %.2f %.2f)",
		got.WheelRadiusM[0]*1000, got.WheelRadiusM[1]*1000, got.WheelRadiusM[2]*1000, got.WheelRadiusM[3]*1000,
		truth.WheelRadiusM[0]*1000, truth.WheelRadiusM[1]*1000, truth.WheelRadiusM[2]*1000, truth.WheelRadiusM[3]*1000)
}

// 絶対値は車輪ログからは決まらない。比だけである。
func TestRadiiFromNullVectorOnlyDeterminesRatios(t *testing.T) {
	truth := cadGeometry()
	truth.WheelRadiusM = [NumWheels]float64{0.02874, 0.02798, 0.02807, 0.02961}
	samples := excitedWheelLog(t, truth, 4000, 0.02, 72)

	a, _, err := CalibrateRadii(samples, cadGeometry(), 0.0280)
	if err != nil {
		t.Fatal(err)
	}
	b, _, err := CalibrateRadii(samples, cadGeometry(), 0.0300)
	if err != nil {
		t.Fatal(err)
	}
	// 平均を変えても比は変わらない。
	for i := 0; i < NumWheels; i++ {
		ra := a.WheelRadiusM[i] / meanOf(a.WheelRadiusM)
		rb := b.WheelRadiusM[i] / meanOf(b.WheelRadiusM)
		if math.Abs(ra-rb) > 1e-9 {
			t.Fatalf("wheel %d: ratio changed with the assumed mean (%.6f vs %.6f)", i, ra, rb)
		}
	}
	t.Log("only the ratios are determined; the absolute scale must come from vision (or the filter's k_v)")
}

// 加振が足りないログは断る。
func TestCalibrateRadiiRejectsPoorExcitation(t *testing.T) {
	g := cadGeometry()
	k, err := NewKinematics(g)
	if err != nil {
		t.Fatal(err)
	}
	rng := rand.New(rand.NewSource(73))
	samples := make([][NumWheels]float64, 2000)
	for i := range samples {
		w := k.WheelFromBody(0.5*math.Sin(float64(i)*0.02), 0, 0)
		for j := range w {
			w[j] += rng.NormFloat64() * 0.02
		}
		samples[i] = w
	}
	if _, _, err := CalibrateRadii(samples, g, 0.0286); err == nil {
		t.Fatal("a straight-line-only log should be rejected")
	}
}

func meanOf(v [NumWheels]float64) float64 {
	var s float64
	for _, x := range v {
		s += x
	}
	return s / float64(NumWheels)
}
