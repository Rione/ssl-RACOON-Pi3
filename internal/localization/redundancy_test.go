package localization

import (
	"math"
	"math/rand"
	"testing"
)

// cadGeometry は 3D モデル (Robot_V2.step) から読んだ設計値。
func cadGeometry() GeometryConfig {
	g := DefaultGeometry()
	g.WheelAnglesDeg = [NumWheels]float64{60, 135, -135, -60}
	g.MomentArmM = 0.07845
	r := 0.0280
	g.WheelRadiusM = [NumWheels]float64{r, r, r, r}
	g.WheelSigns = [NumWheels]float64{-1, -1, -1, -1}
	return g
}

// stmGeometry は新世代 STM ファームの値 (前後とも左右対称で 55 / 135 度)。
// 閉形式 (phi, theta, -theta, -phi) が成り立つ形なので、比較の基準に使う。
func stmGeometry() GeometryConfig {
	g := DefaultGeometry()
	g.WheelAnglesDeg = [NumWheels]float64{55, 135, -135, -55}
	g.MomentArmM = 0.075
	r := 0.030
	g.WheelRadiusM = [NumWheels]float64{r, r, r, r}
	return g
}

// identifiedGeometry は PoC のリプレイで同定した値。
//
// **前後とも左右非対称** (55.4 と -57.4、136.1 と -136.3) なので、
// 閉形式の左零ベクトルは成り立たない。数値解 (SVD) は使える。
func identifiedGeometry() GeometryConfig {
	g := DefaultGeometry()
	g.WheelAnglesDeg = [NumWheels]float64{55.4, 136.1, -136.3, -57.4}
	g.MomentArmM = 0.074
	r := 0.0286
	g.WheelRadiusM = [NumWheels]float64{r, r, r, r}
	g.WheelSigns = [NumWheels]float64{-1, -1, -1, -1}
	return g
}

func newRedundancyFor(t *testing.T, g GeometryConfig) (*Kinematics, *Redundancy) {
	t.Helper()
	k, err := NewKinematics(g)
	if err != nil {
		t.Fatalf("NewKinematics: %v", err)
	}
	red, err := NewRedundancy(k)
	if err != nil {
		t.Fatalf("NewRedundancy: %v", err)
	}
	return k, red
}

// 数値解 (SVD) と閉形式が一致することを固定する。
// 閉形式は alpha = (phi, theta, -theta, -phi) の機体でのみ成り立つ。
func TestClosedFormNullVectorMatchesSVD(t *testing.T) {
	// 閉形式は alpha = (phi, theta, -theta, -phi) の機体でのみ成り立つ。
	// DefaultGeometry (同定値) は左右非対称なので、ここには入れない。
	for name, g := range map[string]GeometryConfig{
		"cad":      cadGeometry(),
		"stm55":    stmGeometry(),
		"unequalR": unequalRadiusGeometry(),
	} {
		_, red := newRedundancyFor(t, g)
		got := red.NullVector()
		want := ClosedFormNullVector(g)

		// 両方を最大成分で正規化してから比べる。
		got = normalizeMaxAbs(got)
		want = normalizeMaxAbs(want)
		for i := 0; i < NumWheels; i++ {
			if math.Abs(got[i]-want[i]) > 1e-9 {
				t.Fatalf("%s: null vector mismatch: svd=%v closed=%v", name, got, want)
			}
		}
	}
}

func unequalRadiusGeometry() GeometryConfig {
	g := cadGeometry()
	g.WheelRadiusM = [NumWheels]float64{0.0280, 0.0291, 0.0276, 0.0285}
	g.WheelSigns = [NumWheels]float64{-1, -1, -1, -1}
	return g
}

// normalizeMaxAbs は Redundancy と同じ規約へそろえる
// (最大成分の絶対値を 1、最初の有意な成分を正)。
func normalizeMaxAbs(v [NumWheels]float64) [NumWheels]float64 {
	maxAbs := 0.0
	for i := 0; i < NumWheels; i++ {
		if a := math.Abs(v[i]); a > maxAbs {
			maxAbs = a
		}
	}
	if maxAbs == 0 {
		return v
	}
	var out [NumWheels]float64
	for i := 0; i < NumWheels; i++ {
		out[i] = v[i] / maxAbs
	}
	for i := 0; i < NumWheels; i++ {
		if math.Abs(out[i]) > 1e-9 {
			if out[i] < 0 {
				for j := 0; j < NumWheels; j++ {
					out[j] = -out[j]
				}
			}
			break
		}
	}
	return out
}

// 滑りが無ければ、どんな運動でも残差は厳密にゼロになる。
func TestRedundancyResidualIsZeroWithoutSlip(t *testing.T) {
	for name, g := range map[string]GeometryConfig{
		"cad":        cadGeometry(),
		"identified": identifiedGeometry(),
		"unequalR":   unequalRadiusGeometry(),
	} {
		k, red := newRedundancyFor(t, g)
		rng := rand.New(rand.NewSource(1))
		for i := 0; i < 200; i++ {
			vx := rng.NormFloat64() * 2
			vy := rng.NormFloat64() * 2
			w := rng.NormFloat64() * 10
			wheels := k.WheelFromBody(vx, vy, w)
			if r := red.Residual(wheels); math.Abs(r) > 1e-9 {
				t.Fatalf("%s: residual %.3g for v=(%.2f,%.2f,%.2f)", name, r, vx, vy, w)
			}
		}
	}
}

// 残差はモーメントアームに依存しない (角度と半径の比だけで決まる)。
func TestRedundancyIndependentOfMomentArm(t *testing.T) {
	a := cadGeometry()
	b := cadGeometry()
	b.MomentArmM = 0.090
	_, ra := newRedundancyFor(t, a)
	_, rb := newRedundancyFor(t, b)
	na, nb := normalizeMaxAbs(ra.NullVector()), normalizeMaxAbs(rb.NullVector())
	for i := 0; i < NumWheels; i++ {
		if math.Abs(na[i]-nb[i]) > 1e-9 {
			t.Fatalf("null vector changed with moment arm: %v vs %v", na, nb)
		}
	}
}

// 1 輪だけ滑らせると残差が立つ。
func TestRedundancyDetectsSingleWheelSlip(t *testing.T) {
	g := cadGeometry()
	k, red := newRedundancyFor(t, g)
	wheels := k.WheelFromBody(0.4, 0, 0)
	base := red.Residual(wheels)

	wheels[WheelFL] += 1.0 // 1 rad/s ぶん余計に回る = 滑り
	slipped := red.Residual(wheels)
	if math.Abs(slipped-base) < 0.5 {
		t.Fatalf("slip did not move the residual: base=%.4g slipped=%.4g", base, slipped)
	}

	mon := NewSlipMonitor(red, 0.02, 0, 0)
	if !mon.Observe(wheels) {
		t.Fatalf("slip monitor missed a 1 rad/s single-wheel slip (normalized=%.3f)", mon.NormalizedResidual())
	}
}

// 雑音だけならスリップと判定しない。
func TestSlipMonitorAcceptsNoise(t *testing.T) {
	g := cadGeometry()
	k, red := newRedundancyFor(t, g)
	const sigma = 0.02
	mon := NewSlipMonitor(red, sigma, 0, 0)
	rng := rand.New(rand.NewSource(7))

	const n = 2000
	for i := 0; i < n; i++ {
		wheels := k.WheelFromBody(0.4*math.Sin(float64(i)*0.01), 0.2, 1.0)
		for j := range wheels {
			wheels[j] += rng.NormFloat64() * sigma
		}
		mon.Observe(wheels)
	}
	// 99% 点なので、誤検出は 1% 前後に収まるはず。
	if rate := float64(mon.SlipCount()) / n; rate > 0.03 {
		t.Fatalf("false slip rate %.3f is too high", rate)
	}

	got, ok := mon.MeasuredWheelNoise()
	if !ok {
		t.Fatal("MeasuredWheelNoise returned false")
	}
	if got < sigma*0.8 || got > sigma*1.2 {
		t.Fatalf("measured wheel noise %.4f rad/s, want %.4f +-20%%", got, sigma)
	}
	bias, ok := mon.ResidualBias()
	if !ok || math.Abs(bias) > 3*sigma*red.NoiseScale()/math.Sqrt(float64(n)) {
		t.Fatalf("residual bias %.5g is not consistent with zero", bias)
	}
}

// 幾何が間違っていると、滑っていなくても残差に偏りが出る。
// **これが 55 度説と 60 度説を分ける信号である。**
func TestWrongAngleShowsUpAsResidualBias(t *testing.T) {
	truth := cadGeometry() // 実機は 60 度
	kTruth, err := NewKinematics(truth)
	if err != nil {
		t.Fatal(err)
	}
	wrong := cadGeometry()
	wrong.WheelAnglesDeg = [NumWheels]float64{55, 135, -135, -55} // 55 度だと思って復号
	_, redWrong := newRedundancyFor(t, wrong)

	const sigma = 0.02
	rng := rand.New(rand.NewSource(3))
	var sum, sum2 float64
	const n = 1000
	for i := 0; i < n; i++ {
		// 横移動を含む運動でないと差が出にくい。
		wheels := kTruth.WheelFromBody(0.3, 0.3, 1.0)
		for j := range wheels {
			wheels[j] += rng.NormFloat64() * sigma
		}
		r := redWrong.Residual(wheels)
		sum += r
		sum2 += r * r
	}
	mean := sum / n
	rms := math.Sqrt(sum2 / n)
	noiseFloor := sigma * redWrong.NoiseScale()
	if math.Abs(mean) < 3*noiseFloor {
		t.Fatalf("wrong angle produced only %.4g rad/s bias (noise floor %.4g)", mean, noiseFloor)
	}
	t.Logf("wrong-angle residual: mean=%.4f rms=%.4f rad/s (noise floor %.4f)", mean, rms, noiseFloor)
}

func TestRedundancyDoesNotAllocate(t *testing.T) {
	g := cadGeometry()
	k, red := newRedundancyFor(t, g)
	wheels := k.WheelFromBody(0.4, 0.2, 1.0)
	mon := NewSlipMonitor(red, 0.02, 0, 0)
	if n := testing.AllocsPerRun(1000, func() {
		_ = red.Residual(wheels)
		_ = mon.Observe(wheels)
	}); n != 0 {
		t.Fatalf("residual path allocates %v times per run", n)
	}
}
