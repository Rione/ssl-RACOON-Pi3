package localization

import (
	"math"
	"math/rand"
	"testing"
)

// synthesize は既知の機体パラメータから加振ログを合成する。
//
// slotOrder[slot] = 論理輪番号。つまり SPI のスロット slot には
// 論理輪 slotOrder[slot] の値が載る。
func synthesize(t *testing.T, g GeometryConfig, noise float64, seed int64) []IdentSample {
	t.Helper()
	k, err := NewKinematics(g)
	if err != nil {
		t.Fatalf("NewKinematics: %v", err)
	}
	rng := rand.New(rand.NewSource(seed))

	// 3 自由度を順に加振する。SPI の下りには車輪個別指令が無いので、
	// 実機でもこの 3 パターンしか与えられない。
	patterns := [][3]float64{
		{1, 0, 0}, {-1, 0, 0},
		{0, 1, 0}, {0, -1, 0},
		{0, 0, 1}, {0, 0, -1},
	}
	var out []IdentSample
	for _, p := range patterns {
		for i := 0; i < 100; i++ {
			// 各パターン内で振幅を振る。定速だけだと係数が 1 点でしか拘束されない。
			amp := 0.3 + 1.7*rng.Float64()
			vx, vy, om := p[0]*amp, p[1]*amp, p[2]*amp*4
			logical := k.WheelFromBody(vx, vy, om)

			var s IdentSample
			s.VX, s.VY, s.Omega = vx, vy, om
			for slot := 0; slot < NumWheels; slot++ {
				w := logical[g.WheelSlotOrder[slot]]
				if noise > 0 {
					w += rng.NormFloat64() * noise
				}
				s.WheelSlots[slot] = w
			}
			out = append(out, s)
		}
	}
	return out
}

func checkIdentified(t *testing.T, res IdentResult, want GeometryConfig, angleTolDeg, radiusTolMm float64) {
	t.Helper()
	got := res.Geometry

	if got.WheelSlotOrder != want.WheelSlotOrder {
		t.Errorf("wheelSlotOrder = %v, want %v", got.WheelSlotOrder, want.WheelSlotOrder)
	}
	if got.WheelSigns != want.WheelSigns {
		t.Errorf("wheelSigns = %v, want %v", got.WheelSigns, want.WheelSigns)
	}
	for i := 0; i < NumWheels; i++ {
		if d := math.Abs(wrapDeg(got.WheelAnglesDeg[i] - want.WheelAnglesDeg[i])); d > angleTolDeg {
			t.Errorf("wheel %s angle = %.3f deg, want %.3f (off by %.3f)",
				wheelName(i), got.WheelAnglesDeg[i], want.WheelAnglesDeg[i], d)
		}
		if d := math.Abs(got.WheelRadiusM[i]-want.WheelRadiusM[i]) * 1000; d > radiusTolMm {
			t.Errorf("wheel %s radius = %.3f mm, want %.3f (off by %.3f)",
				wheelName(i), got.WheelRadiusM[i]*1000, want.WheelRadiusM[i]*1000, d)
		}
	}
	if d := math.Abs(got.MomentArmM-want.MomentArmM) * 1000; d > radiusTolMm {
		t.Errorf("momentArm = %.3f mm, want %.3f (off by %.3f)",
			got.MomentArmM*1000, want.MomentArmM*1000, d)
	}
}

// 素直なケース。既定のパラメータをそのまま復元できること。
func TestIdentifyRecoversDefaultGeometry(t *testing.T) {
	want := DefaultGeometry()
	res, err := Identify(synthesize(t, want, 0, 1), RefCommand, IdentOptions{})
	if err != nil {
		t.Fatalf("Identify: %v", err)
	}
	checkIdentified(t, res, want, 1e-6, 1e-6)
	if len(res.Warnings) != 0 {
		t.Errorf("unexpected warnings: %v", res.Warnings)
	}
	for _, s := range res.Slots {
		if s.R2 < 0.999999 {
			t.Errorf("%s", s.Summary())
		}
	}
}

// これが本題。計画 §3.4 / §12-A5 の「新世代 STM だけ符号が反転している」が
// 実機で起きていたとして、それを検出できること。
func TestIdentifyDetectsFlippedSignConvention(t *testing.T) {
	want := DefaultGeometry()
	for i := range want.WheelSigns {
		want.WheelSigns[i] = -1
	}
	res, err := Identify(synthesize(t, want, 0, 2), RefCommand, IdentOptions{})
	if err != nil {
		t.Fatalf("Identify: %v", err)
	}
	checkIdentified(t, res, want, 1e-6, 1e-6)
	for slot, s := range res.Slots {
		if s.Sign != -1 {
			t.Errorf("slot %d: sign = %+.0f, want -1", slot, s.Sign)
		}
	}
}

// 計画 §12-A4 の「FL と FR が世代間で入れ替わっている疑い」を検出できること。
func TestIdentifyDetectsSwappedFrontWheels(t *testing.T) {
	want := DefaultGeometry()
	want.WheelSlotOrder = [NumWheels]int{WheelFR, WheelBL, WheelBR, WheelFL}

	res, err := Identify(synthesize(t, want, 0, 3), RefCommand, IdentOptions{})
	if err != nil {
		t.Fatalf("Identify: %v", err)
	}
	checkIdentified(t, res, want, 1e-6, 1e-6)
}

// 符号反転とホイール入れ替わりが同時に起きていても解けること。
func TestIdentifyDetectsSwapAndFlipTogether(t *testing.T) {
	want := DefaultGeometry()
	want.WheelSlotOrder = [NumWheels]int{WheelFR, WheelBR, WheelBL, WheelFL}
	want.WheelSigns = [NumWheels]float64{-1, -1, -1, -1}

	res, err := Identify(synthesize(t, want, 0, 4), RefCommand, IdentOptions{})
	if err != nil {
		t.Fatalf("Identify: %v", err)
	}
	checkIdentified(t, res, want, 1e-6, 1e-6)
}

// 寸法が 4 候補のどれであっても実測から出せること (§12-A1..A3)。
func TestIdentifyRecoversAlternativeDimensions(t *testing.T) {
	cases := []struct {
		name     string
		radiusMm float64
		armMm    float64
		angle    float64
	}{
		{"old-gen STM", 27, 85, 55},
		{"RAVEN", 26, 90, 60},
		{"new-gen STM", 30, 75, 55},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			want := DefaultGeometry()
			r := c.radiusMm / 1000
			want.WheelRadiusM = [NumWheels]float64{r, r, r, r}
			want.MomentArmM = c.armMm / 1000
			want.WheelAnglesDeg = [NumWheels]float64{c.angle, 135, -135, -c.angle}

			res, err := Identify(synthesize(t, want, 0, 5), RefCommand, IdentOptions{})
			if err != nil {
				t.Fatalf("Identify: %v", err)
			}
			checkIdentified(t, res, want, 1e-6, 1e-6)
		})
	}
}

// 車輪ごとに半径が違っても個別に出せること (個体差の較正、§8)。
func TestIdentifyRecoversPerWheelRadius(t *testing.T) {
	want := DefaultGeometry()
	want.WheelRadiusM = [NumWheels]float64{0.0300, 0.0295, 0.0305, 0.0298}

	res, err := Identify(synthesize(t, want, 0, 6), RefCommand, IdentOptions{})
	if err != nil {
		t.Fatalf("Identify: %v", err)
	}
	checkIdentified(t, res, want, 1e-6, 1e-6)
}

// 実測には必ず雑音が乗る。エンコーダ量子化 (0.01 rad/s) の数倍でも
// 符号とホイール順序は確実に出ること。
func TestIdentifyIsRobustToNoise(t *testing.T) {
	want := DefaultGeometry()
	want.WheelSlotOrder = [NumWheels]int{WheelFR, WheelBL, WheelBR, WheelFL}
	want.WheelSigns = [NumWheels]float64{-1, -1, -1, -1}

	res, err := Identify(synthesize(t, want, 0.05, 7), RefCommand, IdentOptions{})
	if err != nil {
		t.Fatalf("Identify: %v", err)
	}
	// 符号と順序は厳密に一致しなければ意味がない。
	if res.Geometry.WheelSlotOrder != want.WheelSlotOrder {
		t.Errorf("wheelSlotOrder = %v, want %v", res.Geometry.WheelSlotOrder, want.WheelSlotOrder)
	}
	if res.Geometry.WheelSigns != want.WheelSigns {
		t.Errorf("wheelSigns = %v, want %v", res.Geometry.WheelSigns, want.WheelSigns)
	}
	// 寸法は雑音ぶん緩める。
	checkIdentified(t, res, want, 1.0, 1.0)
}

// 加振が 1 自由度しか無ければ解けない。黙って嘘の答えを返さないこと。
func TestIdentifyRejectsDegenerateExcitation(t *testing.T) {
	g := DefaultGeometry()
	k, err := NewKinematics(g)
	if err != nil {
		t.Fatal(err)
	}
	var samples []IdentSample
	for i := 0; i < 600; i++ {
		vx := 0.5 + float64(i)*0.001
		w := k.WheelFromBody(vx, 0, 0)
		samples = append(samples, IdentSample{VX: vx, WheelSlots: w})
	}
	if _, err := Identify(samples, RefCommand, IdentOptions{}); err == nil {
		t.Error("expected an error when only vx is excited")
	}
}

// 回転の加振が弱いと、モーメントアームは決まらない。警告を出すこと。
func TestIdentifyWarnsOnWeakRotationExcitation(t *testing.T) {
	g := DefaultGeometry()
	k, err := NewKinematics(g)
	if err != nil {
		t.Fatal(err)
	}
	rng := rand.New(rand.NewSource(8))
	var samples []IdentSample
	for i := 0; i < 600; i++ {
		vx := rng.NormFloat64()
		vy := rng.NormFloat64()
		om := rng.NormFloat64() * 0.01 // ほとんど回さない
		var s IdentSample
		s.VX, s.VY, s.Omega = vx, vy, om
		s.WheelSlots = k.WheelFromBody(vx, vy, om)
		samples = append(samples, s)
	}
	res, err := Identify(samples, RefCommand, IdentOptions{})
	if err != nil {
		t.Fatalf("Identify: %v", err)
	}
	found := false
	for _, w := range res.Warnings {
		if len(w) > 0 && (contains(w, "omega excitation") || contains(w, "moment arm")) {
			found = true
		}
	}
	if !found {
		t.Errorf("expected a warning about the weak rotation excitation, got %v", res.Warnings)
	}
}

func contains(s, sub string) bool {
	for i := 0; i+len(sub) <= len(s); i++ {
		if s[i:i+len(sub)] == sub {
			return true
		}
	}
	return false
}

func TestIdentifyRejectsTooFewSamples(t *testing.T) {
	g := DefaultGeometry()
	samples := synthesize(t, g, 0, 9)[:10]
	if _, err := Identify(samples, RefCommand, IdentOptions{}); err == nil {
		t.Error("expected an error for too few samples")
	}
}

// 同定した設定をそのまま運動学へ戻せること。これが成立して初めて
// 「同定結果を設定ファイルに書けば直る」と言える。
func TestIdentifiedGeometryIsUsable(t *testing.T) {
	want := DefaultGeometry()
	want.WheelSlotOrder = [NumWheels]int{WheelFR, WheelBL, WheelBR, WheelFL}
	want.WheelSigns = [NumWheels]float64{-1, -1, -1, -1}
	want.MomentArmM = 0.085

	res, err := Identify(synthesize(t, want, 0, 10), RefCommand, IdentOptions{})
	if err != nil {
		t.Fatal(err)
	}
	k, err := NewKinematics(res.Geometry)
	if err != nil {
		t.Fatalf("identified geometry is not usable: %v", err)
	}

	// 同定した設定で、スロット順の実測から body 速度を復元できること。
	truth, err := NewKinematics(want)
	if err != nil {
		t.Fatal(err)
	}
	for _, c := range [][3]float64{{1.5, 0, 0}, {0, -0.8, 0}, {0, 0, 3}, {1, -1, 2}} {
		logical := truth.WheelFromBody(c[0], c[1], c[2])
		var slots [NumWheels]float64
		for slot := 0; slot < NumWheels; slot++ {
			slots[slot] = logical[want.WheelSlotOrder[slot]]
		}
		vx, vy, om := k.BodyFromWheel(k.SlotsToLogical(slots))
		if math.Abs(vx-c[0]) > 1e-6 || math.Abs(vy-c[1]) > 1e-6 || math.Abs(om-c[2]) > 1e-6 {
			t.Errorf("body %v recovered as (%v, %v, %v)", c, vx, vy, om)
		}
	}
}
