package locsim

import (
	"math"
	"math/rand"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

func TestEvaluatePosesPerfectEstimateHasZeroError(t *testing.T) {
	s, err := Generate(testTrajectory(), DefaultConfig(), 1)
	if err != nil {
		t.Fatal(err)
	}
	times := make([]localization.Stamp, 0, len(s.Truth))
	for _, tv := range s.Truth {
		times = append(times, tv.Stamp)
	}
	got := EvaluatePoses(s, times, func(st localization.Stamp) (localization.Pose2, bool) {
		tv, ok := s.TruthAt(st)
		return tv.Pose, ok
	})
	if got.PositionRMSEm > 1e-12 || got.HeadingRMSErad > 1e-12 {
		t.Errorf("a perfect estimate scored %s", got)
	}
	if got.Samples != len(times) {
		t.Errorf("evaluated %d samples, want %d", got.Samples, len(times))
	}
}

func TestEvaluatePosesKnownOffset(t *testing.T) {
	s, err := Generate(testTrajectory(), DefaultConfig(), 2)
	if err != nil {
		t.Fatal(err)
	}
	times := []localization.Stamp{s.Truth[10].Stamp, s.Truth[20].Stamp, s.Truth[30].Stamp}
	const dx = 0.03 // 30 mm ずらす
	got := EvaluatePoses(s, times, func(st localization.Stamp) (localization.Pose2, bool) {
		tv, ok := s.TruthAt(st)
		tv.Pose.X += dx
		return tv.Pose, ok
	})
	if math.Abs(got.PositionRMSEm-dx) > 1e-12 {
		t.Errorf("rmse = %v, want exactly %v", got.PositionRMSEm, dx)
	}
	if math.Abs(got.PositionMaxM-dx) > 1e-12 {
		t.Errorf("max = %v, want exactly %v", got.PositionMaxM, dx)
	}
}

// 角度誤差は ±pi を跨いでも正しく測れること。
func TestEvaluatePosesWrapsHeading(t *testing.T) {
	cfg := DefaultConfig()
	tr := Stationary(localization.Vec2{}, HeadingParams{Theta0: math.Pi - 0.01}, time.Second)
	s, err := Generate(tr, cfg, 3)
	if err != nil {
		t.Fatal(err)
	}
	times := []localization.Stamp{s.Truth[5].Stamp}
	got := EvaluatePoses(s, times, func(st localization.Stamp) (localization.Pose2, bool) {
		// 真値のすぐ隣だが、符号が反転して -pi 側にある値。
		return localization.Pose2{Theta: -math.Pi + 0.01}, true
	})
	if math.Abs(got.HeadingRMSErad-0.02) > 1e-9 {
		t.Errorf("heading error across the pi boundary = %v rad, want 0.02", got.HeadingRMSErad)
	}
}

// vision 生値の保持が「最後に届いた観測」を返すこと。
func TestRawVisionHold(t *testing.T) {
	cfg := DefaultConfig()
	cfg.VisionLossRate = 0
	cfg.VisionNoiseM = 0
	cfg.VisionNoiseRad = 0
	s, err := Generate(testTrajectory(), cfg, 4)
	if err != nil {
		t.Fatal(err)
	}
	hold := RawVisionHold(s)

	// 最初の観測より前は値が無い。
	if _, ok := hold(0); ok && s.Vision[0].Stamp > 0 {
		t.Error("RawVisionHold returned a pose before the first observation")
	}
	// 観測のちょうどその時刻と、その直後は同じ値。
	v := s.Vision[10]
	a, ok1 := hold(v.Stamp)
	b, ok2 := hold(v.Stamp + localization.Stamp(time.Millisecond))
	if !ok1 || !ok2 {
		t.Fatal("RawVisionHold failed at a known observation time")
	}
	if a != v.Pose || b != v.Pose {
		t.Errorf("hold returned %+v / %+v, want %+v", a, b, v.Pose)
	}
	// 次の観測の時刻になったら切り替わること。
	next := s.Vision[11]
	c, _ := hold(next.Stamp)
	if c != next.Pose {
		t.Errorf("hold did not advance to the next observation")
	}
}

// 比較対象のベースラインが妥当な大きさであること。
// これがフィルタの越えるべき棒になる。
func TestRawVisionBaselineIsMeasurable(t *testing.T) {
	cfg := DefaultConfig()
	tr := Line(localization.Vec2{}, localization.Vec2{X: 2}, localization.Vec2{},
		HeadingParams{Mode: HeadingFixed}, 10*time.Second)
	s, err := Generate(tr, cfg, 5)
	if err != nil {
		t.Fatal(err)
	}
	times := make([]localization.Stamp, 0, len(s.Truth))
	for _, tv := range s.Truth {
		times = append(times, tv.Stamp)
	}
	raw := EvaluateVisionRaw(s, times)
	t.Logf("raw vision baseline at 2 m/s: %s", raw)

	// 2 m/s で 20 ms の遅延 = 40 mm、加えて 1/60 秒の保持で最大 33 mm。
	// 誤差がこの桁で出ていなければ、シナリオが軽すぎて比較の意味がない。
	if raw.PositionRMSEm < 0.01 {
		t.Errorf("raw vision rmse %.2f mm is implausibly small; the scenario is too easy to show any improvement",
			raw.PositionRMSEm*1000)
	}
	if raw.PositionRMSEm > 0.2 {
		t.Errorf("raw vision rmse %.2f mm is implausibly large; check the delay and hold model",
			raw.PositionRMSEm*1000)
	}
}

func TestImprovement(t *testing.T) {
	raw := Errors{PositionRMSEm: 0.040}
	if got := Improvement(raw, Errors{PositionRMSEm: 0.020}); math.Abs(got-0.5) > 1e-12 {
		t.Errorf("Improvement = %v, want 0.5", got)
	}
	if got := Improvement(raw, Errors{PositionRMSEm: 0.080}); math.Abs(got+1.0) > 1e-12 {
		t.Errorf("Improvement for a worse estimate = %v, want -1.0", got)
	}
	if got := Improvement(Errors{}, Errors{PositionRMSEm: 1}); got != 0 {
		t.Errorf("Improvement against a zero baseline = %v, want 0", got)
	}
}

// --- NEES ---------------------------------------------------------------

// 誤差を既知の共分散から引いたとき、NEES の平均が自由度 3 に一致すること。
// ここが合わないと一貫性の検定そのものが信用できない。
func TestConsistencyOnCorrectCovariance(t *testing.T) {
	s, est, cov := neesFixture(t, 1.0, 11)
	c := EvaluateConsistency(s, est)
	t.Logf("correct covariance: %s", c)

	if math.Abs(c.MeanNEES-3.0) > 0.2 {
		t.Errorf("mean NEES = %.3f, want about 3.0 for 3 degrees of freedom", c.MeanNEES)
	}
	if r := c.Ratio(); r < 0.93 || r > 0.97 {
		t.Errorf("%.1f%% of samples fell in the 95%% interval, want about 95%%", r*100)
	}
	if c.Singular != 0 {
		t.Errorf("%d samples had a singular covariance", c.Singular)
	}
	_ = cov
}

// 共分散を実際より小さく申告した (過信した) 推定を、Optimistic として捕まえること。
// 位置制御が共分散を見てゲインを決める以上、この向きの誤りが一番危ない。
func TestConsistencyCatchesOverconfidence(t *testing.T) {
	// 誤差は真の分散で引くが、申告する共分散は 1/9 に縮めてある。
	s, est, _ := neesFixture(t, 1.0/9.0, 12)
	c := EvaluateConsistency(s, est)
	t.Logf("overconfident covariance: %s", c)

	if c.MeanNEES < 9 {
		t.Errorf("mean NEES = %.2f; a 3x understated sigma should give about 27", c.MeanNEES)
	}
	if c.Ratio() > 0.5 {
		t.Errorf("%.1f%% still fell inside the interval; overconfidence was not detected", c.Ratio()*100)
	}
	if c.Optimistic <= c.Pessimistic {
		t.Errorf("expected mostly optimistic failures, got %d optimistic / %d pessimistic",
			c.Optimistic, c.Pessimistic)
	}
}

// 共分散を実際より大きく申告した (弱気な) 推定は Pessimistic 側に出ること。
func TestConsistencyCatchesUnderconfidence(t *testing.T) {
	s, est, _ := neesFixture(t, 9.0, 13)
	c := EvaluateConsistency(s, est)
	t.Logf("underconfident covariance: %s", c)

	if c.MeanNEES > 1 {
		t.Errorf("mean NEES = %.2f; a 3x overstated sigma should give about 0.33", c.MeanNEES)
	}
	if c.Pessimistic <= c.Optimistic {
		t.Errorf("expected mostly pessimistic failures, got %d optimistic / %d pessimistic",
			c.Optimistic, c.Pessimistic)
	}
}

// neesFixture は真値のまわりに既知の分散で誤差を散らした推定列を作る。
// covScale は「申告する共分散 / 実際の分散」。1.0 なら正直。
func neesFixture(t *testing.T, covScale float64, seed int64) (*Sensors, []localization.Estimate, localization.Mat3) {
	t.Helper()
	cfg := DefaultConfig()
	tr := Line(localization.Vec2{}, localization.Vec2{X: 1}, localization.Vec2{},
		HeadingParams{Mode: HeadingFixed}, 60*time.Second)
	s, err := Generate(tr, cfg, seed)
	if err != nil {
		t.Fatal(err)
	}

	const (
		sigmaXY = 0.010 // 10 mm
		sigmaTh = 0.010 // 10 mrad
	)
	declared := localization.Mat3{
		{sigmaXY * sigmaXY * covScale, 0, 0},
		{0, sigmaXY * sigmaXY * covScale, 0},
		{0, 0, sigmaTh * sigmaTh * covScale},
	}

	rng := rand.New(rand.NewSource(seed))
	est := make([]localization.Estimate, 0, len(s.Truth))
	for _, tv := range s.Truth {
		est = append(est, localization.Estimate{
			Stamp: tv.Stamp,
			Pose: localization.Pose2{
				X:     tv.Pose.X + rng.NormFloat64()*sigmaXY,
				Y:     tv.Pose.Y + rng.NormFloat64()*sigmaXY,
				Theta: localization.WrapAngle(tv.Pose.Theta + rng.NormFloat64()*sigmaTh),
			},
			CovPose: declared,
			Health:  localization.HealthOK,
		})
	}
	return s, est, declared
}

func TestConsistencyFlagsSingularCovariance(t *testing.T) {
	s, err := Generate(testTrajectory(), DefaultConfig(), 14)
	if err != nil {
		t.Fatal(err)
	}
	est := []localization.Estimate{{Stamp: s.Truth[5].Stamp}} // CovPose はゼロ行列
	c := EvaluateConsistency(s, est)
	if c.Singular != 1 {
		t.Errorf("singular = %d, want 1 for an all-zero covariance", c.Singular)
	}
}

func TestInvert3(t *testing.T) {
	m := localization.Mat3{{4, 1, 0}, {1, 3, 1}, {0, 1, 2}}
	inv, ok := invert3(m)
	if !ok {
		t.Fatal("invert3 failed on a well-conditioned matrix")
	}
	// m * inv は単位行列になること。
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			var sum float64
			for k := 0; k < 3; k++ {
				sum += m[i][k] * inv[k][j]
			}
			want := 0.0
			if i == j {
				want = 1
			}
			if math.Abs(sum-want) > 1e-12 {
				t.Errorf("(m*inv)[%d][%d] = %v, want %v", i, j, sum, want)
			}
		}
	}
	if _, ok := invert3(localization.Mat3{}); ok {
		t.Error("invert3 accepted a singular matrix")
	}
}
