package locsim

import (
	"math"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// **ジャイロが載ったときに何が良くなるかを、載る前に測れるようにしておく。**
//
// ssl-Circuit の FW/MainBoard_V26_2 に LSM6DSO32 を読む実装がある。
// SPI の上りに載れば、このテストがそのまま効果の実測になる。
func TestGyroImprovesHeadingDuringVisionOutage(t *testing.T) {
	base := DefaultConfig()
	base.VisionOutages = []Outage{{Start: 8 * time.Second, End: 10 * time.Second}}
	// **ヨーレートが変化する軌道でないと、ジャイロバイアス b_g と回転の倍率 Kw が
	// 分離しない。** 一定の omega では b_g ≒ (Kw - 1)*omega で完全に縮退する。
	// 実機では停止 (ZARU) でも分離するが、ここは走りっぱなしの条件で見る。
	tr := FigureEight(localization.Vec2{}, 1.2, 4*time.Second,
		HeadingParams{Mode: HeadingTangent}, 14*time.Second)

	noGyro, err := Generate(tr, base, 41)
	if err != nil {
		t.Fatal(err)
	}
	withGyro := base
	withGyro.HasGyro = true
	withGyro.GyroNoiseRadS = 0.037 // RoboTeam Twente の走行中の実測
	withGyro.GyroBiasRadS = 0.02   // 1.1 deg/s のバイアス
	gy, err := Generate(tr, withGyro, 41)
	if err != nil {
		t.Fatal(err)
	}

	cfg := localization.DefaultConfig()
	opts := localization.EstimatorOptions{VisionDelayComp: base.VisionTimeBias}

	estNo, _, err := RunFilter(noGyro, cfg, opts)
	if err != nil {
		t.Fatal(err)
	}
	estGy, filter, err := RunFilter(gy, cfg, opts)
	if err != nil {
		t.Fatal(err)
	}

	hNo := headingErrorDuring(noGyro, estNo, base.VisionOutages[0])
	hGy := headingErrorDuring(gy, estGy, base.VisionOutages[0])
	t.Logf("heading error during a 2 s vision outage: no gyro %.3f deg, with gyro %.3f deg",
		hNo*180/math.Pi, hGy*180/math.Pi)
	t.Logf("estimated gyro bias %.4f rad/s (true %.4f), gyro updates %d",
		filter.GyroBias(), withGyro.GyroBiasRadS, filter.Stats().GyroUpdates)

	if hGy >= hNo {
		t.Errorf("the gyro did not help: %.3f deg -> %.3f deg", hNo*180/math.Pi, hGy*180/math.Pi)
	}
	// バイアスが当たっていること。外すと欠落中に向きが流れる。
	if d := math.Abs(filter.GyroBias() - withGyro.GyroBiasRadS); d > 0.01 {
		t.Errorf("gyro bias estimate is off by %.4f rad/s", d)
	}
}

// ジャイロが届かない機体でも、これまでどおり動くこと。
func TestNoGyroIsHarmless(t *testing.T) {
	cfg := DefaultConfig() // HasGyro = false
	tr := Line(localization.Vec2{}, localization.Vec2{X: 1.5, Y: 0}, localization.Vec2{},
		HeadingParams{Mode: HeadingFixed}, 6*time.Second)
	s, err := Generate(tr, cfg, 42)
	if err != nil {
		t.Fatal(err)
	}
	if len(s.Imu) != 0 {
		t.Fatalf("expected no IMU samples, got %d", len(s.Imu))
	}
	rep, err := Evaluate("no gyro", s, localization.DefaultConfig(),
		localization.EstimatorOptions{VisionDelayComp: cfg.VisionTimeBias})
	if err != nil {
		t.Fatal(err)
	}
	if rep.Fused.PositionRMSEm > rep.Raw.PositionRMSEm {
		t.Errorf("without a gyro the filter got worse than raw vision: %.2f vs %.2f mm",
			rep.Fused.PositionRMSEm*1000, rep.Raw.PositionRMSEm*1000)
	}
	t.Log("\n" + rep.String())
}

func headingErrorDuring(s *Sensors, est []localization.Estimate, o Outage) float64 {
	var worst float64
	for _, e := range est {
		if e.Stamp < localization.Stamp(o.Start) || e.Stamp > localization.Stamp(o.End) {
			continue
		}
		truth, ok := s.TruthAt(e.Stamp)
		if !ok {
			continue
		}
		if d := math.Abs(localization.AngleDiff(e.Pose.Theta, truth.Pose.Theta)); d > worst {
			worst = d
		}
	}
	return worst
}
