package localization

import (
	"math"
	"testing"
	"time"
)

// gyroRig は 125 Hz の SPI (車輪 + ジャイロ) と 100 Hz の vision を模した試験用の駆動。
// 真値は「一定の角速度で回りながら、並進はしない」。
type gyroRig struct {
	e       *Estimator
	t       Stamp
	trueYaw float64
	omega   float64
	bias    float64 // ジャイロに乗っているゼロ点のずれ [rad/s]
	useGyro bool
	useVis  bool
}

func newGyroRig(t *testing.T, omega, bias float64, useGyro, useVis bool) *gyroRig {
	t.Helper()
	cfg := DefaultConfig()
	cfg.Geometry.WheelSigns = [NumWheels]float64{-1, -1, -1, -1}
	e, err := NewEstimator(cfg, EstimatorOptions{})
	if err != nil {
		t.Fatal(err)
	}
	return &gyroRig{e: e, t: Stamp(10 * time.Second), omega: omega, bias: bias, useGyro: useGyro, useVis: useVis}
}

// step は 1 周期 (8 ms) 進める。車輪は真の角速度どおり、vision は 10 周期に 1 回。
func (r *gyroRig) step(i int) {
	const dt = 0.008
	r.t += Stamp(dt * float64(time.Second))
	r.trueYaw = WrapAngle(r.trueYaw + r.omega*dt)
	if r.useGyro {
		r.e.AddImu(ImuSample{Stamp: r.t, GyroZ: r.omega + r.bias, HasGyro: true})
	}
	kin, _ := NewKinematics(r.e.f.cfg.Geometry)
	w := kin.WheelFromBody(0, 0, r.omega)
	r.e.AddWheel(WheelSample{Stamp: r.t, Omega: w})
	if r.useVis && i%10 == 0 {
		r.e.AddVision(VisionPose{Stamp: r.t, Pose: Pose2{Theta: r.trueYaw}, Confidence: 1})
	}
}

func TestGyroUpdateTracksYawRate(t *testing.T) {
	r := newGyroRig(t, 1.5, 0, true, true)
	for i := 0; i < 250; i++ { // 2 s
		r.step(i)
	}
	est := r.e.Current()
	if math.Abs(est.YawRate-1.5) > 0.05 {
		t.Errorf("yaw rate = %.3f rad/s, want 1.5", est.YawRate)
	}
	if d := math.Abs(AngleDiff(est.Pose.Theta, r.trueYaw)); d > 0.02 {
		t.Errorf("heading off by %.1f deg", d*180/math.Pi)
	}
	if r.e.Stats().GyroUpdates == 0 {
		t.Error("no gyro updates were applied")
	}
}

func TestGyroBiasIsEstimated(t *testing.T) {
	// 止まっている機体 (真の角速度 0) にゼロ点のずれ 0.05 rad/s が乗ったジャイロ。
	// vision と車輪が「回っていない」と言うので、ずれはバイアスとして吸われるべき。
	r := newGyroRig(t, 0, 0.05, true, true)
	for i := 0; i < 1250; i++ { // 10 s
		r.step(i)
	}
	est := r.e.Current()
	if math.Abs(est.GyroBias-0.05) > 0.02 {
		t.Errorf("gyro bias = %.4f rad/s, want 0.05", est.GyroBias)
	}
	if math.Abs(est.YawRate) > 0.02 {
		t.Errorf("yaw rate = %.4f rad/s, want ~0 (the offset must go to the bias)", est.YawRate)
	}
	if d := math.Abs(AngleDiff(est.Pose.Theta, r.trueYaw)); d > 0.05 {
		t.Errorf("heading drifted %.1f deg", d*180/math.Pi)
	}
}

// vision が切れている間、ジャイロがあると向きがどれだけ持つか。
// 車輪だけのときと比べる (車輪の寸法は少しずれているという想定)。
func TestGyroHoldsHeadingWhileVisionIsGone(t *testing.T) {
	run := func(useGyro bool) float64 {
		r := newGyroRig(t, 2.0, 0, useGyro, true)
		for i := 0; i < 125; i++ { // 1 s: vision あり
			r.step(i)
		}
		// 車輪の腕の長さを 10% ずらす = 車輪から出した角速度がその分ずれる
		r.e.f.kin.cfg.MomentArmM *= 1.1
		k, _ := NewKinematics(r.e.f.kin.cfg)
		r.e.f.kin = k
		r.useVis = false
		for i := 0; i < 50; i++ { // 0.4 s: vision 無し
			r.step(i)
		}
		return math.Abs(AngleDiff(r.e.Current().Pose.Theta, r.trueYaw))
	}
	withGyro, wheelsOnly := run(true), run(false)
	t.Logf("vision を 0.4 s 抜いたときの向きのずれ: ジャイロあり %.2f deg / 車輪だけ %.2f deg",
		withGyro*180/math.Pi, wheelsOnly*180/math.Pi)
	if withGyro > wheelsOnly {
		t.Errorf("gyro must not make the heading worse: %.3f vs %.3f rad", withGyro, wheelsOnly)
	}
}

func TestAddImuWithoutGyroDoesNothing(t *testing.T) {
	r := newGyroRig(t, 1.0, 0, false, true)
	before := r.e.Stats()
	r.e.AddImu(ImuSample{Stamp: r.t, Accel: Vec2{X: 1}, HasAccel: true})
	if r.e.Stats() != before {
		t.Error("an accel-only sample must not change anything yet")
	}
}
