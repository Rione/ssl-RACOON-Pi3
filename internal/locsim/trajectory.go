// Package locsim は自己位置推定の検証用に、真値つきの合成データを作る。
//
// 計画 §9 の「効果が測れること」を機能要件にするための土台である。
// RAVEN の EKF が不採用になった理由は「効果が測れなかった」ことなので、
// **フィルタ本体より先にこれを用意する。**
//
// 提供するのは 2 系統:
//
//	解析軌道 (trajectory.go)  位置・速度・加速度が閉じた式で求まる。真値が
//	                          数値誤差ゼロなので、フィルタの誤差だけを見られる。
//	動力学モデル (dynamics.go) 指令 -> 一次遅れ -> 実際の運動。スリップや飽和を
//	                          注入でき、スリップ状態の検証に使える。
//
// このパッケージにビルドタグは付けない。
package locsim

import (
	"fmt"
	"math"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/localization"
)

// Truth は真値の 1 サンプル。
type Truth struct {
	Stamp localization.Stamp
	// Pose はワールド系の姿勢 [m, m, rad]。
	Pose localization.Pose2
	// VelBody はロボット系の並進速度 [m/s]。
	VelBody localization.Vec2
	// YawRate はヨーレート [rad/s]。
	YawRate float64
	// VelWorld はワールド系の並進速度 [m/s]。診断用。
	VelWorld localization.Vec2
	// AccelBody はロボット系の加速度 [m/s^2]。IMU が載ったときに使う。
	AccelBody localization.Vec2
	// SlipBody は実際に生じたスリップ速度 [m/s]。解析軌道では常にゼロ。
	SlipBody localization.Vec2
}

// HeadingMode は機体の向きの決め方。
//
// SSL のロボットは全方向移動なので、**進行方向と機体の向きは独立**である。
// 接線追従だけで検証すると横方向速度 vy が常に 0 になり、
// 横方向の可観測性を一切試さないまま「動いた」ことになってしまう。
type HeadingMode int

const (
	// HeadingFixed は向きを固定する。真横・真後ろへの移動が入る。
	HeadingFixed HeadingMode = iota
	// HeadingSpin は一定角速度で回り続ける。並進と回転が混ざる。
	HeadingSpin
	// HeadingTangent は進行方向を向く。
	HeadingTangent
)

// HeadingParams は向きの設定。
type HeadingParams struct {
	Mode HeadingMode
	// Theta0 は初期角 [rad]。
	Theta0 float64
	// SpinRate は HeadingSpin のときの角速度 [rad/s]。
	SpinRate float64
}

// Trajectory は真値軌道。
type Trajectory interface {
	// Name は軌道の名前。テストの出力に使う。
	Name() string
	// Duration は軌道の長さ。
	Duration() time.Duration
	// At は経過時間 t における真値を返す。
	At(t time.Duration) Truth
}

// pathSample は向きを決める前の並進運動。
type pathSample struct {
	p, v, a localization.Vec2
}

// path は位置・速度・加速度を閉じた式で返す。
type path interface {
	name() string
	at(t float64) pathSample
}

// analytic は path と向きの設定を組み合わせた軌道。
type analytic struct {
	p        path
	heading  HeadingParams
	duration time.Duration
}

// NewTrajectory は path と向きから軌道を作る。
func newTrajectory(p path, h HeadingParams, d time.Duration) Trajectory {
	return &analytic{p: p, heading: h, duration: d}
}

func (a *analytic) Name() string            { return a.p.name() }
func (a *analytic) Duration() time.Duration { return a.duration }

func (a *analytic) At(t time.Duration) Truth {
	s := a.p.at(t.Seconds())
	theta, yawRate := a.headingAt(t.Seconds(), s)

	return Truth{
		Stamp:     localization.Stamp(t),
		Pose:      localization.Pose2{X: s.p.X, Y: s.p.Y, Theta: localization.WrapAngle(theta)},
		VelBody:   localization.RotateInv(theta, s.v),
		YawRate:   yawRate,
		VelWorld:  s.v,
		AccelBody: localization.RotateInv(theta, s.a),
	}
}

func (a *analytic) headingAt(t float64, s pathSample) (theta, yawRate float64) {
	switch a.heading.Mode {
	case HeadingSpin:
		return a.heading.Theta0 + a.heading.SpinRate*t, a.heading.SpinRate
	case HeadingTangent:
		speedSq := s.v.X*s.v.X + s.v.Y*s.v.Y
		if speedSq < 1e-12 {
			// 停止中は接線が定義できない。初期角を保つ。
			return a.heading.Theta0, 0
		}
		// d/dt atan2(vy, vx) = (vx*ay - vy*ax) / (vx^2 + vy^2)
		return math.Atan2(s.v.Y, s.v.X), (s.v.X*s.a.Y - s.v.Y*s.a.X) / speedSq
	default:
		return a.heading.Theta0, 0
	}
}

// --- 静止 ---------------------------------------------------------------

type stationaryPath struct{ at0 localization.Vec2 }

func (p stationaryPath) name() string { return "stationary" }
func (p stationaryPath) at(float64) pathSample {
	return pathSample{p: p.at0}
}

// Stationary は静止したままの軌道。ZUPT と停止時ドリフトの検証に使う
// (計画 §10-6: 10 秒静止で位置 <= 5 mm)。
func Stationary(at localization.Vec2, heading HeadingParams, d time.Duration) Trajectory {
	return newTrajectory(stationaryPath{at0: at}, heading, d)
}

// --- 等加速度直線 -------------------------------------------------------

type linePath struct {
	p0, v0, acc localization.Vec2
}

func (p linePath) name() string { return "line" }
func (p linePath) at(t float64) pathSample {
	return pathSample{
		p: localization.Vec2{
			X: p.p0.X + p.v0.X*t + 0.5*p.acc.X*t*t,
			Y: p.p0.Y + p.v0.Y*t + 0.5*p.acc.Y*t*t,
		},
		v: localization.Vec2{X: p.v0.X + p.acc.X*t, Y: p.v0.Y + p.acc.Y*t},
		a: p.acc,
	}
}

// Line は等加速度の直線軌道。acc をゼロにすれば等速直線になる。
// 計画 §10-1 (2 m/s 直線走行) と §10-3 (遅延補償) の検証に使う。
func Line(p0, v0, acc localization.Vec2, heading HeadingParams, d time.Duration) Trajectory {
	return newTrajectory(linePath{p0: p0, v0: v0, acc: acc}, heading, d)
}

// --- 円 -----------------------------------------------------------------

type circlePath struct {
	center localization.Vec2
	radius float64
	omega  float64
	phase  float64
}

func (p circlePath) name() string { return "circle" }
func (p circlePath) at(t float64) pathSample {
	ang := p.omega*t + p.phase
	s, c := math.Sincos(ang)
	rw := p.radius * p.omega
	rw2 := rw * p.omega
	return pathSample{
		p: localization.Vec2{X: p.center.X + p.radius*c, Y: p.center.Y + p.radius*s},
		v: localization.Vec2{X: -rw * s, Y: rw * c},
		a: localization.Vec2{X: -rw2 * c, Y: -rw2 * s},
	}
}

// Circle は等速円運動。向心加速度が常にかかるので、
// 等速直線では出ない誤差 (回転と並進の結合) が出る。
func Circle(center localization.Vec2, radius, speed float64, heading HeadingParams, d time.Duration) Trajectory {
	omega := 0.0
	if radius != 0 {
		omega = speed / radius
	}
	return newTrajectory(circlePath{center: center, radius: radius, omega: omega}, heading, d)
}

// --- 8 の字 -------------------------------------------------------------

type figureEightPath struct {
	center localization.Vec2
	amp    float64
	omega  float64
}

func (p figureEightPath) name() string { return "figure8" }

// x = A*sin(w t), y = (A/2)*sin(2 w t) のレムニスケート。
func (p figureEightPath) at(t float64) pathSample {
	w := p.omega
	s1, c1 := math.Sincos(w * t)
	s2, c2 := math.Sincos(2 * w * t)
	return pathSample{
		p: localization.Vec2{X: p.center.X + p.amp*s1, Y: p.center.Y + p.amp/2*s2},
		v: localization.Vec2{X: p.amp * w * c1, Y: p.amp * w * c2},
		a: localization.Vec2{X: -p.amp * w * w * s1, Y: -2 * p.amp * w * w * s2},
	}
}

// FigureEight は 8 の字軌道。曲率の符号が反転するので、
// 円では隠れる「回転方向が切り替わるときの誤差」が出る。
// period は 1 周にかかる時間。
func FigureEight(center localization.Vec2, amplitude float64, period time.Duration, heading HeadingParams, d time.Duration) Trajectory {
	omega := 0.0
	if period > 0 {
		omega = 2 * math.Pi / period.Seconds()
	}
	return newTrajectory(figureEightPath{center: center, amp: amplitude, omega: omega}, heading, d)
}

// --- 急加減速 -----------------------------------------------------------

type stepAccelPath struct {
	p0        localization.Vec2
	dir       localization.Vec2 // 単位ベクトル
	accel     float64
	cruise    float64
	tAccel    float64
	tCruise   float64
	tDecel    float64
	distAccel float64
	distCruis float64
}

func (p stepAccelPath) name() string { return "step-accel" }

func (p stepAccelPath) at(t float64) pathSample {
	var dist, speed, acc float64
	switch {
	case t <= 0:
		// 何もしない (すべてゼロ)
	case t < p.tAccel:
		acc = p.accel
		speed = p.accel * t
		dist = 0.5 * p.accel * t * t
	case t < p.tAccel+p.tCruise:
		speed = p.cruise
		dist = p.distAccel + p.cruise*(t-p.tAccel)
	case t < p.tAccel+p.tCruise+p.tDecel:
		td := t - p.tAccel - p.tCruise
		acc = -p.accel
		speed = p.cruise - p.accel*td
		dist = p.distAccel + p.distCruis + p.cruise*td - 0.5*p.accel*td*td
	default:
		// 停止
		dist = p.distAccel + p.distCruis + 0.5*p.cruise*p.tDecel
	}
	return pathSample{
		p: localization.Vec2{X: p.p0.X + p.dir.X*dist, Y: p.p0.Y + p.dir.Y*dist},
		v: localization.Vec2{X: p.dir.X * speed, Y: p.dir.Y * speed},
		a: localization.Vec2{X: p.dir.X * acc, Y: p.dir.Y * acc},
	}
}

// StepAccel は「加速 -> 等速 -> 急停止」の軌道。
//
// 加速度が不連続に変わるので、定速度モデルの予測が最も苦しくなる。
// 遅延補償の誤りもここで一番はっきり出る。
func StepAccel(p0 localization.Vec2, headingDir float64, accel, cruise float64,
	cruiseFor time.Duration, heading HeadingParams, d time.Duration) Trajectory {
	if accel <= 0 {
		accel = 1
	}
	s, c := math.Sincos(headingDir)
	tAccel := cruise / accel
	p := stepAccelPath{
		p0:      p0,
		dir:     localization.Vec2{X: c, Y: s},
		accel:   accel,
		cruise:  cruise,
		tAccel:  tAccel,
		tCruise: cruiseFor.Seconds(),
		tDecel:  tAccel,
	}
	p.distAccel = 0.5 * accel * tAccel * tAccel
	p.distCruis = cruise * p.tCruise
	return newTrajectory(p, heading, d)
}

// Sample は軌道を一定間隔で刻んで真値列を返す。
func Sample(tr Trajectory, dt time.Duration) []Truth {
	if dt <= 0 {
		panic(fmt.Sprintf("locsim: non-positive sample interval %v", dt))
	}
	n := int(tr.Duration() / dt)
	out := make([]Truth, 0, n+1)
	for i := 0; i <= n; i++ {
		out = append(out, tr.At(time.Duration(i)*dt))
	}
	return out
}
