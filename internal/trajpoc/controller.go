package trajpoc

import (
	"fmt"
	"math"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// Method は参照から速度を作る手法。PoC で比べる本体。
type Method string

const (
	// MethodP は位置の P 制御だけ。参照速度を使わないので、走行中は
	// 「速度 / Kp」だけ遅れ続ける。ベースライン。
	MethodP Method = "p"
	// MethodFFP は参照速度を先回し (フィードフォワード) し、位置の誤差を P で閉じる。
	// Pi2 の feat/#2-set-velocity (internal/control) と同じ形。
	MethodFFP Method = "ffp"
	// MethodFFPLead は MethodFFP に遅れ補償を足したもの。
	//   - vision の位置は撮影時刻のものなので、直前の指令で「今」まで進めてから誤差を取る
	//   - 指令が効くまでの遅れ (Lead) だけ先の参照を狙う
	MethodFFPLead Method = "ffp_lead"
)

// ParseMethod は文字列を Method にする。
func ParseMethod(s string) (Method, error) {
	switch Method(s) {
	case MethodP, MethodFFP, MethodFFPLead:
		return Method(s), nil
	}
	return "", fmt.Errorf("unknown method %q (p|ffp|ffp_lead)", s)
}

// Config は PoC の設定。ゲインは [1/s]、速度・加速度は SI。
type Config struct {
	Method Method
	Interp Interp

	Kp   float64 // 位置の P ゲイン [1/s]
	Kth  float64 // 向きの P ゲイン [1/s]
	Lead float64 // MethodFFPLead で先を狙う秒 (指令が効くまでの遅れ) [s]

	// 安全の枠。超えた指令は削り、枠の外へ出たら止める。
	MaxSpeed    float64 // 並進速度の上限 [m/s]
	MaxYawRate  float64 // 角速度の上限 [rad/s]
	MaxAccel    float64 // 指令の変化の上限 [m/s^2] (指令が跳ねて滑るのを防ぐ)
	Fence       float64 // 軌道が収まるべき開始位置からの半径 [m]。走行中は +FenceMargin で止める
	FenceMargin float64

	VisionHoldAge  float64 // vision がこれより古ければ 0 を出して待つ [s]
	VisionAbortAge float64 // これより古ければ打ち切る [s]

	StartDelay float64 // 走り出しの猶予 [s] (軌道の t=0 をいつにするか)
	Settle     float64 // 軌道の末尾の後、位置保持を続けて到着を測る秒 [s]
}

// DefaultConfig は最初の実機試験用の保守的な設定。
func DefaultConfig() Config {
	return Config{
		Method: MethodFFP, Interp: InterpHermite,
		Kp: 3.0, Kth: 4.0, Lead: 0.03,
		MaxSpeed: 0.5, MaxYawRate: 2.0, MaxAccel: 2.0,
		Fence: 1.0, FenceMargin: 0.2,
		VisionHoldAge: 0.25, VisionAbortAge: 1.0,
		StartDelay: 0.3, Settle: 0.8,
	}
}

// Validate は設定と相対軌道が走らせてよいものかを確かめる。走る前に呼ぶ。
func (c Config) Validate(rel []Knot) error {
	if !finite(c.Kp, c.Kth, c.Lead, c.MaxSpeed, c.MaxYawRate, c.MaxAccel, c.Fence) ||
		c.Kp < 0 || c.Kth < 0 || c.Lead < 0 || c.MaxSpeed <= 0 || c.MaxYawRate <= 0 || c.MaxAccel <= 0 || c.Fence <= 0 {
		return fmt.Errorf("invalid config")
	}
	b := MeasureBounds(rel)
	if b.StartOffset > 0.05 {
		return fmt.Errorf("trajectory must start at the origin (first point is %.3f m away)", b.StartOffset)
	}
	if b.MaxRadius > c.Fence {
		return fmt.Errorf("trajectory leaves the fence: %.2f m > %.2f m", b.MaxRadius, c.Fence)
	}
	if b.MaxSpeed > c.MaxSpeed*1.05 {
		return fmt.Errorf("trajectory is faster than the limit: %.2f m/s > %.2f m/s", b.MaxSpeed, c.MaxSpeed)
	}
	if b.MaxYawRate > c.MaxYawRate*1.05 {
		return fmt.Errorf("trajectory turns faster than the limit: %.2f rad/s > %.2f rad/s", b.MaxYawRate, c.MaxYawRate)
	}
	return nil
}

// command は手法ごとの生の指令 (ワールド系、制限前)。
// pose は vision の姿勢、age はその古さ [s]、prev は直前に出した指令。
func command(c Config, ref *Reference, t float64, pose localization.Pose2, age float64,
	prevVel localization.Vec2, prevOmega float64) (vel localization.Vec2, omega float64, bodyTheta float64) {
	p := localization.Vec2{X: pose.X, Y: pose.Y}
	th := pose.Theta
	target := ref.At(t)
	switch c.Method {
	case MethodP:
		target.Vel = localization.Vec2{}
		target.YawRate = 0
	case MethodFFPLead:
		// vision は撮影時刻の姿勢。直前の指令で今まで進めてから誤差を取る。
		p.X += prevVel.X * age
		p.Y += prevVel.Y * age
		th = localization.WrapAngle(th + prevOmega*age)
		target = ref.At(t + c.Lead)
	}
	vel = localization.Vec2{
		X: target.Vel.X + c.Kp*(target.Pos.X-p.X),
		Y: target.Vel.Y + c.Kp*(target.Pos.Y-p.Y),
	}
	omega = target.YawRate + c.Kth*localization.AngleDiff(target.Theta, th)
	return vel, omega, th
}

// limit は速度・角速度の上限と、指令の変化の上限を掛ける。dt は前回からの秒。
func limit(c Config, vel localization.Vec2, omega float64, prevVel localization.Vec2, prevOmega, dt float64) (localization.Vec2, float64) {
	if s := math.Hypot(vel.X, vel.Y); s > c.MaxSpeed {
		vel.X *= c.MaxSpeed / s
		vel.Y *= c.MaxSpeed / s
	}
	omega = math.Max(-c.MaxYawRate, math.Min(c.MaxYawRate, omega))
	if dt > 0 {
		dvx, dvy := vel.X-prevVel.X, vel.Y-prevVel.Y
		if d, maxD := math.Hypot(dvx, dvy), c.MaxAccel*dt; d > maxD {
			vel.X = prevVel.X + dvx*maxD/d
			vel.Y = prevVel.Y + dvy*maxD/d
		}
	}
	return vel, omega
}
