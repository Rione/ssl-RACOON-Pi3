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
	// MethodFFPVelLead は速度の先回しだけを Lead 先の参照から取る。位置の目標は今の時刻のまま、
	// vision の位置の外挿もしない。指令が効くまでの遅れで速度の向きが古くなり、曲がりで外へ
	// 膨らむ (と考えられる) 分だけを打ち消す。遅れと横滑りを見分けるための手法。
	MethodFFPVelLead Method = "ffp_vlead"
)

// ParseMethod は文字列を Method にする。
func ParseMethod(s string) (Method, error) {
	switch Method(s) {
	case MethodP, MethodFFP, MethodFFPLead, MethodFFPVelLead:
		return Method(s), nil
	}
	return "", fmt.Errorf("unknown method %q (p|ffp|ffp_lead|ffp_vlead)", s)
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

	// NearGoal は軌道が終わった後の寄せ方を √ブレーキ則 + 不感帯に替える (RAVEN の near-goal brake と同じ考え)。
	// 位置の P のままだと、数 mm の誤差では指令が小さすぎて静止摩擦に負け、寄せきれない上に、
	// 小さな並進の指令で機体が回されて向きが振動する (traj-poc-log §5-13)。
	NearGoal NearGoalConfig
}

// NearGoalConfig は止まり際の寄せ方。距離は m、角度は rad。
type NearGoalConfig struct {
	Enabled     bool
	Radius      float64 // これより近ければ √ブレーキ則 [m]
	Decel       float64 // √ブレーキ則の減速度 [m/s^2]
	MaxSpeed    float64 // √ブレーキ則の速度の上限 [m/s]
	Deadband    float64 // これより近ければ止める [m]
	HeadDecel   float64 // 向きの √ブレーキ則の角減速度 [rad/s^2]
	HeadMaxRate float64 // 向きの角速度の上限 [rad/s]
	HeadBand    float64 // これより向きが近ければ回さない [rad]
	ExitFactor  float64 // 止めた後、不感帯のこの倍を超えたら動き直す (ヒステリシス)
	Delay       float64 // 指令が効くまでの遅れ [s]。止まれる速度をこの分だけ控える (PoC の実測で約 90 ms)
}

// DefaultNearGoal は RAVEN の near_goal_brake の値 (control.yaml) に合わせたもの。速度の上限だけ PoC の上限に合わせて低い。
func DefaultNearGoal() NearGoalConfig {
	return NearGoalConfig{Radius: 0.1, Decel: 2.5, MaxSpeed: 0.3, Deadband: 0.004,
		HeadDecel: 15, HeadMaxRate: 2.0, HeadBand: 0.03, ExitFactor: 2.5, Delay: 0.09}
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
		NearGoal: DefaultNearGoal(),
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
	case MethodFFPVelLead:
		ahead := ref.At(t + c.Lead)
		target.Vel, target.YawRate = ahead.Vel, ahead.YawRate
	}
	vel = localization.Vec2{
		X: target.Vel.X + c.Kp*(target.Pos.X-p.X),
		Y: target.Vel.Y + c.Kp*(target.Pos.Y-p.Y),
	}
	omega = target.YawRate + c.Kth*localization.AngleDiff(target.Theta, th)
	return vel, omega, th
}

// nearGoal は軌道が終わった後の寄せ方。ゴールまで Radius 以内なら、遅れ Td を入れた止まれる速度
// v = a·(√(Td² + 2(d−不感帯)/a) − Td) でゴールへ向かい、不感帯に入ったら止める。
// (素の √ブレーキ則 √(2a(d−不感帯)) は「今すぐ減速し始められる」前提で、90 ms の遅れの間に
// 不感帯を越えて行ったり来たりする。遅れの間は今の速度で進むとして、その分だけ控える。)
// stopped は前回止めていたか (ヒステリシス用)。
// 使わないとき (範囲外・無効) は ok=false。
func nearGoal(c NearGoalConfig, goal RefSample, pose localization.Pose2, stopped bool) (vel localization.Vec2, omega float64, nowStopped, ok bool) {
	dx, dy := goal.Pos.X-pose.X, goal.Pos.Y-pose.Y
	d := math.Hypot(dx, dy)
	he := localization.AngleDiff(goal.Theta, pose.Theta)
	if !c.Enabled || d > c.Radius {
		return localization.Vec2{}, 0, false, false
	}
	band, hband := c.Deadband, c.HeadBand
	if stopped {
		band, hband = band*c.ExitFactor, hband*c.ExitFactor
	}
	if d < band && math.Abs(he) < hband {
		return localization.Vec2{}, 0, true, true
	}
	if d >= band {
		v := math.Min(c.MaxSpeed, stoppableSpeed(d-c.Deadband, c.Decel, c.Delay))
		vel = localization.Vec2{X: dx / d * v, Y: dy / d * v}
	}
	if math.Abs(he) >= hband {
		omega = math.Copysign(math.Min(c.HeadMaxRate, stoppableSpeed(math.Abs(he)-c.HeadBand, c.HeadDecel, c.Delay)), he)
	}
	return vel, omega, false, true
}

// stoppableSpeed は、遅れ td の間は今の速度で進み、その後に減速度 a で止まるとき、
// 残り s でちょうど止まれる速度 (v·td + v²/(2a) = s の正の解)。
func stoppableSpeed(s, a, td float64) float64 {
	if s <= 0 {
		return 0
	}
	return a * (math.Sqrt(td*td+2*s/a) - td)
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
