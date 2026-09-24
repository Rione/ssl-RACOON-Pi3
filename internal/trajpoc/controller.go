package trajpoc

import (
	"fmt"
	"math"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/control"
	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// Method は参照から速度を作る手法。PoC で比べる本体。
// Method は比べる追従の手法。中身は control.Config の組み合わせで表す。
//
//	p         先回し無し (位置・向きの P だけ)。参照より遅れる (実測で 294 ms)
//	ffp       参照の速度を先回し + P
//	ffp_vlead 先回しの速度を「指令が効く頃」の参照から取る (Config.Lead)。実機で最良
//
// 位置の目標まで先へずらす形 (以前の ffp_lead) は実機で効果が無かったので消した (§5-8)。
type Method string

const (
	MethodP          Method = "p"
	MethodFFP        Method = "ffp"
	MethodFFPVelLead Method = "ffp_vlead"
)

// ParseMethod は文字列を Method にする。
func ParseMethod(s string) (Method, error) {
	switch Method(s) {
	case MethodP, MethodFFP, MethodFFPVelLead:
		return Method(s), nil
	}
	return "", fmt.Errorf("unknown method %q (p|ffp|ffp_vlead)", s)
}

// FeedForward は参照の速度を先回しする手法か。
func (m Method) FeedForward() bool { return m != MethodP }

// Config は PoC の設定。ゲインは [1/s]、速度・加速度は SI。
type Config struct {
	Method Method

	Kp   float64 // 位置の P ゲイン [1/s]
	Kth  float64 // 向きの P ゲイン [1/s]
	Lead float64 // MethodFFPVelLead で先回しの速度を取る秒 (指令が効くまでの遅れ) [s]

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

	// UseEstimate は追従器に渡す姿勢を、vision の生の値ではなく自己位置推定の出力にする。
	//
	// **比較のための旗。** 時刻 (Estimate.Stamp) は両方とも「今」のままにしてあるので、
	// 変わるのは「追従器が見る姿勢」だけ。参照の引き方が変わると比較にならないため。
	// 指標は常に vision の生の値で測る (推定を真値扱いしない)。
	UseEstimate bool

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
	Delay       float64 // 指令が効くまでの遅れ [s] (PoC の実測で約 90 ms)。スミス予測の窓
	MinSpeed    float64 // 不感帯の外で出す並進の最低速度 [m/s]。これ未満だと静止摩擦で動かない (§5-14)
}

// DefaultNearGoal は RAVEN の near_goal_brake の値 (control.yaml) が元。速度の上限は PoC の上限に合わせて低い。
// 不感帯は 4 → 1.5 mm: スミス予測の位置が不感帯の縁に入った時点で止めるので、縁の分だけ手前で止まる
// (動き直すのは 4.5 mm を超えてから)。vision の揺れは 0.2〜0.4 mm なので 1.5 mm でも足りる。
func DefaultNearGoal() NearGoalConfig {
	return NearGoalConfig{Radius: 0.1, Decel: 2.5, MaxSpeed: 0.3, Deadband: 0.0015,
		HeadDecel: 15, HeadMaxRate: 2.0, HeadBand: 0.03, ExitFactor: 3, Delay: 0.09, MinSpeed: 0.03}
}

// DefaultConfig は最初の実機試験用の保守的な設定。
func DefaultConfig() Config {
	return Config{
		Method: MethodFFP,
		Kp:     3.0, Kth: 4.0, Lead: 0.03,
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

// ControlConfig は本番の追従器 (internal/control) に渡す設定にする。
// 先読みは MethodFFPVelLead のときだけ効かせる。
func (c Config) ControlConfig() control.Config {
	cfg := control.Config{
		PositionGain: c.Kp,
		HeadingGain:  c.Kth,
		MaxSpeed:     c.MaxSpeed,
		MaxYawRate:   c.MaxYawRate,
	}
	if c.Method == MethodFFPVelLead {
		cfg.VelocityLead = time.Duration(c.Lead * float64(time.Second))
	}
	return cfg
}

// holdCommand は軌道が終わった後に最後の点へ留まる指令 (ワールド系)。
//
// 本番の control は終端の後にゼロを返す (計画側が止まる軌道を作る前提)。PoC は到着の精度を測るために、
// 終端の姿勢へ P で寄せ続ける。この保持は実験のためのもので、本番の経路には入れていない。
func holdCommand(c Config, goal control.Reference, pose localization.Pose2) (vel localization.Vec2, omega float64) {
	vel = localization.Vec2{
		X: c.Kp * (goal.Pose.X - pose.X),
		Y: c.Kp * (goal.Pose.Y - pose.Y),
	}
	omega = c.Kth * localization.AngleDiff(goal.Pose.Theta, pose.Theta)
	return vel, omega
}

// nearGoal は軌道が終わった後の寄せ方。pose は vision の位置に「出したがまだ vision に現れていない指令」を
// 足したスミス予測の位置 (今の指令が効き始める頃の位置)。予測の位置で判断するので、止める判断のときに
// まだ効いていない指令を見落とさない (v1 は vision の位置で止め、遅れて効いた指令で 12 mm 流れた)。
// ゴールまで Radius 以内なら √ブレーキ則の速度 (下限 MinSpeed) でゴールへ向かい、不感帯に入ったら止める。
// stopped は前回止めていたか (ヒステリシス用)。使わないとき (範囲外・無効) は ok=false。
func nearGoal(c NearGoalConfig, goal control.Reference, pose localization.Pose2, stopped bool) (vel localization.Vec2, omega float64, nowStopped, ok bool) {
	dx, dy := goal.Pose.X-pose.X, goal.Pose.Y-pose.Y
	d := math.Hypot(dx, dy)
	he := localization.AngleDiff(goal.Pose.Theta, pose.Theta)
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
		v := math.Min(c.MaxSpeed, math.Max(c.MinSpeed, stoppableSpeed(d, c.Decel, 0)))
		vel = localization.Vec2{X: dx / d * v, Y: dy / d * v}
	}
	if math.Abs(he) >= hband {
		omega = math.Copysign(math.Min(c.HeadMaxRate, stoppableSpeed(math.Abs(he)-c.HeadBand, c.HeadDecel, 0)), he)
	}
	return vel, omega, false, true
}

// predict はスミス予測: vision の撮影時刻から遅れ delay を引いた時刻より後に出した指令は、まだ vision に
// 現れていない。その分を vision の位置に足す。hist は出した指令 (古い順、各指令は次の指令まで続く)。
func predict(pose localization.Pose2, capture, now localization.Stamp, delay float64, hist []cmdRecord) localization.Pose2 {
	from := capture - localization.Stamp(delay*1e9)
	for i, h := range hist {
		end := now
		if i+1 < len(hist) {
			end = hist[i+1].at
		}
		a := h.at
		if a < from {
			a = from
		}
		if end <= a {
			continue
		}
		dt := (end - a).Seconds()
		pose.X += h.vel.X * dt
		pose.Y += h.vel.Y * dt
		pose.Theta = localization.WrapAngle(pose.Theta + h.omega*dt)
	}
	return pose
}

// cmdRecord は出した指令 (world 系)。スミス予測に使う。
type cmdRecord struct {
	at    localization.Stamp
	vel   localization.Vec2
	omega float64
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
