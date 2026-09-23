package trajpoc

import (
	"fmt"
	"math"
	"sync"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// VisionSource は自機の最新の vision 姿勢と、その撮影時刻を返す。
// 撮影時刻は Clock と同じ時間軸 (Rock5A の単調時計) で渡すこと。
type VisionSource func() (pose localization.Pose2, capture localization.Stamp, ok bool)

// WheelSource は STM から届いた最新の 4 輪の回転速度 [rad/s] を FL, BL, BR, FR の順で返す。
// 記録するだけで制御には使わない (スリップの切り分け用、traj-poc-log §5-16)。
type WheelSource func() [4]float64

// ImuSource は STM から届いた最新の IMU を返す (ヨーの角速度 [rad/s]、機体座標の加速度 [m/s^2])。
// IMU の無いファームでは ok=false。記録するだけで制御には使わない。
type ImuSource func() (yawRate, accelX, accelY float64, ok bool)

// Clock は現在の単調時刻を返す。
type Clock func() localization.Stamp

// State は Driver の状態。
type State uint8

const (
	Idle    State = iota // Arm 前。0 を出す
	Running              // 追従中
	Done                 // 軌道 + Settle が終わった。0 を出す
	Aborted              // 安全のために打ち切った。0 を出す
	Stopped              // 外から止めた (Ctrl+C・SSH 切断など)。0 を出す
)

func (s State) String() string {
	return [...]string{"idle", "running", "done", "aborted", "stopped"}[s]
}

// Sample は 1 周期の記録。T は現在の軌道時刻、TV は vision の撮影時刻での軌道時刻。
// 誤差は「撮影した瞬間の参照」と比べる (vision の遅れを追従の誤差に混ぜないため)。
type Sample struct {
	T, TV                            float64
	Age                              float64
	Capture                          localization.Stamp
	Pose                             localization.Pose2
	Ref                              RefSample // TV での参照
	CmdWorld                         localization.Vec2
	CmdOmega                         float64
	CmdBody                          localization.Vec2
	Held                             bool               // vision が古くて 0 を出した周期
	Wheels                           [4]float64         // その周期に読んだ車輪の回転速度 [rad/s] (FL, BL, BR, FR)
	ImuValid                         bool               // その周期に IMU が読めたか
	ImuYawRate, ImuAccelX, ImuAccelY float64            // [rad/s], [m/s^2] (機体座標)
	NearGoal                         bool               // 止まり際の √ブレーキ則で指令した周期
	Pred                             localization.Pose2 // そのときのスミス予測の位置 (NearGoal のときだけ)
}

// Driver は link.VelocityOverride を満たし、SPI の周期 (125 Hz) ごとに速度を作る。
// link パッケージはビルドタグ付きなので import せず、同じ形のメソッドだけ持つ。
type Driver struct {
	mu     sync.Mutex
	cfg    Config
	rel    []Knot
	vision VisionSource
	now    Clock
	wheels WheelSource       // nil なら記録も検査もしない
	imu    ImuSource         // nil なら記録しない
	check  *wheelVisionCheck // 車輪と vision の食い違いの検査 (wheels があるとき)

	state  State
	reason string
	ref    *Reference
	start  localization.Pose2
	t0     localization.Stamp

	prevVel   localization.Vec2
	prevOmega float64
	ngStopped bool        // 止まり際の不感帯で止めているか
	hist      []cmdRecord // 直近に出した指令 (スミス予測用)
	prevTick  localization.Stamp
	samples   []Sample
}

// NewDriver は相対軌道と設定を検証して Driver を作る。まだ動かない (Arm で動く)。
func NewDriver(rel []Knot, cfg Config, vision VisionSource, now Clock) (*Driver, error) {
	if err := cfg.Validate(rel); err != nil {
		return nil, err
	}
	if _, err := NewReference(rel, cfg.Interp); err != nil {
		return nil, err
	}
	n := int(math.Ceil((MeasureBounds(rel).Duration+cfg.Settle+cfg.StartDelay+1)*125)) + 64
	return &Driver{cfg: cfg, rel: append([]Knot(nil), rel...), vision: vision, now: now,
		samples: make([]Sample, 0, n)}, nil
}

// SetWheelSource は車輪の回転速度の読み出しを登録する。Arm の前に呼ぶ。
// 記録するほか、車輪と vision の食い違いの検査 (consistency.go) に使う。
func (d *Driver) SetWheelSource(w WheelSource) error {
	c, err := newWheelVisionCheck(PoCGeometry())
	if err != nil {
		return err
	}
	d.mu.Lock()
	defer d.mu.Unlock()
	d.wheels, d.check = w, c
	return nil
}

// SetImuSource は IMU の読み出しを登録する (記録用)。Arm の前に呼ぶ。
func (d *Driver) SetImuSource(i ImuSource) {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.imu = i
}

// Arm は現在の vision 姿勢に軌道を貼り付け、StartDelay 後を軌道の t=0 にして走らせる。
func (d *Driver) Arm() error {
	d.mu.Lock()
	defer d.mu.Unlock()
	if d.state != Idle {
		return fmt.Errorf("cannot arm in state %s", d.state)
	}
	pose, capture, ok := d.vision()
	now := d.now()
	if !ok {
		return fmt.Errorf("no vision of the robot")
	}
	if age := (now - capture).Seconds(); age > d.cfg.VisionHoldAge {
		return fmt.Errorf("vision is stale (%.0f ms)", age*1000)
	}
	// 軌道の先頭の向きは相対 0。ロボットの今の向きを基準にする。
	ref, err := NewReference(ToWorld(d.rel, pose), d.cfg.Interp)
	if err != nil {
		return err
	}
	d.ref, d.start = ref, pose
	d.t0 = now + localization.Stamp(d.cfg.StartDelay*1e9)
	d.prevTick = now
	d.state = Running
	return nil
}

// Stop は外から止める (Ctrl+C・SSH 切断など)。以後は 0 を出す。
func (d *Driver) Stop(reason string) {
	d.mu.Lock()
	defer d.mu.Unlock()
	if d.state == Running || d.state == Idle {
		d.state, d.reason = Stopped, reason
	}
}

// Status は状態と、止まった理由を返す。
func (d *Driver) Status() (State, string) {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.state, d.reason
}

// Finished は Running でなくなったか (Idle は含まない)。
func (d *Driver) Finished() bool {
	s, _ := d.Status()
	return s == Done || s == Aborted || s == Stopped
}

// Start は軌道を貼り付けた開始姿勢を返す。
func (d *Driver) Start() localization.Pose2 {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.start
}

// Reference は貼り付け後の参照 (Arm 前は nil)。
func (d *Driver) Reference() *Reference {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.ref
}

// Samples は記録のコピーを返す。
func (d *Driver) Samples() []Sample {
	d.mu.Lock()
	defer d.mu.Unlock()
	return append([]Sample(nil), d.samples...)
}

func (d *Driver) abort(reason string) {
	d.state, d.reason = Aborted, reason
}

// OverrideVelocity は link.VelocityOverride を満たす。ロボット系の
// VelX, VelY [mm/s] と VelAng [mrad/s]。Running 以外は 0 を出し続ける
// (指令を返さなくなると通常経路へ戻り、直前の速度が残っていると走り去るため)。
func (d *Driver) OverrideVelocity() (velX, velY, velAng int16, ok bool) {
	d.mu.Lock()
	defer d.mu.Unlock()
	if d.state != Running {
		return 0, 0, 0, true
	}
	now := d.now()
	dt := math.Min((now - d.prevTick).Seconds(), 0.05)
	d.prevTick = now
	t := (now - d.t0).Seconds()

	pose, capture, vok := d.vision()
	age := (now - capture).Seconds()
	if !vok || age > d.cfg.VisionAbortAge {
		d.abort(fmt.Sprintf("vision lost (%.0f ms)", age*1000))
		return 0, 0, 0, true
	}
	if math.Hypot(pose.X-d.start.X, pose.Y-d.start.Y) > d.cfg.Fence+d.cfg.FenceMargin {
		d.abort(fmt.Sprintf("left the fence (%.2f m from start)", math.Hypot(pose.X-d.start.X, pose.Y-d.start.Y)))
		return 0, 0, 0, true
	}
	if t > d.ref.End()+d.cfg.Settle {
		d.state, d.reason = Done, "trajectory finished"
		return 0, 0, 0, true
	}

	s := Sample{T: t, TV: (capture - d.t0).Seconds(), Age: age, Capture: capture, Pose: pose}
	if d.imu != nil {
		s.ImuYawRate, s.ImuAccelX, s.ImuAccelY, s.ImuValid = d.imu()
	}
	if d.wheels != nil {
		s.Wheels = d.wheels()
		// vision が古いときは下の「止まって待つ」に任せる (止まった vision と比べると必ず食い違う)
		if reason, bad := d.check.step(t, dt, s.Wheels, pose, s.TV); bad && age <= d.cfg.VisionHoldAge {
			d.abort(reason)
			return 0, 0, 0, true
		}
	}
	s.Ref = d.ref.At(s.TV)
	if age > d.cfg.VisionHoldAge {
		// 古い位置で閉ループを回すと振動する。止まって新しい vision を待つ。
		s.Held = true
		d.prevVel, d.prevOmega = localization.Vec2{}, 0
		d.remember(now, localization.Vec2{}, 0)
		d.samples = append(d.samples, s)
		return 0, 0, 0, true
	}

	vel, omega, bodyTheta := command(d.cfg, d.ref, t, pose, age, d.prevVel, d.prevOmega)
	if t > d.ref.End() {
		pred := predict(pose, capture, now, d.cfg.NearGoal.Delay, d.hist)
		if v, w, st, ok := nearGoal(d.cfg.NearGoal, d.ref.At(t), pred, d.ngStopped); ok {
			vel, omega, d.ngStopped, bodyTheta = v, w, st, pred.Theta
			s.NearGoal, s.Pred = true, pred
		}
	}
	vel, omega = limit(d.cfg, vel, omega, d.prevVel, d.prevOmega, dt)
	d.prevVel, d.prevOmega = vel, omega
	d.remember(now, vel, omega)
	body := localization.RotateInv(bodyTheta, vel)
	s.CmdWorld, s.CmdOmega, s.CmdBody = vel, omega, body
	d.samples = append(d.samples, s)
	return toInt16(body.X * 1000), toInt16(body.Y * 1000), toInt16(omega * 1000), true
}

// remember は出した指令を覚え、スミス予測に要らなくなった古いものを捨てる
// (vision の古さの上限 VisionHoldAge + 遅れ より前は使わない)。
func (d *Driver) remember(now localization.Stamp, vel localization.Vec2, omega float64) {
	d.hist = append(d.hist, cmdRecord{at: now, vel: vel, omega: omega})
	keep := now - localization.Stamp((d.cfg.VisionHoldAge+d.cfg.NearGoal.Delay+0.05)*1e9)
	i := 0
	for i+1 < len(d.hist) && d.hist[i+1].at <= keep {
		i++
	}
	d.hist = d.hist[i:]
}

func toInt16(v float64) int16 {
	if math.IsNaN(v) {
		return 0
	}
	return int16(math.Max(math.MinInt16, math.Min(math.MaxInt16, math.Round(v))))
}
