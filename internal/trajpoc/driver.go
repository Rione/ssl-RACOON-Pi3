package trajpoc

import (
	"fmt"
	"math"
	"sync"

	"github.com/Rione/ssl-RACOON-Pi3/internal/control"
	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi3/internal/receive"
	"github.com/Rione/ssl-RACOON-Pi3/internal/supervisor"
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
	Ref                              control.Reference // TV (撮影時刻) での参照
	Phase                            control.Phase
	CmdWorld                         localization.Vec2
	CmdOmega                         float64
	CmdBody                          localization.Vec2
	Held                             bool       // vision が古くて 0 を出した周期
	Wheels                           [4]float64 // その周期に読んだ車輪の回転速度 [rad/s] (FL, BL, BR, FR)
	ImuValid                         bool       // その周期に IMU が読めたか
	ImuYawRate, ImuAccelX, ImuAccelY float64    // [rad/s], [m/s^2] (機体座標)
	// 自己位置推定の出力 (横で回しているだけ。制御には使っていない)
	Est      localization.Estimate
	EstValid bool
	NearGoal bool               // 止まり際の √ブレーキ則で指令した周期
	Pred     localization.Pose2 // そのときのスミス予測の位置 (NearGoal のときだけ)
}

// Driver は link.VelocityOverride を満たし、SPI の周期 (125 Hz) ごとに速度を作る。
// link パッケージはビルドタグ付きなので import せず、同じ形のメソッドだけ持つ。
type Driver struct {
	mu     sync.Mutex
	cfg    Config
	rel    []Knot
	vision VisionSource
	now    Clock
	wheels WheelSource                  // nil なら記録も検査もしない
	imu    ImuSource                    // nil なら記録しない
	est    *localization.Estimator      // nil なら回さない。横で回して記録するだけ (制御には使わない)
	estTV  float64                      // 推定器に渡した最後の撮影時刻
	check  *supervisor.WheelVisionCheck // 車輪と vision の食い違いの検査 (wheels があるとき)

	state  State
	reason string
	ctl    *control.Controller // 本番と同じ追従器 (internal/control)
	eval   *control.Controller // 指標を出すための参照 (先回しの速度を必ず持つ。指令には使わない)
	path   []Knot              // 貼り付け後の点列 (輪郭誤差の評価に使う)
	endT   float64             // 軌道の末尾の時刻 (軌道の先頭を 0 とした秒)
	fence  supervisor.Fence
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
	n := int(math.Ceil((MeasureBounds(rel).Duration+cfg.Settle+cfg.StartDelay+1)*125)) + 64
	return &Driver{cfg: cfg, rel: append([]Knot(nil), rel...), vision: vision, now: now,
		samples: make([]Sample, 0, n)}, nil
}

// SetWheelSource は車輪の回転速度の読み出しを登録する。Arm の前に呼ぶ。
// 記録するほか、車輪と vision の食い違いの検査 (consistency.go) に使う。
func (d *Driver) SetWheelSource(w WheelSource) error {
	c, err := supervisor.NewWheelVisionCheck(PoCGeometry())
	if err != nil {
		return err
	}
	d.mu.Lock()
	defer d.mu.Unlock()
	d.wheels, d.check = w, c
	return nil
}

// SetEstimator は自己位置推定を横で回す (記録するだけで、指令には使わない)。Arm の前に呼ぶ。
// 車輪・IMU・vision を同じ周期で渡し、その時点の推定を記録に残す。
func (d *Driver) SetEstimator(e *localization.Estimator) {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.est, d.estTV = e, math.Inf(-1)
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
	world := ToWorld(d.rel, pose)
	t0 := now + localization.Stamp(d.cfg.StartDelay*1e9)
	nodes := ToNodes(world, t0, d.cfg.Method.FeedForward())
	// 受信の入口と同じ検証を通す (本番では RAVEN から来た点列がここを通る)。
	last := nodes[len(nodes)-1].Stamp
	plan, err := receive.PreparePlan(1, last+localization.Stamp(d.cfg.Settle*1e9), nodes, d.cfg.ControlConfig())
	if err != nil {
		return err
	}
	d.ctl, d.path = plan.Controller(), world
	// 指標は「本来の参照」と比べたいので、先回しを切った手法 (p) でも速度入りのノードで評価する。
	d.eval = d.ctl
	if !d.cfg.Method.FeedForward() {
		if d.eval, err = control.New(ToNodes(world, t0, true), d.cfg.ControlConfig()); err != nil {
			return err
		}
	}
	d.endT = world[len(world)-1].T
	d.start, d.t0 = pose, t0
	d.fence = supervisor.Fence{Start: localization.Vec2{X: pose.X, Y: pose.Y}, Radius: d.cfg.Fence + d.cfg.FenceMargin}
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

// Path は貼り付け後の点列 (Arm 前は nil)。指標の輪郭誤差に使う。
func (d *Driver) Path() []Knot {
	d.mu.Lock()
	defer d.mu.Unlock()
	return append([]Knot(nil), d.path...)
}

// Duration は軌道の長さ [s] (軌道の先頭を 0 とした末尾の時刻)。
func (d *Driver) Duration() float64 {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.endT
}

// Controller は走らせている追従器 (Arm 前は nil)。
func (d *Driver) Controller() *control.Controller {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.ctl
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
	if dist, out := d.fence.Outside(localization.Vec2{X: pose.X, Y: pose.Y}); out {
		d.abort(fmt.Sprintf("%s: %.2f m from start", supervisor.OutOfFence, dist))
		return 0, 0, 0, true
	}
	if t > d.endT+d.cfg.Settle {
		d.state, d.reason = Done, "trajectory finished"
		return 0, 0, 0, true
	}

	s := Sample{T: t, TV: (capture - d.t0).Seconds(), Age: age, Capture: capture, Pose: pose}
	if d.imu != nil {
		s.ImuYawRate, s.ImuAccelX, s.ImuAccelY, s.ImuValid = d.imu()
	}
	if d.est != nil {
		d.feedEstimator(now, capture, pose, s)
		s.Est, s.EstValid = d.est.Current(), true
	}
	if d.wheels != nil {
		s.Wheels = d.wheels()
		// vision が古いときは下の「止まって待つ」に任せる (止まった vision と比べると必ず食い違う)
		if reason, bad := d.check.Step(t, dt, s.Wheels, pose, s.TV); bad && age <= d.cfg.VisionHoldAge {
			d.abort(reason)
			return 0, 0, 0, true
		}
	}
	s.Ref, s.Phase = d.refAt(capture)
	if age > d.cfg.VisionHoldAge {
		// 古い位置で閉ループを回すと振動する。止まって新しい vision を待つ。
		s.Held = true
		d.prevVel, d.prevOmega = localization.Vec2{}, 0
		d.remember(now, localization.Vec2{}, 0)
		d.samples = append(d.samples, s)
		return 0, 0, 0, true
	}

	// 走っている間は本番の追従器に任せる。姿勢は vision の生の値 (推定器はまだ通さない)。
	//
	// 時刻は「撮影時刻」ではなく「今」を渡す。control は estimate.Stamp で参照を引くので、
	// 撮影時刻を渡すと vision の古さのぶんだけ参照まで古くなり、そのまま追従の遅れになる。
	// 本番は推定器が今まで進めた姿勢を出すので、この差は無くなる。
	est := localization.Estimate{Stamp: now, Pose: pose, Health: localization.HealthOK}
	bodyTheta := pose.Theta
	var vel localization.Vec2
	var omega float64
	if cmd, phase, err := d.ctl.Calculate(est); err != nil {
		d.abort(fmt.Sprintf("%s: %v", supervisor.CalculationFailed, err))
		return 0, 0, 0, true
	} else if phase == control.Tracking {
		vel = localization.Rotate(pose.Theta, cmd.VelBody)
		omega = cmd.YawRate
	} else {
		// 終端の後 (本番はここで 0 指令)。PoC は到着の精度を測るため最後の点へ寄せ続ける。
		goal := control.Reference{Stamp: capture, Pose: d.path[len(d.path)-1].Pose}
		vel, omega = holdCommand(d.cfg, goal, pose)
		pred := predict(pose, capture, now, d.cfg.NearGoal.Delay, d.hist)
		if v, w, st, ok := nearGoal(d.cfg.NearGoal, goal, pred, d.ngStopped); ok {
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

// feedEstimator は 1 周期ぶんの観測を推定器へ渡す。
// 車輪と IMU は同じ SPI のフレームで届くので同じ時刻 (1 周期前の転送) にし、
// vision は新しい撮影のときだけ渡す。順番は「車輪 → IMU → vision」(vision は遅れて届くため)。
func (d *Driver) feedEstimator(now, capture localization.Stamp, pose localization.Pose2, s Sample) {
	const spiPeriod = 0.008
	at := now - localization.Stamp(spiPeriod*1e9)
	if d.wheels != nil {
		d.est.AddWheel(localization.WheelSample{Stamp: at, Omega: s.Wheels})
	}
	if s.ImuValid {
		d.est.AddImu(localization.ImuSample{Stamp: at, GyroZ: s.ImuYawRate,
			Accel: localization.Vec2{X: s.ImuAccelX, Y: s.ImuAccelY}, HasGyro: true, HasAccel: true})
	}
	if s.TV > d.estTV {
		d.estTV = s.TV
		d.est.AddVision(localization.VisionPose{Stamp: capture, Pose: pose, Confidence: 1})
	}
}

// refAt は指標に使う参照。軌道の前後では端の点で静止しているものとして扱う
// (本番の control は範囲外でゼロの参照を返すが、それでは「参照との距離」を測れない)。
func (d *Driver) refAt(at localization.Stamp) (control.Reference, control.Phase) {
	r, phase := d.eval.ReferenceAt(at)
	switch phase {
	case control.Waiting:
		r = control.Reference{Stamp: at, Pose: d.path[0].Pose}
	case control.Finished:
		r = control.Reference{Stamp: at, Pose: d.path[len(d.path)-1].Pose}
	}
	return r, phase
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
