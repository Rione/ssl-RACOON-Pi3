package trajpoc

import (
	"bytes"
	"math"
	"strings"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/control"
	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

func TestReadTrajectory(t *testing.T) {
	in := "# comment\n{\"x\":0,\"y\":0,\"theta\":0,\"t\":0}\n\n{\"x\":100,\"y\":-50,\"theta\":0.5,\"t\":16000000}\nEND\nignored\n"
	k, err := ReadTrajectory(strings.NewReader(in))
	if err != nil {
		t.Fatal(err)
	}
	if len(k) != 2 || k[1].Pose.X != 0.1 || k[1].Pose.Y != -0.05 || k[1].Pose.Theta != 0.5 || math.Abs(k[1].T-0.016) > 1e-12 {
		t.Fatalf("unexpected knots %+v", k)
	}
	if _, err := ReadTrajectory(strings.NewReader("{\"x\":0,\"y\":0,\"theta\":0,\"t\":0}\n")); err == nil {
		t.Fatal("missing END must be an error (EOF means the sender went away)")
	}
	bad := "{\"x\":0,\"y\":0,\"theta\":0,\"t\":0}\n{\"x\":0,\"y\":0,\"theta\":0,\"t\":0}\nEND\n"
	if _, err := ReadTrajectory(strings.NewReader(bad)); err == nil {
		t.Fatal("non-increasing t must be rejected")
	}
}

func TestGenerateShapesRoundTrip(t *testing.T) {
	for _, shape := range []string{"line", "square", "circle", "fig8"} {
		c := DefaultGenConfig()
		c.Shape = shape
		k, err := Generate(c)
		if err != nil {
			t.Fatalf("%s: %v", shape, err)
		}
		b := MeasureBounds(k)
		last := k[len(k)-1].Pose
		if b.StartOffset != 0 || math.Hypot(last.X, last.Y) > 1e-6 {
			t.Errorf("%s: must start and end at the origin (end %.4f,%.4f)", shape, last.X, last.Y)
		}
		maxR := c.Size
		if shape == "square" {
			maxR = c.Size * math.Sqrt2 // 対角の角が一番遠い
		}
		if b.MaxSpeed > c.Speed*1.01 || b.MaxRadius > maxR+1e-9 {
			t.Errorf("%s: bounds %+v exceed speed %.2f / size %.2f", shape, b, c.Speed, c.Size)
		}
		// 書いて読み戻すと同じ (mm と ns の丸めの範囲で)。
		var buf bytes.Buffer
		if err := WriteJSONL(&buf, k, "test"); err != nil {
			t.Fatal(err)
		}
		k2, err := ReadTrajectory(&buf)
		if err != nil || len(k2) != len(k) {
			t.Fatalf("%s: round trip failed: %v", shape, err)
		}
		if err := DefaultConfig().Validate(k2); err != nil {
			t.Errorf("%s: default generator output must pass the default safety limits: %v", shape, err)
		}
	}
	turn, err := Generate(GenConfig{Shape: "turn", Size: math.Pi / 2, Speed: 1.0, Accel: 3.0, Dt: 0.016, Heading: "fixed", Laps: 1})
	if err != nil {
		t.Fatal(err)
	}
	tb := MeasureBounds(turn)
	mid := turn[len(turn)/2].Pose.Theta
	if tb.MaxRadius != 0 || math.Abs(mid-math.Pi/2) > 0.02 || math.Abs(turn[len(turn)-1].Pose.Theta) > 1e-9 || tb.MaxYawRate > 1.01 {
		t.Errorf("turn must spin in place to +90 deg and back: mid %.3f rad, bounds %+v", mid, tb)
	}
	if _, err := Generate(GenConfig{Shape: "square", Size: 0.5, Speed: 0.4, Accel: 1, Dt: 0.016, Heading: "tangent", Laps: 1}); err == nil {
		t.Error("tangent heading on a square (turns in place) must be rejected")
	}
}

func TestToNodesFillsVelocities(t *testing.T) {
	// 等速直線 0.2 m/s。中心差分なので内側の点は厳密、端は片側差分。
	var k []Knot
	for i := 0; i <= 10; i++ {
		k = append(k, Knot{T: float64(i) * 0.1, Pose: localization.Pose2{X: 0.2 * float64(i) * 0.1}})
	}
	nodes := ToNodes(k, localization.Stamp(time.Second), true)
	if len(nodes) != len(k) {
		t.Fatalf("got %d nodes, want %d", len(nodes), len(k))
	}
	if math.Abs(nodes[5].VelBody.X-0.2) > 1e-9 || math.Abs(nodes[5].VelBody.Y) > 1e-9 {
		t.Errorf("node velocity %+v, want (0.2, 0)", nodes[5].VelBody)
	}
	if nodes[0].Stamp != localization.Stamp(time.Second) {
		t.Errorf("first node stamp %v, want t0", nodes[0].Stamp)
	}
	// 先回し無し (p) では速度を 0 にする
	zero := ToNodes(k, 0, false)
	if zero[5].VelBody != (localization.Vec2{}) || zero[5].YawRate != 0 {
		t.Errorf("without feed-forward the node velocity must be zero, got %+v", zero[5])
	}
	// control の参照は、その速度をそのまま返す
	c, err := control.New(nodes, DefaultConfig().ControlConfig())
	if err != nil {
		t.Fatal(err)
	}
	ref, phase := c.ReferenceAt(localization.Stamp(1.537 * float64(time.Second)))
	if phase != control.Tracking || math.Abs(ref.Pose.X-0.1074) > 1e-9 || math.Abs(ref.VelWorld.X-0.2) > 1e-9 {
		t.Errorf("reference %+v phase %v", ref, phase)
	}
}

// simRobot は簡易な機体: 指令は Dead だけ遅れて届き、実速度は Tau の一次遅れで追う。
// vision は VisionPeriod ごとに撮影し、Latency 後に届く。
// stiction > 0 なら、止まっている機体はそれ未満の並進の指令では動かない (静止摩擦、traj-poc-log §5-14)。
type simRobot struct {
	stiction                    float64
	wheelScale, wheelRotScale   float64 // 車輪の読みの倍率 (寸法の誤差の代わり)。0 なら 1
	now                         localization.Stamp
	pose                        localization.Pose2
	vel                         localization.Vec2 // world
	omega                       float64
	dead, tau                   float64
	cmdQueue                    []simCmd
	visionPeriod, latency       float64
	frames                      []simFrame
	lastShot                    localization.Stamp
	blackoutFrom, blackoutUntil float64
}

type simCmd struct {
	at    localization.Stamp
	body  localization.Vec2
	omega float64
}

type simFrame struct {
	capture, arrive localization.Stamp
	pose            localization.Pose2
}

func sec(s float64) localization.Stamp { return localization.Stamp(s * 1e9) }

// wheels は機体の今の速度から、既定の機体パラメータで車輪の回転速度 (SPI の並び) を作る。
func (r *simRobot) wheels() [4]float64 {
	k, err := localization.NewKinematics(localization.DefaultGeometry())
	if err != nil {
		panic(err)
	}
	ts, rs := r.wheelScale, r.wheelRotScale
	if ts == 0 {
		ts = 1
	}
	if rs == 0 {
		rs = 1
	}
	b := localization.RotateInv(r.pose.Theta, r.vel)
	return k.WheelFromBody(b.X*ts, b.Y*ts, r.omega*rs)
}

func (r *simRobot) clock() localization.Stamp { return r.now }

func (r *simRobot) vision() (localization.Pose2, localization.Stamp, bool) {
	var best *simFrame
	for i := range r.frames {
		if r.frames[i].arrive <= r.now {
			best = &r.frames[i]
		}
	}
	if best == nil {
		return localization.Pose2{}, 0, false
	}
	return best.pose, best.capture, true
}

// step は dt 進める。applied は今届いている指令 (Dead 前に出したもの)。
func (r *simRobot) step(dt float64) {
	var target simCmd
	for _, c := range r.cmdQueue {
		if c.at <= r.now-sec(r.dead) {
			target = c
		}
	}
	if math.Hypot(target.body.X, target.body.Y) < r.stiction && math.Hypot(r.vel.X, r.vel.Y) < 0.002 {
		target.body = localization.Vec2{}
	}
	tv := localization.Rotate(r.pose.Theta, target.body)
	a := dt / r.tau
	r.vel.X += a * (tv.X - r.vel.X)
	r.vel.Y += a * (tv.Y - r.vel.Y)
	r.omega += a * (target.omega - r.omega)
	r.pose.X += r.vel.X * dt
	r.pose.Y += r.vel.Y * dt
	r.pose.Theta = localization.WrapAngle(r.pose.Theta + r.omega*dt)
	r.now += sec(dt)
	t := r.now.Seconds()
	if r.now-r.lastShot >= sec(r.visionPeriod) && !(t >= r.blackoutFrom && t < r.blackoutUntil) {
		r.lastShot = r.now
		r.frames = append(r.frames, simFrame{capture: r.now, arrive: r.now + sec(r.latency), pose: r.pose})
	}
}

func newSim(start localization.Pose2) *simRobot {
	r := &simRobot{pose: start, dead: 0.04, tau: 0.06, visionPeriod: 1.0 / 70, latency: 0.03,
		blackoutFrom: math.Inf(1), blackoutUntil: math.Inf(1)}
	for i := 0; i < 30; i++ { // 最初の vision が届くまで助走する (撮影間隔 + 遅延 > 40 ms)
		r.step(0.004)
	}
	return r
}

// run は 125 Hz の SPI 周期で Driver を回す (1 周期を 2 ms 刻みで積分)。
func run(t *testing.T, r *simRobot, d *Driver, maxSec float64) {
	t.Helper()
	if d.wheels == nil {
		if err := d.SetWheelSource(r.wheels); err != nil {
			t.Fatal(err)
		}
	}
	if err := d.Arm(); err != nil {
		t.Fatal(err)
	}
	for r.now.Seconds() < maxSec && !d.Finished() {
		vx, vy, w, ok := d.OverrideVelocity()
		if !ok {
			t.Fatal("override must always return ok")
		}
		body := localization.Vec2{X: float64(vx) / 1000, Y: float64(vy) / 1000}
		if math.Hypot(body.X, body.Y) > d.cfg.MaxSpeed+0.002 {
			t.Fatalf("command %.3f m/s exceeds the speed limit", math.Hypot(body.X, body.Y))
		}
		r.cmdQueue = append(r.cmdQueue, simCmd{at: r.now, body: body, omega: float64(w) / 1000})
		for i := 0; i < 4; i++ {
			r.step(0.002)
		}
	}
}

func TestDriverClosedLoopComparesMethods(t *testing.T) {
	gen := DefaultGenConfig()
	gen.Shape = "fig8"
	gen.Size = 0.5
	gen.Speed = 0.45
	rel, err := Generate(gen)
	if err != nil {
		t.Fatal(err)
	}
	results := map[Method]Metrics{}
	for _, m := range []Method{MethodP, MethodFFP, MethodFFPVelLead} {
		cfg := DefaultConfig()
		cfg.Method = m
		r := newSim(localization.Pose2{X: 1.0, Y: -0.5, Theta: 0.7})
		d, err := NewDriver(rel, cfg, r.vision, r.clock)
		if err != nil {
			t.Fatal(err)
		}
		run(t, r, d, 60)
		st, reason := d.Status()
		if st != Done {
			t.Fatalf("%s ended in %s: %s", m, st, reason)
		}
		if vx, vy, w, _ := d.OverrideVelocity(); vx != 0 || vy != 0 || w != 0 {
			t.Fatal("after the end the driver must keep sending zero")
		}
		met := ComputeMetrics(d.Samples(), d.Path())
		t.Logf("%-10s pos RMS %5.1f mm  contour RMS %5.1f  lag %4.0f ms  final %4.1f mm  cmd accel %4.2f",
			m, met.PosRMS, met.ContourRMS, met.LagMs, met.FinalPos, met.CmdAccelRMS)
		results[m] = met
	}
	// 参照速度を先回しすれば、P だけより遅れが明確に小さい。
	if results[MethodFFP].LagMs >= results[MethodP].LagMs*0.5 {
		t.Errorf("ffp lag %.0f ms should be well below p lag %.0f ms", results[MethodFFP].LagMs, results[MethodP].LagMs)
	}
	// 遅れ補償の先読み時間は機体の遅れ次第で、大きすぎると先走る。ここでは合否にせず、
	// 感度だけを出す (値は仮の機体モデルのものなので、実機で振って決める)。
	for _, lead := range []float64{0, 0.04, 0.08, 0.12} {
		cfg := DefaultConfig()
		cfg.Method, cfg.Lead = MethodFFPVelLead, lead
		r := newSim(localization.Pose2{X: 1.0, Y: -0.5, Theta: 0.7})
		d, _ := NewDriver(rel, cfg, r.vision, r.clock)
		run(t, r, d, 60)
		met := ComputeMetrics(d.Samples(), d.Path())
		t.Logf("ffp_vlead lead=%3.0fms  pos RMS %5.1f mm  contour RMS %5.1f  lag %4.0f ms",
			lead*1000, met.PosRMS, met.ContourRMS, met.LagMs)
	}
}

// 円を回ると、速度の向きが遅れて効く分だけ外へ膨らむ。速度の先回しだけを先の参照から取れば
// (ffp_vlead) 膨らみは消えるはず。仮の機体 (むだ時間 40 ms + 一次遅れ 60 ms) で確かめる。
func TestVelocityLeadRemovesOutwardDrift(t *testing.T) {
	gen := DefaultGenConfig()
	gen.Shape, gen.Size, gen.Speed = "circle", 0.6, 0.4
	rel, err := Generate(gen)
	if err != nil {
		t.Fatal(err)
	}
	drift := func(m Method, lead float64) float64 {
		cfg := DefaultConfig()
		cfg.Method, cfg.Lead = m, lead
		r := newSim(localization.Pose2{})
		d, err := NewDriver(rel, cfg, r.vision, r.clock)
		if err != nil {
			t.Fatal(err)
		}
		run(t, r, d, 60)
		// 相対の円の中心 (0, 0.3) は開始姿勢 (原点・向き 0) のままワールドでも同じ。
		var sum float64
		var n int
		for _, s := range d.Samples() {
			if s.TV > 1.0 && s.TV < d.Duration()-1.0 {
				sum += math.Hypot(s.Pose.X, s.Pose.Y-0.3) - 0.3
				n++
			}
		}
		return sum / float64(n) * 1000
	}
	base := drift(MethodFFP, 0)
	t.Logf("circle 0.4 m/s: ffp radial drift %+.1f mm", base)
	for _, lead := range []float64{0.05, 0.1, 0.15} {
		t.Logf("circle 0.4 m/s: ffp_vlead lead=%3.0fms radial drift %+.1f mm", lead*1000, drift(MethodFFPVelLead, lead))
	}
	if base < 2 {
		t.Errorf("the simulated robot should drift outward with plain ffp (got %+.1f mm)", base)
	}
	if v := drift(MethodFFPVelLead, 0.1); math.Abs(v) > math.Abs(base)*0.5 {
		t.Errorf("velocity lead of the plant delay should remove most of the drift: %+.1f mm vs %+.1f mm", v, base)
	}
}

func TestNearGoalSettlesAndStops(t *testing.T) {
	line := DefaultGenConfig()
	line.Size, line.Speed = 0.3, 0.2
	circle := DefaultGenConfig()
	circle.Shape, circle.Size, circle.Speed = "circle", 0.6, 0.4
	for _, tc := range []struct {
		name     string
		gen      GenConfig
		stiction float64
		dead     float64 // 0 なら newSim の既定 (40 ms)
	}{
		{"line", line, 0, 0},
		{"line/stiction", line, 0.02, 0},
		{"circle/stiction", circle, 0.02, 0},
		{"circle/stiction/slower", circle, 0.02, 0.08}, // 実際の遅れが想定 (90 ms) より 40 ms 長い
	} {
		t.Run(tc.name, func(t *testing.T) {
			rel, _ := Generate(tc.gen)
			cfg := DefaultConfig()
			cfg.Method, cfg.Lead, cfg.NearGoal.Enabled = MethodFFPVelLead, 0.085, true
			r := newSim(localization.Pose2{X: 0.3, Y: 0.2, Theta: 1.0})
			r.stiction = tc.stiction
			if tc.dead > 0 {
				r.dead = tc.dead
			}
			d, err := NewDriver(rel, cfg, r.vision, r.clock)
			if err != nil {
				t.Fatal(err)
			}
			run(t, r, d, 30)
			if st, reason := d.Status(); st != Done {
				t.Fatalf("ended in %s: %s", st, reason)
			}
			met := ComputeMetrics(d.Samples(), d.Path())
			smp := d.Samples()
			t.Logf("final %.1f mm %.2f deg, settle swing ±%.2f deg, settle cmd max %.0f mm/s",
				met.FinalPos, met.FinalHead, met.SettleHeadSwing, met.SettleCmdMax)
			if met.FinalPos > 2.5 {
				t.Errorf("must settle within the deadband: final %.1f mm", met.FinalPos)
			}
			last := smp[len(smp)-1]
			if !last.NearGoal || last.CmdWorld.X != 0 || last.CmdWorld.Y != 0 || last.CmdOmega != 0 {
				t.Errorf("once inside the deadband the command must be exactly zero: %+v", last)
			}
		})
	}
}

func TestPredictAddsCommandsNotYetSeen(t *testing.T) {
	// 0.1 m/s を 0 s から出し続け、vision は 0.2 s に撮影、今 0.25 s、遅れ 0.09 s。
	// vision に現れていないのは 0.11 s 以降に出した分 = 0.14 s × 0.1 m/s = 14 mm。
	hist := []cmdRecord{{at: 0, vel: localization.Vec2{X: 0.1}}}
	p := predict(localization.Pose2{}, sec(0.2), sec(0.25), 0.09, hist)
	if math.Abs(p.X-0.014) > 1e-9 || p.Y != 0 {
		t.Errorf("predicted %+v, want x=0.014", p)
	}
	// 0.15 s に 0 へ変えたら 0.11〜0.15 s の 4 mm だけ
	hist = append(hist, cmdRecord{at: sec(0.15)})
	p = predict(localization.Pose2{}, sec(0.2), sec(0.25), 0.09, hist)
	if math.Abs(p.X-0.004) > 1e-9 {
		t.Errorf("predicted %+v, want x=0.004", p)
	}
}

func TestDriverAbortsWhenVisionDoesNotFollowWheels(t *testing.T) {
	// 実機で起きたこと: vision の模様が止まっている別のロボットのもので、ロボットは見張られずに走った。
	for _, shape := range []string{"circle", "turn"} {
		t.Run(shape, func(t *testing.T) {
			gen := DefaultGenConfig()
			gen.Shape, gen.Size, gen.Speed = shape, 0.6, 0.4
			if shape == "turn" {
				gen.Size, gen.Speed, gen.Accel = math.Pi/2, 1.0, 3.0
			}
			rel, _ := Generate(gen)
			r := newSim(localization.Pose2{X: 0.3, Y: 0.2, Theta: 1.0})
			frozen := r.pose
			wrong := func() (localization.Pose2, localization.Stamp, bool) { return frozen, r.now - sec(0.004), true }
			d, err := NewDriver(rel, DefaultConfig(), wrong, r.clock)
			if err != nil {
				t.Fatal(err)
			}
			run(t, r, d, 30)
			st, reason := d.Status()
			if st != Aborted || !strings.Contains(reason, "does not follow the wheels") {
				t.Fatalf("must abort on the wheel/vision mismatch, got %s: %s", st, reason)
			}
			smp := d.Samples()
			t.Logf("aborted at t=%.2f s after moving %.0f mm / %.0f deg: %s", smp[len(smp)-1].T,
				math.Hypot(r.pose.X-frozen.X, r.pose.Y-frozen.Y)*1000, localization.AngleDiff(r.pose.Theta, frozen.Theta)*180/math.Pi, reason)
		})
	}
}

func TestWheelCheckToleratesWrongDimensions(t *testing.T) {
	// 寸法は未確定。車輪の読みが 0.7〜1.4 倍、回転が 0.4〜2.5 倍ずれていても誤って止めない。
	circle := DefaultGenConfig()
	circle.Shape, circle.Size, circle.Speed = "circle", 0.6, 0.4
	turn := DefaultGenConfig()
	turn.Shape, turn.Size, turn.Speed, turn.Accel = "turn", math.Pi/2, 1.0, 3.0
	for _, g := range []GenConfig{circle, turn} {
		for _, sc := range [][2]float64{{0.7, 0.4}, {1.4, 2.5}, {1, 1}} {
			rel, _ := Generate(g)
			r := newSim(localization.Pose2{X: 0.3, Y: 0.2, Theta: 1.0})
			r.wheelScale, r.wheelRotScale = sc[0], sc[1]
			d, err := NewDriver(rel, DefaultConfig(), r.vision, r.clock)
			if err != nil {
				t.Fatal(err)
			}
			run(t, r, d, 30)
			if st, reason := d.Status(); st != Done {
				t.Errorf("%s with wheel scale %v: ended in %s: %s", g.Shape, sc, st, reason)
			}
		}
	}
}

func TestDriverAbortsOnVisionLoss(t *testing.T) {
	rel, _ := Generate(DefaultGenConfig())
	r := newSim(localization.Pose2{})
	r.blackoutFrom, r.blackoutUntil = 0.8, 100
	d, err := NewDriver(rel, DefaultConfig(), r.vision, r.clock)
	if err != nil {
		t.Fatal(err)
	}
	run(t, r, d, 10)
	if st, reason := d.Status(); st != Aborted || !strings.Contains(reason, "vision lost") {
		t.Fatalf("expected abort on vision loss, got %s: %s", st, reason)
	}
	held := false
	for _, s := range d.Samples() {
		held = held || s.Held
	}
	if !held {
		t.Error("stale vision must first hold (send zero) before aborting")
	}
}

func TestDriverAbortsOutsideFence(t *testing.T) {
	rel, _ := Generate(DefaultGenConfig())
	r := newSim(localization.Pose2{})
	d, err := NewDriver(rel, DefaultConfig(), r.vision, r.clock)
	if err != nil {
		t.Fatal(err)
	}
	if err := d.Arm(); err != nil {
		t.Fatal(err)
	}
	r.pose.X += 1.5 // 押されて枠の外へ
	for i := 0; i < 50 && !d.Finished(); i++ {
		d.OverrideVelocity()
		r.step(0.008)
	}
	if st, reason := d.Status(); st != Aborted || !strings.Contains(reason, "fence") {
		t.Fatalf("expected abort outside the fence, got %s: %s", st, reason)
	}
}

func TestDriverStopSendsZero(t *testing.T) {
	rel, _ := Generate(DefaultGenConfig())
	r := newSim(localization.Pose2{})
	d, _ := NewDriver(rel, DefaultConfig(), r.vision, r.clock)
	if vx, vy, w, ok := d.OverrideVelocity(); !ok || vx != 0 || vy != 0 || w != 0 {
		t.Fatal("before Arm the driver must send zero")
	}
	_ = d.Arm()
	for i := 0; i < 150; i++ {
		d.OverrideVelocity()
		r.step(0.008)
	}
	d.Stop("test")
	if vx, vy, w, ok := d.OverrideVelocity(); !ok || vx != 0 || vy != 0 || w != 0 {
		t.Fatal("after Stop the driver must send zero")
	}
}

func TestValidateRejectsUnsafeTrajectories(t *testing.T) {
	cfg := DefaultConfig()
	fast := DefaultGenConfig()
	fast.Speed = 1.0
	k, _ := Generate(fast)
	if err := cfg.Validate(k); err == nil {
		t.Error("a trajectory faster than MaxSpeed must be rejected")
	}
	big := DefaultGenConfig()
	big.Size = 1.5
	k, _ = Generate(big)
	if err := cfg.Validate(k); err == nil {
		t.Error("a trajectory leaving the fence must be rejected")
	}
	k, _ = Generate(DefaultGenConfig())
	for i := range k {
		k[i].Pose.X += 0.3
	}
	if err := cfg.Validate(k); err == nil {
		t.Error("a trajectory not starting at the origin must be rejected")
	}
}

// 横で回す推定器に観測が渡り、記録に残ること (制御には使わない)。
func TestDriverRecordsTheEstimator(t *testing.T) {
	gen := DefaultGenConfig()
	gen.Size, gen.Speed = 0.3, 0.2
	rel, _ := Generate(gen)
	r := newSim(localization.Pose2{X: 0.2, Y: -0.1, Theta: 0.5})
	d, err := NewDriver(rel, DefaultConfig(), r.vision, r.clock)
	if err != nil {
		t.Fatal(err)
	}
	cfg := localization.DefaultConfig()
	cfg.Geometry = localization.DefaultGeometry()
	e, err := localization.NewEstimator(cfg, localization.EstimatorOptions{})
	if err != nil {
		t.Fatal(err)
	}
	d.SetEstimator(e)
	// 机上の機体はジャイロを出さないので、車輪と vision だけで回る
	run(t, r, d, 30)
	if st, reason := d.Status(); st != Done {
		t.Fatalf("ended in %s: %s", st, reason)
	}
	smp := d.Samples()
	last := smp[len(smp)-1]
	if !last.EstValid {
		t.Fatal("the estimate must be recorded")
	}
	if last.Est.Health == localization.HealthInvalid {
		t.Errorf("estimator health %v", last.Est.Health)
	}
	// 推定は真の位置 (机上の機体) の近くに居ること
	if d := math.Hypot(last.Est.Pose.X-r.pose.X, last.Est.Pose.Y-r.pose.Y); d > 0.05 {
		t.Errorf("estimate is %.0f mm away from the simulated robot", d*1000)
	}
	if st := e.Stats(); st.WheelUpdates == 0 || st.VisionUpdates == 0 {
		t.Errorf("estimator did not get both observations: %+v", st)
	}
}

// 推定器には「その周期に読んだ車輪」が渡らなければならない。
//
// **これは実機で踏んだ不具合の番人である。** feedEstimator が s.Wheels を
// 埋める前に呼ばれていたため、推定器は毎周期「4 輪とも 0」を受け取り、
// 「止まっている」と信じ込んでいた。ジャイロの信号は行き場を失って
// バイアスに吸い込まれ、角速度が 0 に張り付く。その場回転で推定の向きが
// vision から 22 度ずれ、その推定で制御すると 76 度まで発散した
// (2026-09-24、docs/traj-poc-log.md)。
//
// **記録した CSV には正しい車輪が残るので、リプレイでは再現しない。**
// だから指標を見る試験では捕まらない。ここで直接押さえる。
func TestEstimatorGetsThisCyclesWheels(t *testing.T) {
	gen := DefaultGenConfig()
	gen.Shape, gen.Size, gen.Speed, gen.Accel = "turn", 1.5708, 1.0, 3.0
	rel, err := Generate(gen)
	if err != nil {
		t.Fatal(err)
	}

	cfg := DefaultConfig()
	cfg.Method = MethodFFPVelLead
	r := newSim(localization.Pose2{X: 0.5, Y: 0.2, Theta: 0.1})
	d, err := NewDriver(rel, cfg, r.vision, r.clock)
	if err != nil {
		t.Fatal(err)
	}
	est, err := localization.NewEstimator(localization.DefaultConfig(), localization.EstimatorOptions{})
	if err != nil {
		t.Fatal(err)
	}
	d.SetEstimator(est)
	// ジャイロは真の角速度をそのまま返す (実機の IMU の代わり)。
	d.SetImuSource(func() (float64, float64, float64, bool) { return r.omega, 0, 0, true })
	run(t, r, d, 60)

	// 回っている間、推定の角速度がジャイロに追従しているか。
	// 車輪が 0 のまま渡ると、ここが 0 に張り付く。
	var ratio []float64
	var headErr []float64
	for _, s := range d.Samples() {
		if !s.EstValid || math.Abs(s.ImuYawRate) < 0.5 {
			continue
		}
		ratio = append(ratio, math.Abs(s.Est.YawRate)/math.Abs(s.ImuYawRate))
		headErr = append(headErr, math.Abs(localization.AngleDiff(s.Est.Pose.Theta, s.Pose.Theta)))
	}
	if len(ratio) < 30 {
		t.Fatalf("回転中の標本が %d 個しかない。軌道か試験の組み立てがおかしい", len(ratio))
	}
	var rs, hs float64
	for i := range ratio {
		rs += ratio[i]
		hs += headErr[i]
	}
	meanRatio, meanHead := rs/float64(len(ratio)), hs/float64(len(headErr))*180/math.Pi
	t.Logf("回転中 %d 標本: est_omega/gyro %.2f, 推定の向きのずれ %.2f deg", len(ratio), meanRatio, meanHead)

	// 実機では 0.02 (壊れている) と 1.00 (直っている) がはっきり分かれた。
	if meanRatio < 0.7 {
		t.Errorf("推定の角速度がジャイロに追従していない (比 %.2f)。"+
			"feedEstimator が車輪を読む前に呼ばれていないか確かめること", meanRatio)
	}
	if meanHead > 5 {
		t.Errorf("推定の向きが vision から %.1f deg ずれている", meanHead)
	}
}
