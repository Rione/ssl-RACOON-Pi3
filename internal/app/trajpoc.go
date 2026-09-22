//go:build pi4 || rock5a

package app

import (
	"bytes"
	"encoding/binary"
	"flag"
	"fmt"
	"io"
	"log"
	"os"
	"os/signal"
	"path/filepath"
	"syscall"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/api"
	"github.com/Rione/ssl-RACOON-Pi3/internal/link"
	"github.com/Rione/ssl-RACOON-Pi3/internal/locadapter"
	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi3/internal/loclog"
	"github.com/Rione/ssl-RACOON-Pi3/internal/state"
	"github.com/Rione/ssl-RACOON-Pi3/internal/timesync"
	"github.com/Rione/ssl-RACOON-Pi3/internal/trajpoc"
)

// 時刻つき軌道追従の PoC (docs/traj-poc.md)。-trajpoc を付けたときだけ動く。
// 標準入力から軌道を読み、SSL-Vision の自機の姿勢で閉ループを回して、
// SPI の速度指令を差し替える。ロボットが自走する。

var trajPoc struct {
	enabled  bool
	method   string
	interp   string
	kp, kth  float64
	leadMs   float64
	maxSpeed float64
	fence    float64
	visionID int
	csvDir   string
	nearGoal bool
}

// PoC で許す上限。フラグの打ち間違いで暴走させないための天井。
const (
	trajPocSpeedCeiling = 2.0 // [m/s]
	trajPocFenceCeiling = 3.0 // [m]
)

func registerTrajPocFlags() {
	d := trajpoc.DefaultConfig()
	flag.BoolVar(&trajPoc.enabled, "trajpoc", false, "標準入力の時刻つき軌道を追従する PoC を実行する。ロボットが自走するので注意")
	flag.StringVar(&trajPoc.method, "trajmethod", string(d.Method), "追従の手法 (p | ffp | ffp_lead | ffp_vlead)")
	flag.StringVar(&trajPoc.interp, "trajinterp", string(d.Interp), "点の間の補間 (linear | hermite)")
	flag.Float64Var(&trajPoc.kp, "trajkp", d.Kp, "位置の P ゲイン [1/s]")
	flag.Float64Var(&trajPoc.kth, "trajkth", d.Kth, "向きの P ゲイン [1/s]")
	flag.Float64Var(&trajPoc.leadMs, "trajlead", d.Lead*1000, "ffp_lead / ffp_vlead で先を狙う時間 [ms]")
	flag.Float64Var(&trajPoc.maxSpeed, "trajmaxspeed", d.MaxSpeed, "並進速度の上限 [m/s]")
	flag.Float64Var(&trajPoc.fence, "trajfence", d.Fence, "軌道が収まるべき開始位置からの半径 [m]")
	flag.IntVar(&trajPoc.visionID, "trajvisionid", -1, "SSL-Vision 上の自機 ID (カバーの模様)。-1 なら DIP スイッチの ID")
	flag.StringVar(&trajPoc.csvDir, "trajcsv", ".", "記録 CSV の出力先ディレクトリ。空なら書かない")
	flag.BoolVar(&trajPoc.nearGoal, "trajneargoal", false, "軌道が終わった後の寄せ方を √ブレーキ則 + 不感帯にする。判断はスミス予測の位置 (vision + まだ効いていない指令)、最低速度 30 mm/s")
}

func trajPocConfig() (trajpoc.Config, error) {
	cfg := trajpoc.DefaultConfig()
	var err error
	if cfg.Method, err = trajpoc.ParseMethod(trajPoc.method); err != nil {
		return cfg, err
	}
	if cfg.Interp, err = trajpoc.ParseInterp(trajPoc.interp); err != nil {
		return cfg, err
	}
	if trajPoc.maxSpeed > trajPocSpeedCeiling || trajPoc.fence > trajPocFenceCeiling {
		return cfg, fmt.Errorf("-trajmaxspeed <= %.1f m/s and -trajfence <= %.1f m", trajPocSpeedCeiling, trajPocFenceCeiling)
	}
	cfg.Kp, cfg.Kth, cfg.Lead = trajPoc.kp, trajPoc.kth, trajPoc.leadMs/1000
	cfg.MaxSpeed, cfg.Fence = trajPoc.maxSpeed, trajPoc.fence
	cfg.NearGoal.Enabled = trajPoc.nearGoal
	return cfg, nil
}

// runTrajPoC は PoC の本体。最後は必ず停止処理を通ってからプロセスを終える。
func runTrajPoC(done <-chan struct{}, myID uint32) {
	// 標準出力の相手 (SSH) が消えたとき、Go は既定で SIGPIPE により即死する。
	// STM は指令が途絶えても最後の速度で走り続けるので、即死は許されない。
	signal.Ignore(syscall.SIGPIPE)
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM, syscall.SIGHUP)

	fail := func(format string, args ...any) {
		log.Printf("[TRAJ] "+format, args...)
		finishTrajPoC(nil, trajpoc.Config{}, 1)
	}
	if state.LocIdent {
		fail("-trajpoc cannot be combined with -locident")
	}
	if !state.IsNewRobot {
		fail("-trajpoc is only for Rock5A (the Pi 4B frame layout is not supported)")
	}
	cfg, err := trajPocConfig()
	if err != nil {
		fail("bad flags: %v", err)
	}

	// 1. 軌道を読む (END の行まで)。
	log.Printf("[TRAJ] reading the trajectory from stdin (JSON lines, then %q) ...", trajpoc.EndMarker)
	stdin := os.Stdin
	rel, err := trajpoc.ReadTrajectory(stdin)
	if err != nil {
		fail("trajectory: %v", err)
	}
	b := trajpoc.MeasureBounds(rel)
	log.Printf("[TRAJ] %d points, %.2f s, max radius %.2f m, max speed %.2f m/s, max yaw rate %.2f rad/s",
		len(rel), b.Duration, b.MaxRadius, b.MaxSpeed, b.MaxYawRate)

	// 2. SSL-Vision を直接受ける。
	team, err := locadapter.ParseTeam(state.LocTeam)
	if err != nil {
		fail("team: %v", err)
	}
	visionID := uint32(myID)
	if trajPoc.visionID >= 0 {
		visionID = uint32(trajPoc.visionID)
	}
	clock := loclog.NewClock()
	receiver := locadapter.NewVisionReceiver(clock, timesync.NewConvexHullProvider(timesync.Config{}), nil,
		locadapter.VisionConfig{Address: state.LocVisionAddr, Interface: state.LocVisionIface, RobotID: visionID, Team: team})
	go func() {
		if err := receiver.Run(done); err != nil {
			log.Printf("[TRAJ] vision receiver stopped: %v", err)
		}
	}()
	vision := func() (localization.Pose2, localization.Stamp, bool) {
		vp, ok := receiver.Latest()
		return vp.Pose, vp.Stamp, ok
	}
	driver, err := trajpoc.NewDriver(rel, cfg, vision, clock.Now)
	if err != nil {
		fail("trajectory rejected: %v", err)
	}

	log.Printf("[TRAJ] waiting for vision of %s %d ...", team, visionID)
	deadline := time.Now().Add(5 * time.Second)
	for {
		if _, capture, ok := vision(); ok && (clock.Now()-capture).Seconds() < cfg.VisionHoldAge {
			break
		}
		if time.Now().After(deadline) {
			fail("no fresh vision of %s %d within 5 s (check -team / -trajvisionid / -visionaddr / -visioniface)", team, visionID)
		}
		time.Sleep(20 * time.Millisecond)
	}

	// 3. 走らせる。速度の差し替え (この時点では 0) を先に登録してから非常停止を解く。
	//    逆順だと、解いた瞬間に差し替え前の速度が流れる隙間ができる。
	link.SetVelocityOverride(driver)
	state.TrajPoCActive.Store(true)
	state.SetSendPayload(encodeArmedZeroPayload())
	if err := driver.Arm(); err != nil {
		finishTrajPoC(driver, cfg, 1)
	}
	s := driver.Start()
	log.Printf("[TRAJ] *** RUNNING: method=%s interp=%s from (%.0f, %.0f) mm, %.1f deg. Ctrl+C to stop ***",
		cfg.Method, cfg.Interp, s.X*1000, s.Y*1000, s.Theta*180/3.14159265)

	// SSH が切れると標準入力が EOF になる。それを止める合図にする。
	stdinClosed := make(chan struct{})
	go func() {
		_, _ = io.Copy(io.Discard, stdin)
		close(stdinClosed)
	}()

	tick := time.NewTicker(50 * time.Millisecond)
	defer tick.Stop()
	lastPrint := time.Now()
	for {
		select {
		case sg := <-sig:
			driver.Stop("signal " + sg.String())
		case <-stdinClosed:
			driver.Stop("stdin closed (SSH disconnected?)")
			stdinClosed = nil
		case <-done:
			driver.Stop("shutdown")
		case <-tick.C:
		}
		if driver.Finished() {
			finishTrajPoC(driver, cfg, 0)
		}
		if time.Since(lastPrint) >= time.Second {
			lastPrint = time.Now()
			if smp := driver.Samples(); len(smp) > 0 {
				l := smp[len(smp)-1]
				dx, dy := (l.Pose.X-l.Ref.Pos.X)*1000, (l.Pose.Y-l.Ref.Pos.Y)*1000
				log.Printf("[TRAJ] t=%5.2fs err=(%+5.0f,%+5.0f) mm cmd=(%+5.0f,%+5.0f) mm/s age=%3.0f ms",
					l.T, dx, dy, l.CmdWorld.X*1000, l.CmdWorld.Y*1000, l.Age*1000)
			}
		}
	}
}

// finishTrajPoC は止めて、結果を出して、プロセスを終える。どの終わり方でもここを通る。
func finishTrajPoC(driver *trajpoc.Driver, cfg trajpoc.Config, code int) {
	if driver != nil {
		driver.Stop("finishing")
		// 0 の指令を SPI (8 ms 周期) で十分な回数送ってから非常停止を立て直す。
		time.Sleep(150 * time.Millisecond)
	}
	state.SetSendPayload(nil) // 受信前と同じ「非常停止が立ったフレーム」に戻す
	time.Sleep(50 * time.Millisecond)
	link.SetVelocityOverride(nil)
	state.TrajPoCActive.Store(false)

	if driver != nil {
		st, reason := driver.Status()
		samples := driver.Samples()
		if ref := driver.Reference(); ref != nil {
			m := trajpoc.ComputeMetrics(samples, ref.Knots())
			fmt.Print(m.Format(cfg, st, reason))
		} else {
			fmt.Printf("=== trajpoc: did not start (%s: %s) ===\n", st, reason)
		}
		if trajPoc.csvDir != "" && len(samples) > 0 {
			path := filepath.Join(trajPoc.csvDir, fmt.Sprintf("trajpoc-%s-%s-%s.csv",
				time.Now().Format("20060102-150405"), cfg.Method, cfg.Interp))
			if f, err := os.Create(path); err != nil {
				log.Printf("[TRAJ] csv: %v", err)
			} else {
				if err := trajpoc.WriteCSV(f, samples); err != nil {
					log.Printf("[TRAJ] csv: %v", err)
				}
				f.Close()
				fmt.Printf("csv: %s\n", path)
			}
		}
	}
	api.StopPythonProcess()
	state.RunLocalizationShutdown()
	cleanupBoard()
	os.Exit(code)
}

// encodeArmedZeroPayload は「PC から速度 0 を受け取った」のと同じフレームを作る。
// 非常停止のビットを落とし、充電はしない (PoC はキックしない)。
func encodeArmedZeroPayload() []byte {
	var buf bytes.Buffer
	if err := binary.Write(&buf, binary.LittleEndian, state.SendPayload{}); err != nil {
		log.Fatal(err)
	}
	return buf.Bytes()
}
