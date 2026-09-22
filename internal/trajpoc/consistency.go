package trajpoc

import (
	"fmt"
	"math"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// PoCGeometry はテスト機の車輪の並び・符号。寸法は既定 (未確定) のままで、
// 符号だけ実機の記録から決まった -1 にしてある (traj-poc-log §5-17)。
func PoCGeometry() localization.GeometryConfig {
	g := localization.DefaultGeometry()
	g.WheelSigns = [localization.NumWheels]float64{-1, -1, -1, -1}
	return g
}

// 車輪と vision の食い違いの検査 (traj-poc-log §5-18)。
//
// vision の位置は枠の判定にも使うので、vision が別の物 (止まっている他のロボットの模様など) を
// 見ていると、ロボットは見張られないまま走る。実際に 4 本それで走った。
// そこで「車輪は動いたと言っているのに、vision がその向きにほとんど動いていない」が
// Window 続いたら打ち切る。寸法が未確定なので、倍率が多少違っても誤って止めない緩い判定にしてある。
const (
	checkWindow   = 0.5  // 見比べる区間 [s]
	checkMinTrans = 0.08 // 車輪がこれ以上動いたときだけ判定 [m]
	checkMinRot   = 0.3  // 車輪がこれ以上回ったときだけ判定 [rad]
	checkRatio    = 0.25 // vision の動きが車輪のこの割合に満たなければ食い違い
	checkPersist  = 0.1  // 食い違いがこれだけ続いたら止める [s]
)

type wheelEntry struct {
	t      float64           // 軌道の時刻 [s]
	wheel  localization.Vec2 // 車輪から積分した world の移動 (累積) [m]
	wheelR float64           // 車輪から積分した回転 (累積) [rad]
}

type visionEntry struct {
	tv   float64            // 撮影時刻 (軌道の時刻) [s]
	pose localization.Pose2 // Theta は巻き戻さない累積
}

type wheelVisionCheck struct {
	kin    *localization.Kinematics
	wheels []wheelEntry
	vision []visionEntry
	cur    wheelEntry
	visR   float64 // vision の向きの累積
	lastTh float64
	lastTV float64
	init   bool
	since  float64 // 食い違いが続いている時間 [s]
}

func newWheelVisionCheck(g localization.GeometryConfig) (*wheelVisionCheck, error) {
	k, err := localization.NewKinematics(g)
	if err != nil {
		return nil, err
	}
	return &wheelVisionCheck{kin: k}, nil
}

// wheelAt は時刻 t の車輪の累積 (直前の記録)。
func (c *wheelVisionCheck) wheelAt(t float64) wheelEntry {
	e := c.wheels[0]
	for _, w := range c.wheels {
		if w.t > t {
			break
		}
		e = w
	}
	return e
}

// step は 1 周期ぶん進め、食い違っていれば理由を返す。wheels は SPI の並び [rad/s]。
// vision は撮影時刻 tv の姿勢。車輪も同じ撮影時刻どうしで比べる (vision は遅れて届くので、
// 今の車輪と比べると折り返しなどで食い違って見える)。
func (c *wheelVisionCheck) step(t, dt float64, wheels [4]float64, vision localization.Pose2, tv float64) (string, bool) {
	vx, vy, w := c.kin.BodyFromWheel(c.kin.SlotsToLogical(wheels))
	v := localization.Rotate(vision.Theta, localization.Vec2{X: vx, Y: vy})
	c.cur.t = t
	c.cur.wheel.X += v.X * dt
	c.cur.wheel.Y += v.Y * dt
	c.cur.wheelR += w * dt
	c.wheels = append(c.wheels, c.cur)
	if !c.init || tv > c.lastTV {
		if !c.init {
			c.lastTh, c.init = vision.Theta, true
		}
		c.visR += localization.AngleDiff(vision.Theta, c.lastTh)
		c.lastTh, c.lastTV = vision.Theta, tv
		c.vision = append(c.vision, visionEntry{tv: tv, pose: localization.Pose2{X: vision.X, Y: vision.Y, Theta: c.visR}})
	}
	for len(c.wheels) > 1 && c.wheels[1].t < t-checkWindow-0.5 {
		c.wheels = c.wheels[1:]
	}
	for len(c.vision) > 1 && c.vision[1].tv <= tv-checkWindow {
		c.vision = c.vision[1:]
	}
	oldV, curV := c.vision[0], c.vision[len(c.vision)-1]
	if curV.tv-oldV.tv < checkWindow*0.9 || oldV.tv < c.wheels[0].t {
		c.since = 0
		return "", false
	}
	oldW, curW := c.wheelAt(oldV.tv), c.wheelAt(curV.tv)

	reason := ""
	dw := localization.Vec2{X: curW.wheel.X - oldW.wheel.X, Y: curW.wheel.Y - oldW.wheel.Y}
	if n := math.Hypot(dw.X, dw.Y); n > checkMinTrans {
		// vision の移動を車輪の移動の向きへ射影する
		along := ((curV.pose.X-oldV.pose.X)*dw.X + (curV.pose.Y-oldV.pose.Y)*dw.Y) / n
		if along < checkRatio*n {
			reason = fmt.Sprintf("vision does not follow the wheels: wheels moved %.0f mm in %.1f s, vision %.0f mm that way (wrong -trajvisionid?)",
				n*1000, checkWindow, along*1000)
		}
	}
	if dr := curW.wheelR - oldW.wheelR; math.Abs(dr) > checkMinRot {
		if dv := curV.pose.Theta - oldV.pose.Theta; dv*math.Copysign(1, dr) < checkRatio*math.Abs(dr) {
			reason = fmt.Sprintf("vision does not follow the wheels: wheels turned %.0f deg in %.1f s, vision %.0f deg (wrong -trajvisionid?)",
				dr*180/math.Pi, checkWindow, dv*180/math.Pi)
		}
	}
	if reason == "" {
		c.since = 0
		return "", false
	}
	// 一瞬の食い違い (折り返しの瞬間など) では止めない
	c.since += dt
	return reason, c.since >= checkPersist
}

// CheckSample はオフラインで検査を流すときの 1 周期 (traj_locreplay -check 用)。
type CheckSample struct {
	T, TV  float64
	Pose   localization.Pose2
	Wheels [4]float64
}

// ReplayWheelCheck は記録に検査を流し、最初に止めた時刻と理由を返す (止めなければ ok=false)。
func ReplayWheelCheck(samples []CheckSample) (t float64, reason string, ok bool) {
	c, err := newWheelVisionCheck(PoCGeometry())
	if err != nil {
		return 0, err.Error(), true
	}
	for i, s := range samples {
		dt := 0.008
		if i > 0 {
			dt = math.Min(s.T-samples[i-1].T, 0.05)
		}
		if r, bad := c.step(s.T, dt, s.Wheels, s.Pose, s.TV); bad {
			return s.T, r, true
		}
	}
	return 0, "", false
}
