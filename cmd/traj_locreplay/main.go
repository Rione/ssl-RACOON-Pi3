// traj_locreplay は軌道追従 PoC の CSV (車輪の回転速度つき) を自己位置推定器へ流し直す。
// ロボットを動かさずに、機体パラメータ・vision の遅れの補正・雑音の設定を何度でも試せる。
//
//	traj_locreplay -ident -out geometry.json a.csv b.csv   機体パラメータを同定する (vision の速度を基準)
//	traj_locreplay -geometry geometry.json -drop 1.0:1.3 a.csv
//	                                                        推定器に流し、vision を抜いた区間の持ちを測る
//
// CSV は racoon-pi3 -trajpoc が書くもの (3d3b5ed 以降。wheel_*_rad_s の列が要る)。
// 詳しくは docs/traj-poc-log.md。
package main

import (
	"encoding/csv"
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"math"
	"os"
	"sort"
	"strconv"
	"strings"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi3/internal/trajpoc"
)

// wheelLag は CSV の行の車輪の値の古さ。その周期の指令を作る時点で読めるのは
// 1 つ前の SPI 転送 (8 ms 前) で届いた値。
const wheelLag = 0.008

// base は CSV の時刻 (軌道の開始が 0、負もある) を Stamp に直すときの下駄。
const base = 100 * time.Second

type row struct {
	t, tv  float64
	pose   localization.Pose2 // その行で見えていた vision の姿勢 (撮影は tv)
	wheels [4]float64         // FL, BL, BR, FR [rad/s]
}

type frame struct {
	tv   float64
	pose localization.Pose2
}

func main() {
	ident := flag.Bool("ident", false, "機体パラメータを同定する (推定器は回さない)")
	checkOnly := flag.Bool("check", false, "PoC の車輪と vision の食い違いの検査だけを流す (誤って止めないか・止めるべきで止めるか)")
	out := flag.String("out", "", "-ident: 同定結果の書き出し先 JSON")
	geomPath := flag.String("geometry", "", "機体パラメータの JSON。空なら既定値 (未確定の値)")
	delayMs := flag.Float64("delaycomp", 0, "vision の時刻から引く一定の遅れ [ms] (timesync が分離できない片道遅延)")
	drop := flag.String("drop", "", "vision を推定器に渡さない区間 [s] (軌道の時刻)。例 1.0:1.3,2.5:2.8")
	noSlip := flag.Bool("noslip", false, "スリップの状態を使わない")
	estCSV := flag.String("csv", "", "推定の時系列の書き出し先 CSV")
	wheelNoise := flag.Float64("wheelnoise", 0, "車輪の回転速度の観測雑音 [rad/s]。0 なら既定 (0.01、未計測)")
	flag.Parse()
	if flag.NArg() == 0 {
		fmt.Fprintln(os.Stderr, "usage: traj_locreplay [-ident | -geometry g.json -drop a:b] trajpoc-*.csv ...")
		os.Exit(2)
	}

	if *ident {
		runIdent(flag.Args(), *out)
		return
	}
	if *checkOnly {
		for _, p := range flag.Args() {
			rows, err := readCSV(p)
			check(err)
			ss := make([]trajpoc.CheckSample, len(rows))
			for i, r := range rows {
				ss[i] = trajpoc.CheckSample{T: r.t, TV: r.tv, Pose: r.pose, Wheels: r.wheels}
			}
			if t, reason, stop := trajpoc.ReplayWheelCheck(ss); stop {
				fmt.Printf("%s: STOP at t=%.2f s: %s\n", p, t, reason)
			} else {
				fmt.Printf("%s: ok (never stopped)\n", p)
			}
		}
		return
	}

	cfg := localization.DefaultConfig()
	if *geomPath != "" {
		g, err := localization.LoadGeometryFile(*geomPath)
		check(err)
		cfg.Geometry = g
	}
	cfg.Noise.EnableSlip = !*noSlip
	if *wheelNoise > 0 {
		cfg.Noise.WheelNoise = *wheelNoise
	}
	drops, err := parseDrops(*drop)
	check(err)
	for _, p := range flag.Args() {
		rows, err := readCSV(p)
		check(err)
		replay(p, rows, cfg, time.Duration(*delayMs*float64(time.Millisecond)), drops, *estCSV)
	}
}

// ---- 同定 ----

func runIdent(paths []string, out string) {
	var samples []localization.IdentSample
	for _, p := range paths {
		rows, err := readCSV(p)
		check(err)
		s := identSamples(rows)
		fmt.Printf("%s: %d samples\n", p, len(s))
		samples = append(samples, s...)
	}
	res, err := localization.Identify(samples, localization.RefVision, localization.IdentOptions{})
	check(err)
	fmt.Printf("samples %d, excitation vx %.3f m/s  vy %.3f m/s  omega %.3f rad/s, condition %.1f\n",
		res.Samples, res.ExcitationVX, res.ExcitationVY, res.ExcitationOmega, res.ConditionNumber)
	for _, s := range res.Slots {
		fmt.Println("  " + s.Summary())
	}
	for _, w := range res.Warnings {
		fmt.Println("  WARNING: " + w)
	}
	g, _ := json.MarshalIndent(res.Geometry, "", "  ")
	fmt.Printf("geometry:\n%s\n", g)
	if out != "" {
		check(os.WriteFile(out, append(g, '\n'), 0o644))
		fmt.Printf("wrote %s\n", out)
	}
}

// identSamples は vision の撮影ごとに、機体座標の速度 (±20 ms の中心差分) と
// その時刻の 4 輪の回転速度を組にする。
func identSamples(rows []row) []localization.IdentSample {
	fr := frames(rows)
	wt := make([]float64, len(rows))
	for i, r := range rows {
		wt[i] = r.t - wheelLag
	}
	var out []localization.IdentSample
	for i := range fr {
		vx, vy, w, ok := bodyVelocity(fr, i, 0.02)
		if !ok || fr[i].tv < wt[0] || fr[i].tv > wt[len(wt)-1] {
			continue
		}
		var ws [4]float64
		for k := 0; k < 4; k++ {
			ws[k] = interp(wt, fr[i].tv, func(j int) float64 { return rows[j].wheels[k] })
		}
		out = append(out, localization.IdentSample{VX: vx, VY: vy, Omega: w, WheelSlots: ws})
	}
	return out
}

func bodyVelocity(fr []frame, i int, half float64) (vx, vy, w float64, ok bool) {
	a := sort.Search(len(fr), func(j int) bool { return fr[j].tv >= fr[i].tv-half })
	b := sort.Search(len(fr), func(j int) bool { return fr[j].tv > fr[i].tv+half }) - 1
	if !(a < i && i < b) {
		return 0, 0, 0, false
	}
	dt := fr[b].tv - fr[a].tv
	wx, wy := (fr[b].pose.X-fr[a].pose.X)/dt, (fr[b].pose.Y-fr[a].pose.Y)/dt
	s, c := math.Sincos(fr[i].pose.Theta)
	return c*wx + s*wy, -s*wx + c*wy, localization.AngleDiff(fr[b].pose.Theta, fr[a].pose.Theta) / dt, true
}

// ---- 推定器へ流す ----

type estPoint struct {
	t   float64 // 軌道の時刻 [s]
	est localization.Estimate
}

func replay(path string, rows []row, cfg localization.Config, delay time.Duration, drops [][2]float64, estCSV string) {
	e, err := localization.NewEstimator(cfg, localization.EstimatorOptions{VisionDelayComp: delay})
	check(err)
	dropped := func(tv float64) bool {
		for _, d := range drops {
			if tv >= d[0] && tv < d[1] {
				return true
			}
		}
		return false
	}
	var ests []estPoint
	lastTV := math.Inf(-1)
	for _, r := range rows {
		// 行の車輪の値は 1 つ前の SPI 転送で届いたもので、その行で見えた vision より先に手元にある。
		// 逆に入れると、推定器は vision の時刻まで進んだ後の「時刻が戻った車輪」として捨てる。
		e.AddWheel(localization.WheelSample{Stamp: stamp(r.t - wheelLag), Omega: r.wheels})
		if r.tv != lastTV && r.tv > lastTV {
			lastTV = r.tv
			if !dropped(r.tv) {
				e.AddVision(localization.VisionPose{Stamp: stamp(r.tv), Pose: r.pose, Confidence: 1})
			}
		}
		ests = append(ests, estPoint{t: r.t - wheelLag, est: e.Current()})
	}

	// 推定をその撮影時刻まで補間して vision と比べる。推定は撮影時刻の時点で持っている情報
	// (その撮影はまだ届いていない) だけで出したものなので、「一歩先の予測」の誤差になる。
	fr := frames(rows)
	et := make([]float64, len(ests))
	for i, p := range ests {
		et[i] = p.t
	}
	var in2, inH2 float64
	var inN int
	fmt.Printf("== %s (delaycomp %.0f ms, slip %v, wheel noise %.3f rad/s)\n", path, delay.Seconds()*1000, cfg.Noise.EnableSlip, cfg.Noise.WheelNoise)
	for _, f := range fr {
		if f.tv < et[0]+0.5 || f.tv > et[len(et)-1] || dropped(f.tv) {
			continue
		}
		ep := estAt(ests, et, f.tv)
		d := math.Hypot(ep.X-f.pose.X, ep.Y-f.pose.Y) * 1000
		h := localization.AngleDiff(ep.Theta, f.pose.Theta) * 180 / math.Pi
		in2 += d * d
		inH2 += h * h
		inN++
	}
	if inN > 0 {
		fmt.Printf("  vision があるとき: 推定 (その撮影が届く前) と vision の差 RMS %.2f mm  %.2f deg (%d 枚)\n",
			math.Sqrt(in2/float64(inN)), math.Sqrt(inH2/float64(inN)), inN)
	}

	// vision を抜いた区間: 区間の最後の撮影で、推定 / 最後の vision で止めておく / 等速で外挿 を比べる
	for _, d := range drops {
		var last, prev *frame
		var inside []frame
		for i := range fr {
			switch {
			case fr[i].tv < d[0]:
				last = &fr[i]
			case fr[i].tv < d[1]:
				inside = append(inside, fr[i])
			}
		}
		// 等速の外挿の速度は、抜く直前 40 ms の最初と最後の撮影から出す (隣り合う 2 枚だと雑音で不利すぎる)
		for i := range fr {
			if last != nil && fr[i].tv >= last.tv-0.04 && fr[i].tv < last.tv {
				prev = &fr[i]
				break
			}
		}
		if last == nil || prev == nil || len(inside) == 0 {
			fmt.Printf("  drop %.2f〜%.2f s: 区間に vision が無い\n", d[0], d[1])
			continue
		}
		var eMax, hMax, cMax float64
		var eEnd, hEnd, cEnd float64
		vx, vy := (last.pose.X-prev.pose.X)/(last.tv-prev.tv), (last.pose.Y-prev.pose.Y)/(last.tv-prev.tv)
		for _, f := range inside {
			ep := estAt(ests, et, f.tv)
			de := math.Hypot(ep.X-f.pose.X, ep.Y-f.pose.Y) * 1000
			dh := math.Hypot(last.pose.X-f.pose.X, last.pose.Y-f.pose.Y) * 1000
			dt := f.tv - last.tv
			dc := math.Hypot(last.pose.X+vx*dt-f.pose.X, last.pose.Y+vy*dt-f.pose.Y) * 1000
			eMax, hMax, cMax = math.Max(eMax, de), math.Max(hMax, dh), math.Max(cMax, dc)
			eEnd, hEnd, cEnd = de, dh, dc
		}
		ee := estAt(ests, et, inside[len(inside)-1].tv)
		fmt.Printf("  drop %.2f〜%.2f s (%d 枚): 最大/最後の誤差 [mm]  推定 %.1f/%.1f  最後の vision のまま %.1f/%.1f  等速で外挿 %.1f/%.1f  (向き %.2f deg)\n",
			d[0], d[1], len(inside), eMax, eEnd, hMax, hEnd, cMax, cEnd,
			localization.AngleDiff(ee.Theta, inside[len(inside)-1].pose.Theta)*180/math.Pi)
	}
	st := e.Stats()
	fmt.Printf("  stats: wheel %d, vision %d, too old %d, future %d, resets %d, huber %d\n",
		st.WheelUpdates, st.VisionUpdates, st.VisionTooOld, st.VisionFuture, st.Resets, st.HuberDownweights)

	if estCSV != "" {
		f, err := os.Create(estCSV)
		check(err)
		defer f.Close()
		fmt.Fprintln(f, "t_s,est_x_mm,est_y_mm,est_theta_rad,est_vx_mm_s,est_vy_mm_s,est_omega_rad_s,slip_x_mm_s,slip_y_mm_s,health,since_vision_ms")
		for _, p := range ests {
			s := p.est
			fmt.Fprintf(f, "%.4f,%.2f,%.2f,%.5f,%.1f,%.1f,%.4f,%.1f,%.1f,%s,%.0f\n", p.t,
				s.Pose.X*1000, s.Pose.Y*1000, s.Pose.Theta, s.VelBody.X*1000, s.VelBody.Y*1000, s.YawRate,
				s.Slip.X*1000, s.Slip.Y*1000, s.Health, s.SinceVision.Seconds()*1000)
		}
		fmt.Printf("  wrote %s\n", estCSV)
	}
}

// estAt は推定の時系列を時刻 t に補間する (直前の推定から、その速度で進める)。
func estAt(ests []estPoint, et []float64, t float64) localization.Pose2 {
	i := sort.SearchFloat64s(et, t) - 1
	if i < 0 {
		i = 0
	}
	s := ests[i].est
	dt := t - ests[i].t
	v := localization.Rotate(s.Pose.Theta, s.VelBody)
	return localization.Pose2{X: s.Pose.X + v.X*dt, Y: s.Pose.Y + v.Y*dt, Theta: s.Pose.Theta + s.YawRate*dt}
}

// ---- CSV ----

func readCSV(path string) ([]row, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	r := csv.NewReader(f)
	head, err := r.Read()
	if err != nil {
		return nil, err
	}
	col := map[string]int{}
	for i, h := range head {
		col[h] = i
	}
	need := []string{"t_s", "tv_s", "pose_x_mm", "pose_y_mm", "pose_theta_rad",
		"wheel_fl_rad_s", "wheel_bl_rad_s", "wheel_br_rad_s", "wheel_fr_rad_s"}
	for _, n := range need {
		if _, ok := col[n]; !ok {
			return nil, fmt.Errorf("%s: column %q is missing (車輪を記録する前の CSV?)", path, n)
		}
	}
	var rows []row
	for {
		rec, err := r.Read()
		if err == io.EOF {
			break
		}
		if err != nil {
			return nil, err
		}
		v := func(n string) float64 {
			x, _ := strconv.ParseFloat(rec[col[n]], 64)
			return x
		}
		rows = append(rows, row{
			t: v("t_s"), tv: v("tv_s"),
			pose:   localization.Pose2{X: v("pose_x_mm") / 1000, Y: v("pose_y_mm") / 1000, Theta: v("pose_theta_rad")},
			wheels: [4]float64{v("wheel_fl_rad_s"), v("wheel_bl_rad_s"), v("wheel_br_rad_s"), v("wheel_fr_rad_s")},
		})
	}
	if len(rows) < 10 {
		return nil, fmt.Errorf("%s: too few rows", path)
	}
	return rows, nil
}

// frames は vision の撮影ごとに 1 つ (撮影時刻の順)。
func frames(rows []row) []frame {
	seen := map[float64]bool{}
	var fr []frame
	for _, r := range rows {
		if !seen[r.tv] {
			seen[r.tv] = true
			fr = append(fr, frame{tv: r.tv, pose: r.pose})
		}
	}
	sort.Slice(fr, func(i, j int) bool { return fr[i].tv < fr[j].tv })
	return fr
}

func interp(xs []float64, x float64, y func(int) float64) float64 {
	i := sort.SearchFloat64s(xs, x)
	switch {
	case i <= 0:
		return y(0)
	case i >= len(xs):
		return y(len(xs) - 1)
	}
	u := (x - xs[i-1]) / (xs[i] - xs[i-1])
	return y(i-1)*(1-u) + y(i)*u
}

func parseDrops(s string) ([][2]float64, error) {
	var out [][2]float64
	if s == "" {
		return out, nil
	}
	for _, part := range strings.Split(s, ",") {
		ab := strings.Split(part, ":")
		if len(ab) != 2 {
			return nil, fmt.Errorf("bad -drop %q (from:to)", part)
		}
		a, err1 := strconv.ParseFloat(ab[0], 64)
		b, err2 := strconv.ParseFloat(ab[1], 64)
		if err1 != nil || err2 != nil || b <= a {
			return nil, fmt.Errorf("bad -drop %q", part)
		}
		out = append(out, [2]float64{a, b})
	}
	return out, nil
}

func stamp(t float64) localization.Stamp {
	return localization.Stamp(base + time.Duration(t*float64(time.Second)))
}

func check(err error) {
	if err != nil {
		fmt.Fprintln(os.Stderr, "traj_locreplay:", err)
		os.Exit(1)
	}
}
