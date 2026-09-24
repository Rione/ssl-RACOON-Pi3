// loc_replay は実機で録った MCAP を推定器へ流し直し、設定を振って比べる道具。
//
// handoff §6.2 が「これが無いと P1 で録ったログをフィルタの改善に使えない」
// として最優先に挙げていたもの。実機には真値が無いので、**RTS スムーザの
// 平滑化結果を疑似真値**として使う (計画 §8)。
//
// 出すもの:
//
//  1. ログの概要 (レート、欠番、電池)
//  2. **車輪だけの幾何検査** — vision も時刻合わせも使わずに取付角の比を出す
//     (docs/self-localization-research-20260923.md §2.4 / §2.5)。
//     CAD の 60 度説と同定値の 55 度説はここで分かれる。
//  3. **vision の定数遅延** — 車輪と vision の速度の相互相関から測る (§4.3)
//  4. 疑似真値に対する精度と一貫性 (NEES)
//  5. **機能ごとの効果の比較表** — 効いていないものを黙って既定にしないため
//
// 使い方:
//
//	loc_replay -log racoon-loc-*.mcap
//	loc_replay -log x.mcap -geometry config/geometry-racoon-56011.json
//	loc_replay -log x.mcap -vision-delay auto
//	loc_replay -log x.mcap -compare
package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"sort"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi3/internal/loclog"
)

func main() {
	logPath := flag.String("log", "", "実機で録った MCAP ファイル")
	robotFeed := flag.Bool("robotfeed", false, "実機と同じ順・同じ刻印で推定器に入れる (車輪/IMU を 8 ms 戻し、vision は撮影時刻、順は 車輪→IMU→vision)")
	csvGlob := flag.String("csv", "", "軌道追従 PoC の CSV (glob 可、例 trajpoc-dataset/poc/*.csv)")
	geomPath := flag.String("geometry", "", "機体パラメータの JSON (省略すると既定値)")
	visionDelay := flag.String("vision-delay", "auto", "vision の定数遅延: auto | 0 | 12ms のような値")
	compare := flag.Bool("compare", false, "機能ごとの効果を比較する")
	qsweep := flag.Bool("qsweep", false, "プロセス雑音を振って NIS が 3 になる値を探す")
	dropout := flag.Bool("dropout", false, "vision を人為的に落として推定がどこまで持つかを測る")
	calibOut := flag.String("calib-out", "", "較正した機体パラメータの書き出し先 JSON")
	wheelNoise := flag.Float64("wheel-noise", 0, "車輪雑音の定数分 [rad/s] を上書き (0 なら既定)")
	wheelCoef := flag.Float64("wheel-coef", -1, "車輪雑音の速度比例分を上書き (負なら既定)")
	accelNoise := flag.Float64("accel-noise", 0, "並進の加速度雑音密度を上書き (0 なら既定)")
	angAccelNoise := flag.Float64("angaccel-noise", 0, "角加速度の雑音密度を上書き (0 なら既定)")
	rearAngle := flag.Float64("rear-angle", 135, "車輪だけの幾何検査で仮定する後輪の取付角 [deg]")
	flag.Parse()

	if *logPath == "" && *csvGlob == "" {
		fmt.Fprintln(os.Stderr, "loc_replay: one of -log or -csv is required")
		flag.Usage()
		os.Exit(2)
	}
	calOut = *calibOut
	var err error
	if *csvGlob != "" {
		over := overrides{
			wheelNoise: *wheelNoise, wheelCoef: *wheelCoef,
			accelNoise: *accelNoise, angAccelNoise: *angAccelNoise,
		}
		err = runCSV(*csvGlob, *geomPath, *visionDelay, *rearAngle, *compare, *qsweep, *dropout, *robotFeed, over)
	} else {
		err = run(*logPath, *geomPath, *visionDelay, *rearAngle, *compare)
	}
	if err != nil {
		fmt.Fprintf(os.Stderr, "loc_replay: %v\n", err)
		os.Exit(1)
	}
}

// runCSV は PoC の CSV を読んで同じ解析をする。
//
// **車輪だけの幾何検査は全本をまとめて**行う (1 本では加振が足りないことが多い)。
// 推定のリプレイは本ごとに行う。
// overrides はコマンドラインからの雑音の上書き。**調整のための道具**で、
// 既定値そのものは config.go に根拠つきで書くこと。
type overrides struct {
	wheelNoise, wheelCoef     float64
	accelNoise, angAccelNoise float64
}

func (o overrides) apply(c *localization.Config) {
	if o.wheelNoise > 0 {
		c.Noise.WheelNoise = o.wheelNoise
	}
	if o.wheelCoef >= 0 {
		c.Noise.WheelNoiseSpeedCoef = o.wheelCoef
	}
	if o.accelNoise > 0 {
		c.Noise.AccelNoise = o.accelNoise
	}
	if o.angAccelNoise > 0 {
		c.Noise.AngAccelNoise = o.angAccelNoise
	}
}

func runCSV(pattern, geomPath, visionDelay string, rearAngle float64, compare, qsweep, dropout, robotFeed bool, over overrides) error {
	cfg, err := loadConfig(geomPath)
	if err != nil {
		return err
	}
	over.apply(&cfg)
	fmt.Printf("noise: wheel %.3f + %.3f*|w| rad/s, accel %.1f, angAccel %.1f\n",
		cfg.Noise.WheelNoise, cfg.Noise.WheelNoiseSpeedCoef, cfg.Noise.AccelNoise, cfg.Noise.AngAccelNoise)
	runs, err := loadPoCRuns(pattern)
	if err != nil {
		return err
	}

	var allWheels []sample
	var withWheels int
	var totalVision int
	for _, r := range runs {
		if r.hasWheels && len(r.wheels) > 0 {
			allWheels = append(allWheels, r.wheels...)
			withWheels++
		}
		totalVision += len(r.vision)
	}
	fmt.Printf("\n=== %s ===\n", pattern)
	fmt.Printf("runs                  %d (%d with wheel speeds)\n", len(runs), withWheels)
	fmt.Printf("wheel samples         %d\n", len(allWheels))
	fmt.Printf("vision samples        %d\n", totalVision)

	if len(allWheels) > 0 {
		logical := toLogical(allWheels, cfg.Geometry)
		if err := reportWheelOnlyGeometry(logical, cfg.Geometry, rearAngle); err != nil {
			fmt.Printf("\nwheel-only geometry check: %v\n", err)
		}
		reportHypotheses(logical, cfg.Geometry)
	}

	// 遅延と精度は 1 本ずつ。走りごとに開始姿勢も条件も違う。
	kin, err := localization.NewKinematics(cfg.Geometry)
	if err != nil {
		return err
	}
	for _, r := range runs {
		if !r.hasWheels || len(r.wheels) < 200 || len(r.vision) < 100 {
			continue
		}
		if r.battery > 0 {
			fmt.Printf("\n--- %s (電池 %.1f V) ---\n", filepath.Base(r.path), r.battery)
		} else {
			fmt.Printf("\n--- %s (電池 不明: 2026-09-24 より前の記録) ---\n", filepath.Base(r.path))
		}
		d, err := resolveVisionDelay(visionDelay, r.wheels, r.vision, kin, cfg.Geometry)
		if err != nil {
			return err
		}
		vcomp, wshift := splitDelay(d)
		ws := shiftWheels(r.wheels, wshift)
		if dropout {
			if err := reportDropout(cfg, ws, r.vision, vcomp); err != nil {
				return err
			}
			continue
		}
		if qsweep {
			if err := reportQSweep(cfg, ws, r.vision, vcomp); err != nil {
				return err
			}
			continue
		}
		if compare {
			if err := reportComparison(cfg, ws, r.vision, nil, vcomp); err != nil {
				return err
			}
			continue
		}
		if robotFeed {
			rep, err := replayRobotFeed(cfg, ws, r.vision, vcomp)
			if err != nil {
				return err
			}
			rep.print(filepath.Base(r.path) + " [実機と同じ給餌]")
			continue
		}
		rep, err := replay(cfg, ws, r.vision, vcomp, nil)
		if err != nil {
			return err
		}
		rep.print(filepath.Base(r.path))
	}
	return nil
}

// reportHypotheses は当てはめた左零ベクトルを、候補の幾何と突き合わせる。
//
// **これが CAD 説と同定値説の決着**になる (研究 §2.5)。
func reportHypotheses(logical [][localization.NumWheels]float64, cur localization.GeometryConfig) {
	fit, err := localization.FitNullVector(logical)
	if err != nil {
		return
	}
	cands := []struct {
		name string
		g    localization.GeometryConfig
	}{
		{"CAD angles, equal radii", localization.CADGeometry()},
		{"CAD angles, calibrated radii (default)", localization.DefaultGeometry()},
		{"PoC identified (all 12 parameters)", pocIdentifiedGeometry()},
		{"STM firmware (+-55/+-135)", stmGeometry()},
		{"configured", cur},
	}
	fmt.Printf("\ndistance from the fitted null vector to each hypothesis (smaller = better):\n")
	best, bestName := math.Inf(1), ""
	for _, c := range cands {
		kin, err := localization.NewKinematics(c.g)
		if err != nil {
			continue
		}
		red, err := localization.NewRedundancy(kin)
		if err != nil {
			continue
		}
		d := nullDistance(normalize(fit.N), normalize(red.NullVector()))
		fmt.Printf("  %-42s %.4f\n", c.name, d)
		if d < best {
			best, bestName = d, c.name
		}
	}
	fmt.Printf("  --> closest: %s\n", bestName)
}

// pocIdentifiedGeometry は PoC が vision 基準で 12 個すべて同定した値。
// 比較の対照として置いてある (trajpoc-dataset/geometry/geometry-racoon-56011.json)。
func pocIdentifiedGeometry() localization.GeometryConfig {
	g := localization.DefaultGeometry()
	g.WheelAnglesDeg = [localization.NumWheels]float64{55.4, 136.1, -136.3, -57.4}
	g.WheelRadiusM = [localization.NumWheels]float64{0.02933, 0.02803, 0.02818, 0.02805}
	g.MomentArmM = 0.074
	return g
}

func stmGeometry() localization.GeometryConfig {
	g := localization.DefaultGeometry()
	g.WheelAnglesDeg = [localization.NumWheels]float64{55, 135, -135, -55}
	g.MomentArmM = 0.075
	r := 0.030
	g.WheelRadiusM = [localization.NumWheels]float64{r, r, r, r}
	return g
}

func nullDistance(a, b [localization.NumWheels]float64) float64 {
	var d float64
	for i := range a {
		d += (a[i] - b[i]) * (a[i] - b[i])
	}
	return math.Sqrt(d)
}

func loadConfig(geomPath string) (localization.Config, error) {
	cfg := localization.DefaultConfig()
	if geomPath != "" {
		g, err := localization.LoadGeometryFile(geomPath)
		if err != nil {
			return cfg, err
		}
		cfg.Geometry = g
		fmt.Printf("geometry from %s\n", geomPath)
	} else {
		fmt.Println("geometry: package default (CAD angles + calibrated radii)")
	}
	return cfg, nil
}

// sample はリプレイに使う 1 周期ぶんの入力。
type sample struct {
	wheelStamp localization.Stamp
	wheels     [localization.NumWheels]float64 // スロット順
	hasWheel   bool
	// imu は同じ SPI フレームで届いた IMU。古い記録には無い。
	imu    localization.ImuSample
	hasImu bool
}

type visionSample struct {
	stamp   localization.Stamp
	arrival localization.Stamp
	pose    localization.Pose2
	camera  uint32
	frame   uint32
}

func run(logPath, geomPath, visionDelay string, rearAngle float64, compare bool) error {
	cfg, err := loadConfig(geomPath)
	if err != nil {
		return err
	}

	r, err := loclog.OpenReader(logPath)
	if err != nil {
		return err
	}
	defer r.Close()

	wheels, err := loclog.ReadJSONInto[loclog.WheelRecord](r, loclog.ChWheel.Topic())
	if err != nil {
		return err
	}
	metas, err := loclog.ReadJSONInto[loclog.VisionMetaRecord](r, loclog.ChVisionMeta.Topic())
	if err != nil {
		return err
	}
	if len(wheels) == 0 {
		return fmt.Errorf("no wheel samples in %s", logPath)
	}

	ws := make([]sample, 0, len(wheels))
	for _, w := range wheels {
		ws = append(ws, sample{
			wheelStamp: localization.Stamp(w.SampleNs),
			wheels: [localization.NumWheels]float64{
				w.WheelFLRadS, w.WheelBLRadS, w.WheelBRRadS, w.WheelFRRadS,
			},
			hasWheel: true,
		})
	}
	sort.Slice(ws, func(i, j int) bool { return ws[i].wheelStamp < ws[j].wheelStamp })

	vs := make([]visionSample, 0, len(metas))
	var lost, total int64
	for _, m := range metas {
		total++
		if m.FrameGap > 1 {
			lost += m.FrameGap - 1
		}
		if !m.SelfSeen {
			continue
		}
		stamp := localization.Stamp(m.MappedNs)
		if !m.Mapped || m.MappedNs == 0 {
			stamp = localization.Stamp(m.RecvNs)
		}
		vs = append(vs, visionSample{
			stamp:   stamp,
			arrival: localization.Stamp(m.RecvNs),
			pose: localization.Pose2{
				X: m.SelfXMm / 1000, Y: m.SelfYMm / 1000, Theta: m.SelfThetaRad,
			},
			camera: m.CameraID,
			frame:  m.FrameNumber,
		})
	}
	sort.Slice(vs, func(i, j int) bool { return vs[i].arrival < vs[j].arrival })

	printSummary(logPath, ws, vs, total, lost, wheels)

	kin, err := localization.NewKinematics(cfg.Geometry)
	if err != nil {
		return err
	}
	logical := toLogical(ws, cfg.Geometry)

	if err := reportWheelOnlyGeometry(logical, cfg.Geometry, rearAngle); err != nil {
		fmt.Printf("\nwheel-only geometry check: %v\n", err)
	}

	delay, err := resolveVisionDelay(visionDelay, ws, vs, kin, cfg.Geometry)
	if err != nil {
		return err
	}
	fmt.Printf("\nvision delay compensation in use: %v\n", delay)

	vcomp, wshift := splitDelay(delay)
	ws = shiftWheels(ws, wshift)
	if compare {
		return reportComparison(cfg, ws, vs, logical, vcomp)
	}
	rep, err := replay(cfg, ws, vs, vcomp, nil)
	if err != nil {
		return err
	}
	rep.print("configured")
	return nil
}

func toLogical(ws []sample, g localization.GeometryConfig) [][localization.NumWheels]float64 {
	out := make([][localization.NumWheels]float64, 0, len(ws))
	for _, w := range ws {
		var v [localization.NumWheels]float64
		for slot := 0; slot < localization.NumWheels; slot++ {
			v[g.WheelSlotOrder[slot]] = w.wheels[slot]
		}
		out = append(out, v)
	}
	return out
}

func printSummary(path string, ws []sample, vs []visionSample, total, lost int64, raw []loclog.WheelRecord) {
	dur := ws[len(ws)-1].wheelStamp.Sub(ws[0].wheelStamp)
	fmt.Printf("\n=== %s ===\n", path)
	fmt.Printf("duration              %.1f s\n", dur.Seconds())
	fmt.Printf("wheel samples         %d (%.1f Hz)\n", len(ws), float64(len(ws))/dur.Seconds())
	fmt.Printf("vision packets        %d total, %d with self (%.1f Hz)\n",
		total, len(vs), float64(len(vs))/dur.Seconds())
	if total > 0 {
		fmt.Printf("vision frame loss     %d missing (%.2f%%)\n", lost, 100*float64(lost)/float64(total+lost))
	}
	if len(vs) > 1 {
		gaps := make([]float64, 0, len(vs)-1)
		for i := 1; i < len(vs); i++ {
			gaps = append(gaps, vs[i].stamp.Sub(vs[i-1].stamp).Seconds())
		}
		sort.Float64s(gaps)
		fmt.Printf("vision gap            median %.1f ms, p95 %.1f ms, max %.1f ms\n",
			1000*gaps[len(gaps)/2], 1000*gaps[int(float64(len(gaps))*0.95)], 1000*gaps[len(gaps)-1])
	}
	var vmin, vmax = math.Inf(1), math.Inf(-1)
	for _, w := range raw {
		if w.BatteryV > 0 {
			vmin = math.Min(vmin, w.BatteryV)
			vmax = math.Max(vmax, w.BatteryV)
		}
	}
	if !math.IsInf(vmin, 1) {
		fmt.Printf("battery               %.1f .. %.1f V\n", vmin, vmax)
	}
}

// reportWheelOnlyGeometry は車輪ログだけで幾何を検査する (研究 §2.4 / §2.5)。
func reportWheelOnlyGeometry(logical [][localization.NumWheels]float64, g localization.GeometryConfig, rearAngle float64) error {
	fit, err := localization.FitNullVector(logical)
	if err != nil {
		return err
	}
	kin, err := localization.NewKinematics(g)
	if err != nil {
		return err
	}
	red, err := localization.NewRedundancy(kin)
	if err != nil {
		return err
	}
	fmt.Printf("\n=== wheel-only geometry check (no vision, no clock) ===\n")
	fmt.Printf("fitted null vector    [%+.4f %+.4f %+.4f %+.4f]\n", fit.N[0], fit.N[1], fit.N[2], fit.N[3])
	// **閉形式ではなく数値解を使う。** 閉形式は前後が左右対称な機体でしか
	// 成り立たず、同定値 (55.4 / -57.4) のような非対称な設定では合わない。
	cfgN := normalize(red.NullVector())
	fmt.Printf("configured            [%+.4f %+.4f %+.4f %+.4f]\n", cfgN[0], cfgN[1], cfgN[2], cfgN[3])
	fmt.Printf("identifiability       %.1f  (need > 5; drive forward, sideways and spin)\n", fit.Identifiability)
	fmt.Printf("residual RMS          %.4f rad/s  (= effective wheel noise if not slipping)\n", fit.ResidualRMS)

	if fit.Identifiability < 5 {
		fmt.Println("**the log does not excite all three degrees of freedom; the angle check below is meaningless**")
		return nil
	}
	ang, err := localization.AnglesFromNullVector(fit.N, g.WheelSigns, rearAngle)
	if err != nil {
		return err
	}
	// **この読みは「4 輪の半径が等しい」と仮定したときの角度**である。
	// 実機では半径が 5% 違うので、差が角度に化けて 56 度あたりに出る。
	// CAD の角度 (+-60) が権威と分かった今、正しい読み方は下の半径較正のほう。
	// ここは「角度と半径のどちらかがずれている」ことを示す指標として残す。
	fmt.Printf("front angle           %.2f deg  **if all four radii were equal** (rear assumed %.1f deg)\n",
		ang.PhiDeg, rearAngle)
	fmt.Printf("  55 deg would give   |n_FL| = %.4f\n", localization.PredictedNullVectorMagnitude(55, rearAngle))
	fmt.Printf("  60 deg would give   |n_FL| = %.4f\n", localization.PredictedNullVectorMagnitude(60, rearAngle))
	fmt.Printf("  measured            |n_FL| = %.4f\n", math.Abs(normalize(fit.N)[localization.WheelFL]))
	fmt.Println("  -> the CAD angles are exact (+-60.0000 / +-135.0000); read this as a radius difference instead")
	fmt.Printf("left/right imbalance  front %+.3f, rear %+.3f  (0 = equal radii)\n",
		ang.FrontImbalance, ang.RearImbalance)

	// **角度を固定したまま半径だけを較正する。** これが正しい切り分け (研究 §2.6):
	// 角度は機械加工で決まるので CAD を信用し、ゴム・摩耗・荷重で変わる半径を
	// データから決める。絶対値は決まらないので、設定の平均をそのまま使う。
	mean := 0.0
	for _, r := range g.WheelRadiusM {
		mean += r
	}
	mean /= float64(localization.NumWheels)
	cal, _, err := localization.CalibrateRadii(logical, g, mean)
	if err != nil {
		fmt.Printf("radius calibration    %v\n", err)
		return nil
	}
	fmt.Printf("\ncalibrated radii (angles and arm kept at the configured values):\n")
	fmt.Printf("  %-8s %10s %10s %8s\n", "wheel", "configured", "calibrated", "change")
	names := [localization.NumWheels]string{"FL", "BL", "BR", "FR"}
	for i := 0; i < localization.NumWheels; i++ {
		old := g.WheelRadiusM[i] * 1000
		nw := cal.WheelRadiusM[i] * 1000
		fmt.Printf("  %-8s %8.3f mm %8.3f mm %+7.2f%%\n", names[i], old, nw, 100*(nw/old-1))
	}
	fmt.Println("  (only the ratios are measurable here; the mean comes from vision / the filter's k_v)")
	if calOut != "" {
		if err := writeGeometryJSON(calOut, cal); err != nil {
			return err
		}
		fmt.Printf("  wrote %s\n", calOut)
	}
	return nil
}

// calOut は較正した幾何の書き出し先 (空なら書かない)。
var calOut string

func writeGeometryJSON(path string, g localization.GeometryConfig) error {
	g.Comment = []string{
		"loc_replay が車輪ログから較正した設定。",
		"取付角・モーメントアーム・符号・並びは入力のまま (機械加工で決まる量なので CAD を信用する)。",
		"**車輪半径の比だけ**が車輪ログから決まっている。絶対値は入力の平均をそのまま使っており、",
		"vision 基準の同定か、フィルタの k_v のオンライン較正で詰めること。",
		"詳しくは docs/self-localization-research-20260923.md 2.6。",
	}
	b, err := json.MarshalIndent(g, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(path, append(b, '\n'), 0o644)
}

func normalize(v [localization.NumWheels]float64) [localization.NumWheels]float64 {
	maxAbs := 0.0
	for _, x := range v {
		maxAbs = math.Max(maxAbs, math.Abs(x))
	}
	if maxAbs == 0 {
		return v
	}
	var out [localization.NumWheels]float64
	for i, x := range v {
		out[i] = x / maxAbs
	}
	for i := range out {
		if math.Abs(out[i]) > 1e-9 {
			if out[i] < 0 {
				for j := range out {
					out[j] = -out[j]
				}
			}
			break
		}
	}
	return out
}

// resolveVisionDelay は -vision-delay の指定を解決する。auto なら相互相関で測る。
func resolveVisionDelay(spec string, ws []sample, vs []visionSample,
	kin *localization.Kinematics, g localization.GeometryConfig) (time.Duration, error) {
	if spec != "auto" {
		d, err := time.ParseDuration(spec)
		if err != nil {
			return 0, fmt.Errorf("-vision-delay %q: %w", spec, err)
		}
		return d, nil
	}
	est := localization.NewVisionDelayEstimator(localization.VisionDelayOptions{})
	for _, w := range ws {
		var logical [localization.NumWheels]float64
		for slot := 0; slot < localization.NumWheels; slot++ {
			logical[g.WheelSlotOrder[slot]] = w.wheels[slot]
		}
		vx, vy, _ := kin.BodyFromWheel(logical)
		est.AddWheel(w.wheelStamp, vx, vy)
	}
	for _, v := range vs {
		est.AddVision(localization.VisionPose{Stamp: v.stamp, Pose: v.pose}, 25*time.Millisecond)
	}
	fmt.Printf("\n=== vision delay (wheel vs vision speed correlation) ===\n")
	d, ok := est.Estimate(0.7)
	if !ok {
		fmt.Println("could not measure it: the run needs acceleration (a constant speed leaves it unobservable)")
		return 0, nil
	}
	_, peak, _ := est.Last()
	fmt.Printf("estimated             %v  (correlation peak %.3f)\n", d.Round(time.Millisecond), peak)
	if d < 0 {
		// **負の「vision 遅延」は物理的におかしい。** 露光より前の時刻が
		// 刻印されることはない。負に出るのは**車輪の刻印のほうが遅れている**
		// ときで、実際 PoC の CSV は STM が読んだ時刻ではなく制御周期の時刻を
		// 記録しているので 8-12 ms 遅れる。補正は車輪側に掛けるのが正しい。
		fmt.Printf("negative -> the wheel timestamps are late by %v, not the vision ones.\n",
			(-d).Round(time.Millisecond))
		fmt.Println("shifting the wheel stamps instead (use stmframe timeOffsetMs on the robot)")
	} else {
		fmt.Println("positive = the vision timestamp is later than the true exposure; feed it to VisionDelayComp")
	}
	return d, nil
}

// splitDelay は測った相対遅れを、vision 側の補正と車輪側の補正に振り分ける。
func splitDelay(d time.Duration) (visionComp, wheelShift time.Duration) {
	if d < 0 {
		return 0, d
	}
	return d, 0
}

// shiftWheels は車輪サンプルの刻印をずらした写しを返す。
func shiftWheels(ws []sample, shift time.Duration) []sample {
	if shift == 0 {
		return ws
	}
	out := make([]sample, len(ws))
	copy(out, ws)
	for i := range out {
		out[i].wheelStamp += localization.Stamp(shift)
	}
	return out
}

// report は 1 回のリプレイの結果。
type report struct {
	name       string
	posRMSE    float64
	posMax     float64
	headRMSE   float64
	meanNEES   float64
	stats      localization.EstimatorStats
	params     localization.KinematicParams
	slipRate   float64
	noiseScale float64
	noiseOK    bool
	nis        float64
	nisOK      bool

	// vision を基準にした向きのずれ。**疑似真値 (RTS スムーザ) はフィルタと
	// 同じモデルを共有するので、系統的な誤りには両方が同じだけ外れて見えない。**
	// vision は外から来る唯一の基準なので、発散の検出にはこちらを見る。
	// (精度の指標ではない: フィルタは vision を使って更新しているため)
	headVsVision    float64 // 平均の絶対値 [rad]
	headVsVisionMax float64
	posVsVision     float64 // [m]
	visionRefs      int
}

func (r report) print(label string) {
	fmt.Printf("\n=== replay: %s ===\n", label)
	fmt.Printf("vs RTS smoother       pos RMSE %.2f mm (max %.2f), heading RMSE %.3f deg\n",
		r.posRMSE*1000, r.posMax*1000, r.headRMSE*180/math.Pi)
	if r.visionRefs > 0 {
		fmt.Printf("vs VISION             heading %.2f deg (max %.2f), pos %.2f mm  [%d frames]\n",
			r.headVsVision*180/math.Pi, r.headVsVisionMax*180/math.Pi, r.posVsVision*1000, r.visionRefs)
	}
	fmt.Printf("consistency (NEES)    %.2f vs the smoother (ideal 3.0, but the smoother shares the model)\n", r.meanNEES)
	if r.nisOK {
		fmt.Printf("consistency (NIS)     %.2f  **ideal 3.0, needs no ground truth -- tune Q and R with this**\n", r.nis)
	}
	fmt.Printf("kinematic scale       trans %.4f, rot %.4f, angle %+.2f deg\n",
		r.params.TransScale, r.params.RotScale, r.params.AngleBias*180/math.Pi)
	fmt.Printf("slip rate             %.3f  (> 0.5 means the geometry is wrong, not slipping)\n", r.slipRate)
	if r.noiseOK {
		fmt.Printf("wheel noise model     measured/predicted = %.2f  (1.0 means WheelNoise is right)\n", r.noiseScale)
	}
	fmt.Printf("stats                 %+v\n", r.stats)
	if r.stats.WheelUpdates == 0 {
		fmt.Println("**no wheel updates were applied** -- the wheel and vision timestamps are inconsistent")
	}
}

// buildPseudoTruth は RTS スムーザで疑似真値を作る。
//
// **基準は 1 度だけ作って固定する。** 設定ごとに作り直すと、比較が
// 「その設定のオンライン推定が、その設定の平滑化にどれだけ近いか」になり、
// 精度の比較にならない (実装中に一度これをやって、機能を切ったほうが
// 良く見えるという嘘の表が出た)。
func buildPseudoTruth(cfg localization.Config, ws []sample, vs []visionSample, delay time.Duration) ([]localization.SmoothedState, error) {
	inputs := make([]localization.SmootherInput, 0, len(ws)+len(vs))
	for i := range ws {
		var logical [localization.NumWheels]float64
		for slot := 0; slot < localization.NumWheels; slot++ {
			logical[cfg.Geometry.WheelSlotOrder[slot]] = ws[i].wheels[slot]
		}
		inputs = append(inputs, localization.SmootherInput{
			Stamp: ws[i].wheelStamp, Wheels: logical, HasWheels: true,
		})
	}
	for _, v := range vs {
		inputs = append(inputs, localization.SmootherInput{
			Stamp: v.stamp - localization.Stamp(delay), Vision: v.pose, HasVision: true,
		})
	}
	return localization.Smooth(inputs, cfg)
}

func replay(cfg localization.Config, ws []sample, vs []visionSample, delay time.Duration,
	truth []localization.SmoothedState) (report, error) {
	opts := localization.EstimatorOptions{VisionDelayComp: delay}
	est, err := localization.NewEstimator(cfg, opts)
	if err != nil {
		return report{}, err
	}

	out := make([]localization.Estimate, 0, len(ws))
	vi := 0
	for _, w := range ws {
		for vi < len(vs) && vs[vi].arrival <= w.wheelStamp {
			est.AddVision(localization.VisionPose{
				Stamp: vs[vi].stamp, Arrival: vs[vi].arrival, Pose: vs[vi].pose,
				CameraID: vs[vi].camera, FrameNumber: vs[vi].frame, Mapped: true,
			})
			vi++
		}
		est.AddWheel(localization.WheelSample{Stamp: w.wheelStamp, Omega: w.wheels})
		// 実機と同じ順 (車輪 → IMU → vision) で入れる。
		if w.hasImu && cfg.Noise.EnableGyro {
			est.AddImu(w.imu)
		}
		out = append(out, est.Current())
	}

	// 疑似真値。**同じモデルを使うので、モデル誤差には甘い評価になる。**
	// 遅れと雑音の評価には有効で、実機に真値が無い以上これが最善。
	smoothed := truth
	if smoothed == nil {
		var err error
		smoothed, err = buildPseudoTruth(cfg, ws, vs, delay)
		if err != nil {
			return report{}, err
		}
	}

	rep := report{stats: est.Stats(), slipRate: est.SlipRate()}
	rep.noiseScale, rep.noiseOK = est.WheelNoiseScaleFactor()
	rep.nis, rep.nisOK = est.VisionNIS()
	if len(out) > 0 {
		rep.params = out[len(out)-1].Params
	}

	// 立ち上がりを除いて比べる。
	warm := len(out) / 10
	var sum, headSum, nees float64
	var n int
	si := 0
	for i := warm; i < len(out); i++ {
		e := out[i]
		for si+1 < len(smoothed) && smoothed[si+1].Stamp <= e.Stamp {
			si++
		}
		if si >= len(smoothed) {
			break
		}
		s := smoothed[si]
		if d := e.Stamp.Sub(s.Stamp); d > 10*time.Millisecond || d < -10*time.Millisecond {
			continue
		}
		dx := e.Pose.X - s.Pose.X
		dy := e.Pose.Y - s.Pose.Y
		dth := localization.AngleDiff(e.Pose.Theta, s.Pose.Theta)
		d2 := dx*dx + dy*dy
		sum += d2
		headSum += dth * dth
		if d2 > rep.posMax*rep.posMax {
			rep.posMax = math.Sqrt(d2)
		}
		if v := neesOf(e.CovPose, dx, dy, dth); v > 0 {
			nees += v
		}
		n++
	}
	// vision を基準にした比較。各 vision の撮影時刻に最も近い推定と突き合わせる。
	{
		var hsum, psum float64
		var hmax float64
		var hn int
		oi := 0
		for _, v := range vs {
			for oi+1 < len(out) && out[oi+1].Stamp <= v.stamp {
				oi++
			}
			if oi >= len(out) {
				break
			}
			e := out[oi]
			if d := e.Stamp.Sub(v.stamp); d > 20*time.Millisecond || d < -20*time.Millisecond {
				continue
			}
			dth := math.Abs(localization.AngleDiff(e.Pose.Theta, v.pose.Theta))
			hsum += dth
			if dth > hmax {
				hmax = dth
			}
			psum += math.Hypot(e.Pose.X-v.pose.X, e.Pose.Y-v.pose.Y)
			hn++
		}
		if hn > 0 {
			rep.headVsVision = hsum / float64(hn)
			rep.headVsVisionMax = hmax
			rep.posVsVision = psum / float64(hn)
			rep.visionRefs = hn
		}
	}

	if n > 0 {
		rep.posRMSE = math.Sqrt(sum / float64(n))
		rep.headRMSE = math.Sqrt(headSum / float64(n))
		rep.meanNEES = nees / float64(n)
	}
	return rep, nil
}

// neesOf は [dx, dy, dtheta] の正規化二乗誤差を返す。
func neesOf(cov localization.Mat3, dx, dy, dth float64) float64 {
	// 3x3 の逆行列を直接。
	a := cov
	det := a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1]) -
		a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0]) +
		a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0])
	if math.Abs(det) < 1e-30 {
		return 0
	}
	inv := [3][3]float64{
		{(a[1][1]*a[2][2] - a[1][2]*a[2][1]) / det, (a[0][2]*a[2][1] - a[0][1]*a[2][2]) / det, (a[0][1]*a[1][2] - a[0][2]*a[1][1]) / det},
		{(a[1][2]*a[2][0] - a[1][0]*a[2][2]) / det, (a[0][0]*a[2][2] - a[0][2]*a[2][0]) / det, (a[0][2]*a[1][0] - a[0][0]*a[1][2]) / det},
		{(a[1][0]*a[2][1] - a[1][1]*a[2][0]) / det, (a[0][1]*a[2][0] - a[0][0]*a[2][1]) / det, (a[0][0]*a[1][1] - a[0][1]*a[1][0]) / det},
	}
	e := [3]float64{dx, dy, dth}
	var v float64
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			v += e[i] * inv[i][j] * e[j]
		}
	}
	return v
}

// reportQSweep はプロセス雑音を振って NIS を見る。
//
// **NIS は真値が要らない。** 平均が観測の次元 (3) になる Q が、実機に合った値である。
// NEES は疑似真値が同じモデルで作られるので循環するが、NIS は観測そのものとの
// 食い違いを測るので実機でそのまま使える (Bar-Shalom)。
func reportQSweep(cfg localization.Config, ws []sample, vs []visionSample, delay time.Duration) error {
	fmt.Printf("\n=== process noise sweep (NIS should be 3.0) ===\n")
	fmt.Printf("%12s %12s %10s %12s %12s\n", "accelNoise", "angAccel", "NIS", "huber/vision", "wheel scale")
	for _, a := range []float64{0.5, 1.0, 2.0, 4.0} {
		for _, w := range []float64{2.0, 5.0, 10.0, 20.0} {
			c := cfg
			c.Noise.AccelNoise = a
			c.Noise.AngAccelNoise = w
			rep, err := replay(c, ws, vs, delay, nil)
			if err != nil {
				return err
			}
			nis := math.NaN()
			if rep.nisOK {
				nis = rep.nis
			}
			frac := 0.0
			if rep.stats.VisionUpdates > 0 {
				frac = float64(rep.stats.HuberDownweights) / float64(rep.stats.VisionUpdates)
			}
			fmt.Printf("%12.1f %12.1f %10.2f %12.3f %12.2f\n", a, w, nis, frac, rep.noiseScale)
		}
	}
	return nil
}

// replayRobotFeed は実機 (internal/trajpoc の feedEstimator) と同じ順・同じ刻印で入れる。
//
// 実機: 車輪と IMU を「今 - 8 ms」、vision は撮影時刻。順は 車輪 → IMU → vision。
// 通常のリプレイ: 到着順に vision → 車輪 → IMU で、刻印は記録のまま。
//
// **同じ記録でも、この違いだけで推定が壊れるかを確かめるためにある。**
func replayRobotFeed(cfg localization.Config, ws []sample, vs []visionSample,
	delay time.Duration) (report, error) {
	opts := localization.EstimatorOptions{VisionDelayComp: delay}
	est, err := localization.NewEstimator(cfg, opts)
	if err != nil {
		return report{}, err
	}
	const spiPeriod = 8 * time.Millisecond

	out := make([]localization.Estimate, 0, len(ws))
	vi := 0
	var lastTV localization.Stamp = -1
	for _, w := range ws {
		at := w.wheelStamp - localization.Stamp(spiPeriod)
		est.AddWheel(localization.WheelSample{Stamp: at, Omega: w.wheels})
		if w.hasImu {
			im := w.imu
			im.Stamp = at
			est.AddImu(im)
		}
		// 実機は「その周期に見えている vision」を、撮影時刻が進んだときだけ入れる。
		for vi < len(vs) && vs[vi].arrival <= w.wheelStamp {
			vi++
		}
		if vi > 0 {
			v := vs[vi-1]
			if v.stamp > lastTV {
				lastTV = v.stamp
				est.AddVision(localization.VisionPose{Stamp: v.stamp, Pose: v.pose, Confidence: 1})
			}
		}
		out = append(out, est.Current())
	}

	rep := report{stats: est.Stats(), slipRate: est.SlipRate()}
	var hsum, psum, hmax float64
	var hn, oi int
	for _, v := range vs {
		for oi+1 < len(out) && out[oi+1].Stamp <= v.stamp {
			oi++
		}
		if oi >= len(out) {
			break
		}
		e := out[oi]
		if d := e.Stamp.Sub(v.stamp); d > 20*time.Millisecond || d < -20*time.Millisecond {
			continue
		}
		dth := math.Abs(localization.AngleDiff(e.Pose.Theta, v.pose.Theta))
		hsum += dth
		if dth > hmax {
			hmax = dth
		}
		psum += math.Hypot(e.Pose.X-v.pose.X, e.Pose.Y-v.pose.Y)
		hn++
	}
	if hn > 0 {
		rep.headVsVision, rep.headVsVisionMax = hsum/float64(hn), hmax
		rep.posVsVision, rep.visionRefs = psum/float64(hn), hn
	}
	return rep, nil
}

// reportComparison は機能ごとの効果を並べる。
//
// **効いていないものを黙って既定にしないため**の表である
// (計画 §9「効果が測れること」を機能要件にする)。
func reportComparison(cfg localization.Config, ws []sample, vs []visionSample,
	logical [][localization.NumWheels]float64, delay time.Duration) error {
	type variant struct {
		name   string
		mutate func(*localization.Config)
	}
	variants := []variant{
		{"all features", func(c *localization.Config) {}},
		{"no param estimation", func(c *localization.Config) { c.Noise.EnableParamEstimation = false }},
		{"no slip state", func(c *localization.Config) { c.Noise.EnableSlip = false }},
		{"no ZUPT", func(c *localization.Config) { c.Noise.EnableZupt = false }},
		{"no adaptive R", func(c *localization.Config) { c.Noise.AdaptiveVisionR = false }},
		// IMU を積む価値を測るための項目。ジャイロを ω の観測として使うのをやめると、
		// 向きは車輪と vision だけから決まる (2026-09 に IMU が届くようになったので追加)。
		{"no gyro (IMU off)", func(c *localization.Config) { c.Noise.EnableGyro = false }},
		{"constant wheel noise", func(c *localization.Config) { c.Noise.WheelNoiseSpeedCoef = 0 }},
		{"no delay compensation", func(c *localization.Config) {}},
	}

	// **基準は全機能ありの設定で 1 度だけ作り、全変種で使い回す。**
	truth, err := buildPseudoTruth(cfg, ws, vs, delay)
	if err != nil {
		return err
	}

	fmt.Printf("\n=== feature comparison (vs a FIXED RTS smoother reference) ===\n")
	fmt.Printf("%-24s %12s %10s %10s %8s %6s\n",
		"variant", "head vs VISION", "pos RMSE", "head RMSE", "NEES", "slip")
	for _, v := range variants {
		c := cfg
		v.mutate(&c)
		d := delay
		if v.name == "no delay compensation" {
			d = 0
		}
		rep, err := replay(c, ws, vs, d, truth)
		if err != nil {
			return err
		}
		fmt.Printf("%-24s %9.2f dg %7.2f mm %8.3f dg %8.2f %6.3f\n",
			v.name, rep.headVsVision*180/math.Pi, rep.posRMSE*1000,
			rep.headRMSE*180/math.Pi, rep.meanNEES, rep.slipRate)
	}
	return nil
}
