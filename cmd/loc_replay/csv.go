package main

import (
	"encoding/csv"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// 軌道追従 PoC が残した CSV を読む (trajpoc-dataset/poc/*.csv)。
//
// MCAP がまだ手元に無いあいだ、**実機で取れているのはこの形だけ**なので
// 読めるようにしてある。1 行が制御の 1 周期 (125 Hz) で、
//
//	t_s             軌道の時刻 [s] (走り出し前は負)
//	tv_s            その行で見ていた vision の撮影時刻 [s]
//	pose_*          vision で見た姿勢 (生値、mm と rad)
//	wheel_*_rad_s   STM から届いた 4 輪の回転速度 (17 本のみ)
//	imu_*           STM から届いた IMU (2026-09 以降の記録のみ)
//	battery_v       電池電圧 [V] (2026-09-24 以降の記録のみ)。
//	                指令から動き出しまでの遅れが電圧で変わるので、
//	                記録どうしを比べるときは必ず突き合わせる。
//
// 同じ撮影が複数行に出るので、vision は tv_s で重複を落とす。

// poCRun は 1 本の走りから取り出した入力。
type poCRun struct {
	path   string
	wheels []sample
	vision []visionSample
	// hasWheels は車輪の列があったか。
	hasWheels bool
	// hasImu は IMU の列があったか。古い記録には無い。
	hasImu bool
	// battery は記録中の電池電圧 [V] の平均。0 なら列が無い (2026-09-24 より前の記録)。
	battery float64
}

func readPoCCSV(path string) (*poCRun, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	r := csv.NewReader(f)
	r.FieldsPerRecord = -1
	head, err := r.Read()
	if err != nil {
		return nil, fmt.Errorf("%s: read header: %w", path, err)
	}
	col := make(map[string]int, len(head))
	for i, h := range head {
		col[h] = i
	}
	need := []string{"t_s", "tv_s", "pose_x_mm", "pose_y_mm", "pose_theta_rad"}
	for _, n := range need {
		if _, ok := col[n]; !ok {
			return nil, fmt.Errorf("%s: missing column %q", path, n)
		}
	}
	wheelCols := []string{"wheel_fl_rad_s", "wheel_bl_rad_s", "wheel_br_rad_s", "wheel_fr_rad_s"}
	hasWheels := true
	for _, n := range wheelCols {
		if _, ok := col[n]; !ok {
			hasWheels = false
		}
	}

	imuCols := []string{"imu_valid", "imu_yaw_rate_rad_s", "imu_accel_x_m_s2", "imu_accel_y_m_s2"}
	hasImu := true
	for _, n := range imuCols {
		if _, ok := col[n]; !ok {
			hasImu = false
		}
	}

	out := &poCRun{path: path, hasWheels: hasWheels, hasImu: hasImu}
	var battSum float64
	var battN int
	lastTv := math.NaN()
	for {
		rec, err := r.Read()
		if err == io.EOF {
			break
		}
		if err != nil {
			return nil, fmt.Errorf("%s: %w", path, err)
		}
		get := func(name string) float64 {
			i, ok := col[name]
			if !ok || i >= len(rec) {
				return math.NaN()
			}
			v, err := strconv.ParseFloat(rec[i], 64)
			if err != nil {
				return math.NaN()
			}
			return v
		}
		t := get("t_s")
		if math.IsNaN(t) {
			continue
		}
		stamp := localization.Stamp(time.Duration(t * float64(time.Second)))

		if hasWheels {
			var w [localization.NumWheels]float64
			ok := true
			for i, n := range wheelCols {
				w[i] = get(n)
				if math.IsNaN(w[i]) {
					ok = false
				}
			}
			if ok {
				sm := sample{wheelStamp: stamp, wheels: w, hasWheel: true}
				// IMU は車輪と同じ SPI フレームで届くので、時刻も同じ。
				if hasImu && truthy(rec, col, "imu_valid") {
					gz := get("imu_yaw_rate_rad_s")
					ax, ay := get("imu_accel_x_m_s2"), get("imu_accel_y_m_s2")
					if !math.IsNaN(gz) {
						sm.imu = localization.ImuSample{
							Stamp: stamp, GyroZ: gz, HasGyro: true,
						}
						if !math.IsNaN(ax) && !math.IsNaN(ay) {
							sm.imu.Accel = localization.Vec2{X: ax, Y: ay}
							sm.imu.HasAccel = true
						}
						sm.hasImu = true
					}
				}
				out.wheels = append(out.wheels, sm)
			}
		}

		if v := get("battery_v"); !math.IsNaN(v) && v > 0 {
			battSum += v
			battN++
		}

		tv := get("tv_s")
		if !math.IsNaN(tv) && tv != lastTv {
			lastTv = tv
			x, y, th := get("pose_x_mm"), get("pose_y_mm"), get("pose_theta_rad")
			if !math.IsNaN(x) && !math.IsNaN(y) && !math.IsNaN(th) {
				out.vision = append(out.vision, visionSample{
					stamp:   localization.Stamp(time.Duration(tv * float64(time.Second))),
					arrival: stamp,
					pose:    localization.Pose2{X: x / 1000, Y: y / 1000, Theta: th},
				})
			}
		}
	}
	if battN > 0 {
		out.battery = battSum / float64(battN)
	}
	sort.Slice(out.wheels, func(i, j int) bool { return out.wheels[i].wheelStamp < out.wheels[j].wheelStamp })
	sort.Slice(out.vision, func(i, j int) bool { return out.vision[i].arrival < out.vision[j].arrival })
	return out, nil
}

// truthy は CSV の真偽の列を読む。書き手によって "1" と "true" のどちらもありうる。
func truthy(rec []string, col map[string]int, name string) bool {
	i, ok := col[name]
	if !ok || i >= len(rec) {
		return false
	}
	switch rec[i] {
	case "1", "true", "TRUE", "True":
		return true
	}
	return false
}

// loadPoCRuns はパターンに合う CSV をすべて読む。
func loadPoCRuns(pattern string) ([]*poCRun, error) {
	paths, err := filepath.Glob(pattern)
	if err != nil {
		return nil, err
	}
	if len(paths) == 0 {
		return nil, fmt.Errorf("no files match %q", pattern)
	}
	sort.Strings(paths)
	runs := make([]*poCRun, 0, len(paths))
	for _, p := range paths {
		run, err := readPoCCSV(p)
		if err != nil {
			fmt.Fprintf(os.Stderr, "skip %s: %v\n", p, err)
			continue
		}
		runs = append(runs, run)
	}
	if len(runs) == 0 {
		return nil, fmt.Errorf("no readable runs in %q", pattern)
	}
	return runs, nil
}
