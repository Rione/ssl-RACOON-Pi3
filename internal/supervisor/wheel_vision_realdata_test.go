package supervisor

import (
	"encoding/csv"
	"os"
	"path/filepath"
	"strconv"
	"testing"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// 実機の走行で「車輪と vision の食い違い」の検査が正しく判定するか。
//
// 2026-09-22 に、vision の模様が止まっている別の物になっていて、ロボットが見張られないまま
// 走った (docs/traj-poc-log.md §5-18)。枠は vision の位置で判定するので効かない。
// ここでは、そのときの記録と正常な記録の両方を流して、止めるべきときだけ止まることを確かめる。
func TestWheelVisionCheckOnRealRuns(t *testing.T) {
	for _, c := range []struct {
		file string
		stop bool
		note string
	}{
		{"trajpoc-20260922-131552-ffp_vlead-hermite.csv", false, "正常な円の走り"},
		{"trajpoc-20260922-131559-ffp_vlead-hermite.csv", false, "正常なその場回転"},
		{"trajpoc-20260922-125411-ffp_vlead-hermite.csv", true, "vision の模様が別の物 (円)"},
		{"trajpoc-20260922-125417-ffp_vlead-hermite.csv", true, "vision の模様が別の物 (その場回転)"},
		// 2026-09-24: 床の traction が落ちて激しく滑った 2 本。車輪は回っているのに
		// 機体が進まず、食いついた瞬間に飛び出す。**これは止めてはいけない。**
		// 模様の取り違えは vision/車輪の比が 0 のまま続くのに対し、滑りは一瞬沈んで戻る
		// (実測の中央値 0.81〜0.95)。区別できないと、滑りやすい場所で走れなくなる。
		{"trajpoc-20260924-075514-ffp_vlead.csv", false, "床が滑る (中止された回)"},
		{"trajpoc-20260924-075522-ffp_vlead.csv", false, "床が滑る (完走した回)"},
	} {
		path := filepath.Join("..", "..", "trajpoc-dataset", "poc", c.file)
		samples, err := readCheckSamples(path)
		if err != nil {
			t.Skipf("%s: %v", c.file, err) // データセットを置いていない環境では飛ばす
		}
		at, reason, stopped := ReplayWheelCheck(localization.DefaultGeometry(), samples)
		if stopped != c.stop {
			t.Errorf("%s (%s): stopped=%v, want %v (%s)", c.file, c.note, stopped, c.stop, reason)
			continue
		}
		if stopped {
			t.Logf("%s (%s): t=%.2f s で止めた: %s", c.file, c.note, at, reason)
		}
	}
}

// readCheckSamples は PoC の CSV から検査に要る列だけを読む。
func readCheckSamples(path string) ([]CheckSample, error) {
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
			return nil, os.ErrNotExist // 車輪を記録する前の CSV
		}
	}
	var out []CheckSample
	for {
		rec, err := r.Read()
		if err != nil {
			break
		}
		num := func(n string) float64 {
			v, _ := strconv.ParseFloat(rec[col[n]], 64)
			return v
		}
		out = append(out, CheckSample{
			T: num("t_s"), TV: num("tv_s"),
			Pose: localization.Pose2{X: num("pose_x_mm") / 1000, Y: num("pose_y_mm") / 1000, Theta: num("pose_theta_rad")},
			Wheels: [4]float64{num("wheel_fl_rad_s"), num("wheel_bl_rad_s"),
				num("wheel_br_rad_s"), num("wheel_fr_rad_s")},
		})
	}
	return out, nil
}
