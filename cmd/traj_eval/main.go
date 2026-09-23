// traj_eval は RAVEN の軌道追従の比較 (--trajpoc) の記録を、ロボット上の PoC と同じ指標で評価する。
//
//	traj_eval -ref raven-<日時>-id15-ref.jsonl -raven raven-<日時>-id15.csv -vision rec.csv [-csv out.csv]
//
// 位置は vision の生の記録 (vision_rec) を使う (RAVEN の追跡の推定値ではなく、PoC と同じ物差し)。
// 撮影時刻は「到着の単調時刻 − (送信 − 撮影)」で RAVEN の System.nanoTime の時間軸に写す。
// 指令は RAVEN の記録 (制御の tick ごと) から、撮影の時刻の直前の値を使う。
package main

import (
	"bufio"
	"encoding/csv"
	"flag"
	"fmt"
	"io"
	"math"
	"os"
	"regexp"
	"sort"
	"strconv"
	"strings"

	"github.com/Rione/ssl-RACOON-Pi3/internal/control"
	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi3/internal/trajpoc"
)

func main() {
	refPath := flag.String("ref", "", "RAVEN が書いた貼り付け後の軌道 (-ref.jsonl)")
	ravenPath := flag.String("raven", "", "RAVEN の tick ごとの記録 (.csv)")
	visPath := flag.String("vision", "", "vision_rec の記録")
	outCSV := flag.String("csv", "", "PoC と同じ形の CSV を書く (任意)")
	flag.Parse()
	if *refPath == "" || *ravenPath == "" || *visPath == "" {
		fmt.Fprintln(os.Stderr, "usage: traj_eval -ref x-ref.jsonl -raven x.csv -vision rec.csv")
		os.Exit(2)
	}

	// 軌道: t は System.nanoTime の絶対値。先頭の注釈の t0_ns を引いて相対秒にする
	raw, err := os.ReadFile(*refPath)
	check(err)
	m := regexp.MustCompile(`t0_ns=(\d+)`).FindSubmatch(raw)
	if m == nil {
		check(fmt.Errorf("%s: t0_ns not found", *refPath))
	}
	t0, _ := strconv.ParseInt(string(m[1]), 10, 64)
	knots, err := trajpoc.ReadTrajectory(strings.NewReader(string(raw)))
	check(err)
	for i := range knots {
		knots[i].T -= float64(t0) * 1e-9
	}
	cfg := trajpoc.DefaultConfig()
	// 評価は本番と同じ追従器の参照で行う (RAVEN が実際に追った点列を control のノードにする)。
	ctl, err := control.New(trajpoc.ToNodes(knots, 0, true), cfg.ControlConfig())
	check(err)

	// RAVEN の指令 (tick ごと)
	type tick struct {
		t     float64
		cmd   localization.Vec2
		omega float64
	}
	var ticks []tick
	rows := readCSV(*ravenPath)
	for _, r := range rows {
		tns, _ := strconv.ParseInt(r["t_ns"], 10, 64)
		ticks = append(ticks, tick{t: float64(tns-t0) * 1e-9,
			cmd:   localization.Vec2{X: num(r["cmd_world_vx_mm_s"]) / 1000, Y: num(r["cmd_world_vy_mm_s"]) / 1000},
			omega: num(r["cmd_omega_rad_s"])})
	}
	cmdAt := func(t float64) (localization.Vec2, float64) {
		i := sort.Search(len(ticks), func(i int) bool { return ticks[i].t > t }) - 1
		if i < 0 {
			return localization.Vec2{}, 0
		}
		return ticks[i].cmd, ticks[i].omega
	}
	if len(ticks) == 0 {
		check(fmt.Errorf("%s: no ticks", *ravenPath))
	}
	tEnd := ticks[len(ticks)-1].t

	// vision
	var samples []trajpoc.Sample
	for _, r := range readCSV(*visPath) {
		arr, _ := strconv.ParseInt(r["arrival_mono_ns"], 10, 64)
		capNs := arr - int64((num(r["t_sent_s"])-num(r["t_capture_s"]))*1e9)
		tv := float64(capNs-t0) * 1e-9
		ta := float64(arr-t0) * 1e-9
		if tv < -0.3 || ta > tEnd {
			continue
		}
		c, w := cmdAt(ta)
		s := trajpoc.Sample{T: ta, TV: tv, Age: ta - tv, Capture: localization.Stamp(capNs),
			Pose:     localization.Pose2{X: num(r["x_mm"]) / 1000, Y: num(r["y_mm"]) / 1000, Theta: num(r["theta_rad"])},
			CmdWorld: c, CmdOmega: w}
		s.Ref, s.Phase = refAt(ctl, knots, localization.Stamp(tv*1e9))
		samples = append(samples, s)
	}
	if len(samples) == 0 {
		check(fmt.Errorf("no vision frames inside the run (clock mismatch?)"))
	}
	met := trajpoc.ComputeMetrics(samples, knots)
	cfg.Method = "raven_oc"
	fmt.Print(met.Format(cfg, trajpoc.Done, "RAVEN optimal-control tracker ("+*ravenPath+")"))
	if *outCSV != "" {
		f, err := os.Create(*outCSV)
		check(err)
		check(trajpoc.WriteCSV(f, samples))
		f.Close()
		fmt.Println("csv:", *outCSV)
	}
}

// refAt は評価用の参照。control は軌道の前後でゼロの参照を返すので、
// 端の点で静止しているものとして埋める (PoC 側の Driver と同じ扱い)。
func refAt(ctl *control.Controller, knots []trajpoc.Knot, at localization.Stamp) (control.Reference, control.Phase) {
	r, phase := ctl.ReferenceAt(at)
	switch phase {
	case control.Waiting:
		r = control.Reference{Stamp: at, Pose: knots[0].Pose}
	case control.Finished:
		r = control.Reference{Stamp: at, Pose: knots[len(knots)-1].Pose}
	}
	return r, phase
}

func readCSV(path string) []map[string]string {
	f, err := os.Open(path)
	check(err)
	defer f.Close()
	br := bufio.NewReader(f)
	// 先頭の # の行を飛ばす
	for {
		b, err := br.Peek(1)
		if err != nil || b[0] != '#' {
			break
		}
		if _, err := br.ReadString('\n'); err != nil {
			break
		}
	}
	r := csv.NewReader(br)
	head, err := r.Read()
	check(err)
	var out []map[string]string
	for {
		rec, err := r.Read()
		if err == io.EOF {
			break
		}
		check(err)
		m := map[string]string{}
		for i, h := range head {
			m[h] = rec[i]
		}
		out = append(out, m)
	}
	return out
}

func num(s string) float64 {
	v, err := strconv.ParseFloat(s, 64)
	if err != nil {
		return math.NaN()
	}
	return v
}

func check(err error) {
	if err != nil {
		fmt.Fprintln(os.Stderr, "traj_eval:", err)
		os.Exit(1)
	}
}
