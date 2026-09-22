// loc_ident は P1 で記録した MCAP から機体パラメータを同定する。
//
// 確定させるのは計画 §12-A の 5 項目:
//
//	A-1 車輪半径          26 / 27 / 30 mm のどれか
//	A-2 モーメントアーム  75 / 85 / 89 / 90 mm のどれか
//	A-3 車輪取付角        55 度か 60 度か
//	A-4 ホイール番号と取付角の対応 (FL / FR が入れ替わっている疑い)
//	A-5 符号規約          新世代 STM だけ反転している疑い
//
// A-4 と A-5 は「RAVEN の EKF は効果が測れなかった」の最有力の原因候補であり、
// これを潰すのが P1 の最優先事項である (計画 §9)。
//
// 使い方:
//
//	loc_ident -log racoon-loc-*.mcap                 レポートを表示
//	loc_ident -log x.mcap -out geometry.json         同定結果を設定として書き出す
//	loc_ident -log x.mcap -ref vision                vision を基準にする (絶対寸法が出る)
package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"math"
	"os"
	"sort"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi2/internal/loclog"
)

func main() {
	logPath := flag.String("log", "", "P1 で記録した MCAP ファイル (必須)")
	outPath := flag.String("out", "", "同定した機体パラメータの書き出し先 JSON")
	refName := flag.String("ref", "command", "基準速度: command | vision")
	settle := flag.Duration("settle", 150*time.Millisecond, "指令が変わってから使い始めるまでの待ち時間")
	minSamples := flag.Int("min-samples", 200, "同定に必要な最小サンプル数")
	verbose := flag.Bool("v", false, "スロットごとの係数も表示する")
	flag.Parse()

	if *logPath == "" {
		fmt.Fprintln(os.Stderr, "loc_ident: -log is required")
		flag.Usage()
		os.Exit(2)
	}

	if err := run(*logPath, *outPath, *refName, *settle, *minSamples, *verbose); err != nil {
		fmt.Fprintf(os.Stderr, "loc_ident: %v\n", err)
		os.Exit(1)
	}
}

func run(logPath, outPath, refName string, settle time.Duration, minSamples int, verbose bool) error {
	ref := localization.IdentReference(refName)
	switch ref {
	case localization.RefCommand, localization.RefVision:
	default:
		return fmt.Errorf("unknown -ref %q (want command or vision)", refName)
	}
	if ref == localization.RefVision {
		// vision 基準は絶対寸法まで出せるが、vision の数値微分が要る。
		// 微分の窓長と遅延補償が効くので、P2 (timesync) の後でないと意味が無い。
		return fmt.Errorf("-ref vision is not implemented yet; it needs the timesync stage (plan P2) " +
			"so that vision timestamps can be mapped onto the robot clock before differentiating")
	}

	r, err := loclog.OpenReader(logPath)
	if err != nil {
		return err
	}
	defer r.Close()

	if meta, err := r.Metadata(); err == nil && len(meta) > 0 {
		keys := make([]string, 0, len(meta))
		for k := range meta {
			keys = append(keys, k)
		}
		sort.Strings(keys)
		fmt.Println("recording metadata:")
		for _, k := range keys {
			fmt.Printf("  %-30s %s\n", k, meta[k])
		}
		fmt.Println()
	}

	wheels, err := loclog.ReadJSONInto[loclog.WheelRecord](r, loclog.ChWheel.Topic())
	if err != nil {
		return err
	}
	commands, err := loclog.ReadJSONInto[loclog.CommandRecord](r, loclog.ChOutCommand.Topic())
	if err != nil {
		return err
	}
	fmt.Printf("read %d wheel samples and %d commands from %s\n", len(wheels), len(commands), logPath)

	samples, stats := pair(wheels, commands, settle)
	fmt.Printf("paired %d samples (%d dropped: %d unmatched, %d still settling)\n\n",
		len(samples), stats.unmatched+stats.settling, stats.unmatched, stats.settling)

	res, err := localization.Identify(samples, ref, localization.IdentOptions{MinSamples: minSamples})
	if err != nil {
		return err
	}
	report(res, verbose)

	if outPath != "" {
		if err := writeGeometry(outPath, res.Geometry); err != nil {
			return err
		}
		fmt.Printf("\nwrote identified geometry to %s\n", outPath)
	}
	return nil
}

type pairStats struct {
	unmatched int
	settling  int
}

// pair は転送時刻で車輪サンプルと指令を突き合わせる。
//
// 指令は目標であって実測ではないので、モータが追従しきる前のサンプルを使うと
// 係数が小さく出る。指令が変わってから settle 経過したものだけを採用する。
func pair(wheels []loclog.WheelRecord, commands []loclog.CommandRecord, settle time.Duration) ([]localization.IdentSample, pairStats) {
	byTransfer := make(map[int64]loclog.CommandRecord, len(commands))
	for _, c := range commands {
		byTransfer[c.TransferNs] = c
	}

	// 指令が変わった時刻を拾う。
	sorted := append([]loclog.CommandRecord(nil), commands...)
	sort.Slice(sorted, func(i, j int) bool { return sorted[i].TransferNs < sorted[j].TransferNs })

	changedAt := make(map[int64]int64, len(sorted))
	var lastChange int64
	var prev loclog.CommandRecord
	for i, c := range sorted {
		if i == 0 || c.VelXMmS != prev.VelXMmS || c.VelYMmS != prev.VelYMmS || c.VelAngMradS != prev.VelAngMradS {
			lastChange = c.TransferNs
		}
		changedAt[c.TransferNs] = lastChange
		prev = c
	}

	var out []localization.IdentSample
	var stats pairStats
	for _, w := range wheels {
		c, ok := byTransfer[w.TransferNs]
		if !ok {
			stats.unmatched++
			continue
		}
		if w.TransferNs-changedAt[w.TransferNs] < int64(settle) {
			stats.settling++
			continue
		}
		out = append(out, localization.IdentSample{
			// SPI フレーム上は mm/s と mrad/s。コアは SI なのでここで直す。
			VX:    float64(c.VelXMmS) / 1000,
			VY:    float64(c.VelYMmS) / 1000,
			Omega: float64(c.VelAngMradS) / 1000,
			WheelSlots: [localization.NumWheels]float64{
				w.WheelFLRadS, w.WheelBLRadS, w.WheelBRRadS, w.WheelFRRadS,
			},
		})
	}
	return out, stats
}

func report(res localization.IdentResult, verbose bool) {
	fmt.Printf("reference: %s   samples: %d   condition number: %.1f\n",
		res.Reference, res.Samples, res.ConditionNumber)
	fmt.Printf("excitation rms: vx %.3f m/s   vy %.3f m/s   omega %.3f rad/s\n\n",
		res.ExcitationVX, res.ExcitationVY, res.ExcitationOmega)

	fmt.Println("per-slot fit (slot = position in the SPI frame):")
	for _, s := range res.Slots {
		fmt.Println("  " + s.Summary())
		if verbose {
			fmt.Printf("      a=%+0.6f  b=%+0.6f  c=%+0.6f\n", s.A, s.B, s.C)
		}
	}

	g := res.Geometry
	fmt.Println("\nidentified geometry:")
	fmt.Printf("  wheelSlotOrder  %v   (slot -> logical wheel; identity is [0 1 2 3] = FL BL BR FR)\n", g.WheelSlotOrder)
	fmt.Printf("  wheelSigns      %v\n", g.WheelSigns)
	fmt.Printf("  wheelAnglesDeg  [%.2f %.2f %.2f %.2f]\n",
		g.WheelAnglesDeg[0], g.WheelAnglesDeg[1], g.WheelAnglesDeg[2], g.WheelAnglesDeg[3])
	fmt.Printf("  wheelRadiusMm   [%.2f %.2f %.2f %.2f]\n",
		g.WheelRadiusM[0]*1000, g.WheelRadiusM[1]*1000, g.WheelRadiusM[2]*1000, g.WheelRadiusM[3]*1000)
	fmt.Printf("  momentArmMm     %.2f\n", g.MomentArmM*1000)

	fmt.Println("\nverdict:")
	def := localization.DefaultGeometry()
	if g.WheelSlotOrder != def.WheelSlotOrder {
		fmt.Printf("  ! the SPI wheel order is NOT the assumed FL BL BR FR (plan A-4 confirmed)\n")
	} else {
		fmt.Printf("  the SPI wheel order matches the assumed FL BL BR FR\n")
	}
	flipped := 0
	for _, s := range g.WheelSigns {
		if s < 0 {
			flipped++
		}
	}
	switch flipped {
	case 0:
		fmt.Printf("  the sign convention matches the old-gen STM and RAVEN (plan A-5: not flipped)\n")
	case localization.NumWheels:
		fmt.Printf("  ! all four wheels are sign-flipped; this matches the new-gen omni_drive.c (plan A-5 confirmed)\n")
		fmt.Printf("    this is the most likely reason RAVEN's encoder EKF showed no measurable benefit\n")
	default:
		fmt.Printf("  ! %d of 4 wheels are sign-flipped; that is not a convention difference, it is wiring\n", flipped)
	}
	reportDimension("wheel radius", g.WheelRadiusM[0]*1000,
		[]namedValue{{"old-gen STM", 27}, {"new-gen STM", 30}, {"RAVEN", 26}})
	reportDimension("moment arm", g.MomentArmM*1000,
		[]namedValue{{"old-gen STM", 85}, {"new-gen STM", 75}, {"RAVEN", 90}, {"ROBOT_RADIUS", 89}})

	if len(res.Warnings) > 0 {
		fmt.Println("\nwarnings:")
		for _, w := range res.Warnings {
			fmt.Println("  ! " + w)
		}
	}
}

type namedValue struct {
	name string
	mm   float64
}

func reportDimension(label string, gotMm float64, candidates []namedValue) {
	best, bestErr := "", math.Inf(1)
	for _, c := range candidates {
		if e := math.Abs(gotMm - c.mm); e < bestErr {
			best, bestErr = fmt.Sprintf("%s (%.0f mm)", c.name, c.mm), e
		}
	}
	fmt.Printf("  %s %.2f mm is closest to %s, off by %.2f mm\n", label, gotMm, best, bestErr)
}

func writeGeometry(path string, g localization.GeometryConfig) error {
	data, err := json.MarshalIndent(g, "", "  ")
	if err != nil {
		return fmt.Errorf("marshal geometry: %w", err)
	}
	data = append(data, '\n')
	if err := os.WriteFile(path, data, 0o644); err != nil {
		return fmt.Errorf("write %s: %w", path, err)
	}
	return nil
}
