package main

import (
	"encoding/json"
	"math"
	"math/rand"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi3/internal/loclog"
)

// 記録 -> 読み返し -> 同定 を通しで回す。
//
// 実機に繋がっていなくても、「P1 のログを取れば符号と順序が出る」ことを
// ここで確かめられる。実機ログが来たら同じ経路がそのまま動く。
func TestEndToEndIdentificationThroughMCAP(t *testing.T) {
	want := localization.DefaultGeometry()
	// 計画 §12 が疑っている最悪の組み合わせ: 符号反転 + FL/FR 入れ替わり。
	want.WheelSlotOrder = [localization.NumWheels]int{
		localization.WheelFR, localization.WheelBL, localization.WheelBR, localization.WheelFL,
	}
	want.WheelSigns = [localization.NumWheels]float64{-1, -1, -1, -1}
	want.MomentArmM = 0.085
	r := 0.027
	want.WheelRadiusM = [localization.NumWheels]float64{r, r, r, r}

	path := filepath.Join(t.TempDir(), "ident.mcap")
	writeSyntheticLog(t, path, want)

	rd, err := loclog.OpenReader(path)
	if err != nil {
		t.Fatalf("OpenReader: %v", err)
	}
	defer rd.Close()

	wheels, err := loclog.ReadJSONInto[loclog.WheelRecord](rd, loclog.ChWheel.Topic())
	if err != nil {
		t.Fatalf("read wheels: %v", err)
	}
	commands, err := loclog.ReadJSONInto[loclog.CommandRecord](rd, loclog.ChOutCommand.Topic())
	if err != nil {
		t.Fatalf("read commands: %v", err)
	}
	if len(wheels) == 0 || len(commands) == 0 {
		t.Fatalf("log is empty: %d wheels, %d commands", len(wheels), len(commands))
	}

	samples, stats := pair(wheels, commands, 150*time.Millisecond)
	if len(samples) < 200 {
		t.Fatalf("only %d samples survived pairing (%+v)", len(samples), stats)
	}
	if stats.unmatched != 0 {
		t.Errorf("%d wheel samples had no matching command", stats.unmatched)
	}
	// 整定待ちで落としたぶんがあること (落ちていないなら窓が効いていない)。
	if stats.settling == 0 {
		t.Error("no samples were dropped while settling; the settle window is not being applied")
	}

	res, err := localization.Identify(samples, localization.RefCommand, localization.IdentOptions{})
	if err != nil {
		t.Fatalf("Identify: %v", err)
	}

	if res.Geometry.WheelSlotOrder != want.WheelSlotOrder {
		t.Errorf("wheelSlotOrder = %v, want %v", res.Geometry.WheelSlotOrder, want.WheelSlotOrder)
	}
	if res.Geometry.WheelSigns != want.WheelSigns {
		t.Errorf("wheelSigns = %v, want %v", res.Geometry.WheelSigns, want.WheelSigns)
	}
	for i := 0; i < localization.NumWheels; i++ {
		if d := math.Abs(res.Geometry.WheelRadiusM[i]-r) * 1000; d > 0.5 {
			t.Errorf("wheel %d radius off by %.3f mm", i, d)
		}
	}
	if d := math.Abs(res.Geometry.MomentArmM-want.MomentArmM) * 1000; d > 1.0 {
		t.Errorf("momentArm off by %.3f mm", d)
	}

	// 書き出した JSON をそのまま読み戻せること。
	out := filepath.Join(t.TempDir(), "geometry.json")
	if err := writeGeometry(out, res.Geometry); err != nil {
		t.Fatalf("writeGeometry: %v", err)
	}
	data, err := os.ReadFile(out)
	if err != nil {
		t.Fatal(err)
	}
	var back localization.GeometryConfig
	if err := json.Unmarshal(data, &back); err != nil {
		t.Fatalf("the emitted geometry is not valid JSON: %v", err)
	}
	if err := back.Validate(); err != nil {
		t.Errorf("the emitted geometry does not validate: %v", err)
	}
	if _, err := localization.NewKinematics(back); err != nil {
		t.Errorf("the emitted geometry cannot build kinematics: %v", err)
	}
}

// writeSyntheticLog は実機の P1 ログを模した MCAP を書く。
// 指令は 8 ms 周期の量子化 (mm/s, mrad/s) を通し、車輪は一次遅れで追従させる。
func writeSyntheticLog(t *testing.T, path string, g localization.GeometryConfig) {
	t.Helper()
	k, err := localization.NewKinematics(g)
	if err != nil {
		t.Fatal(err)
	}
	clock := loclog.NewClockAt(time.Now())
	w, err := loclog.NewWriter(clock, loclog.Options{
		Path:           path,
		QueueSize:      1 << 16,
		StatusInterval: time.Hour,
		StmProfile:     "rock5a-v1",
	})
	if err != nil {
		t.Fatal(err)
	}

	rng := rand.New(rand.NewSource(42))
	const dt = 8 * time.Millisecond
	// モータの追従。tau = 40 ms 相当。
	alpha := 1 - math.Exp(-float64(dt)/float64(40*time.Millisecond))

	patterns := [][3]float64{
		{1.2, 0, 0}, {-1.2, 0, 0},
		{0, 1.0, 0}, {0, -1.0, 0},
		{0, 0, 4.0}, {0, 0, -4.0},
		{0.8, -0.6, 2.0},
	}

	var now localization.Stamp
	var actual [3]float64
	for _, p := range patterns {
		// 1 パターンあたり 1 秒 = 125 周期。最初の 150 ms は整定待ちで捨てられる。
		for i := 0; i < 125; i++ {
			now += localization.Stamp(dt)

			// 指令は SPI フレーム上の量子化を通す。
			cmd := loclog.CommandRecord{
				TransferNs:  int64(now),
				VelXMmS:     int16(p[0] * 1000),
				VelYMmS:     int16(p[1] * 1000),
				VelAngMradS: int16(p[2] * 1000),
			}
			target := [3]float64{
				float64(cmd.VelXMmS) / 1000,
				float64(cmd.VelYMmS) / 1000,
				float64(cmd.VelAngMradS) / 1000,
			}
			for j := range actual {
				actual[j] += alpha * (target[j] - actual[j])
			}

			logical := k.WheelFromBody(actual[0], actual[1], actual[2])
			var slots [localization.NumWheels]float64
			var raw [localization.NumWheels]int64
			for slot := 0; slot < localization.NumWheels; slot++ {
				v := logical[g.WheelSlotOrder[slot]] + rng.NormFloat64()*0.01
				// エンコーダの量子化 (raw / 100 rad/s)。
				raw[slot] = int64(math.Round(v * 100))
				slots[slot] = float64(raw[slot]) / 100
			}

			w.LogCommand(now, cmd)
			w.LogWheel(now, loclog.WheelRecord{
				SampleNs:    int64(now) - int64(4*time.Millisecond),
				TransferNs:  int64(now),
				WheelFLRadS: slots[0], WheelBLRadS: slots[1],
				WheelBRRadS: slots[2], WheelFRRadS: slots[3],
				WheelFLRaw: raw[0], WheelBLRaw: raw[1],
				WheelBRRaw: raw[2], WheelFRRaw: raw[3],
			})
		}
	}
	if err := w.Close(); err != nil {
		t.Fatalf("close writer: %v", err)
	}
}

// 整定窓が実際に立ち上がりを落としていること。
func TestPairDropsUnsettledSamples(t *testing.T) {
	const dt = int64(8 * time.Millisecond)
	var wheels []loclog.WheelRecord
	var commands []loclog.CommandRecord
	for i := int64(0); i < 100; i++ {
		ts := i * dt
		vel := int16(1000)
		if i >= 50 {
			vel = -1000 // 途中で指令が変わる
		}
		commands = append(commands, loclog.CommandRecord{TransferNs: ts, VelXMmS: vel})
		wheels = append(wheels, loclog.WheelRecord{TransferNs: ts})
	}

	settle := 150 * time.Millisecond // 約 19 周期
	got, stats := pair(wheels, commands, settle)

	// 先頭と、指令が変わった直後の 2 か所で捨てられる。
	if stats.settling < 30 || stats.settling > 45 {
		t.Errorf("settling drops = %d, want roughly 2 x 19", stats.settling)
	}
	if len(got)+stats.settling != len(wheels) {
		t.Errorf("%d kept + %d settling != %d wheels", len(got), stats.settling, len(wheels))
	}
	for _, s := range got {
		if s.VX != 1.0 && s.VX != -1.0 {
			t.Errorf("unexpected vx %v; the mm/s -> m/s conversion is wrong", s.VX)
		}
	}
}

func TestPairCountsUnmatched(t *testing.T) {
	wheels := []loclog.WheelRecord{{TransferNs: 0}, {TransferNs: 999}}
	commands := []loclog.CommandRecord{{TransferNs: 0, VelXMmS: 500}}
	got, stats := pair(wheels, commands, 0)
	if stats.unmatched != 1 {
		t.Errorf("unmatched = %d, want 1", stats.unmatched)
	}
	if len(got) != 1 {
		t.Errorf("kept %d samples, want 1", len(got))
	}
}

func TestRunRejectsVisionReference(t *testing.T) {
	err := run("x.mcap", "", "vision", 0, 200, false)
	if err == nil {
		t.Fatal("expected -ref vision to be rejected until timesync lands")
	}
}

func TestRunRejectsUnknownReference(t *testing.T) {
	if err := run("x.mcap", "", "encoder", 0, 200, false); err == nil {
		t.Error("expected an error for an unknown reference")
	}
}
