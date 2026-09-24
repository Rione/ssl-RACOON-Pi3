package localization

import (
	"math"
	"os"
	"path/filepath"
	"testing"
)

// リポジトリに置いてある設定ファイルが読めること。
//
// **`_comment` が入っていても読めること**を含む。LoadGeometry は
// DisallowUnknownFields なので、出どころを書き残す欄が無いと
// 設定ファイルにコメントを入れられない。
func TestShippedGeometryConfigsLoad(t *testing.T) {
	dir := filepath.Join("..", "..", "config")
	entries, err := os.ReadDir(dir)
	if err != nil {
		t.Skipf("no config directory: %v", err)
	}
	var found int
	for _, e := range entries {
		if e.IsDir() || filepath.Ext(e.Name()) != ".json" {
			continue
		}
		path := filepath.Join(dir, e.Name())
		g, err := LoadGeometryFile(path)
		if err != nil {
			t.Errorf("%s: %v", e.Name(), err)
			continue
		}
		if len(g.Comment) == 0 {
			t.Errorf("%s: no _comment; write down where the numbers came from", e.Name())
		}
		if _, err := NewKinematics(g); err != nil {
			t.Errorf("%s: kinematics: %v", e.Name(), err)
		}
		found++
	}
	if found == 0 {
		t.Skip("no geometry config files")
	}
}

// CAD の設計値が 3D モデルの解析結果と一致していること。
//
// 数字が静かに書き換わるのを防ぐ (研究 §2.2 の表がこれに依存している)。
func TestCADGeometryMatchesTheModel(t *testing.T) {
	g := CADGeometry()
	want := [NumWheels]float64{60, 135, -135, -60}
	for i := range want {
		if g.WheelAnglesDeg[i] != want[i] {
			t.Fatalf("wheel angles = %v, want %v (Robot_V2.step gives exact values)", g.WheelAnglesDeg, want)
		}
	}
	if math.Abs(g.MomentArmM-0.07845) > 1e-9 {
		t.Fatalf("moment arm = %v m, want 0.07845 (contact radius from the model)", g.MomentArmM)
	}
	// 車輪半径は CAD からは決まらない。ハブ外径 26.705 mm とローラ列の
	// 弧長から出る上限 約 29.5 mm のあいだにあること。
	for i, r := range g.WheelRadiusM {
		if r <= 0.026705 || r > 0.0295 {
			t.Fatalf("wheel radius[%d] = %v m is outside the window the model allows", i, r)
		}
	}
}
