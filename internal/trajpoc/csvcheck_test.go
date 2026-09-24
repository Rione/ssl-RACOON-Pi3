package trajpoc

import (
	"bytes"
	"strings"
	"testing"
)

// CSV の見出しと値の列数が合っていること。
//
// 見出しと Fprintf の書式を別々に直すので、片方だけ足すと**黙ってずれる。**
// 記録は移植の正解表になるので、ずれたまま貯まると後から解釈できない。
func TestCSVHeaderMatchesRow(t *testing.T) {
	var b bytes.Buffer
	if err := WriteCSV(&b, []Sample{{T: 1, BatteryV: 24.5, ImuValid: true}}); err != nil {
		t.Fatal(err)
	}
	lines := strings.Split(strings.TrimSpace(b.String()), "\n")
	if len(lines) < 2 {
		t.Fatal("行が足りない")
	}
	h := strings.Split(lines[0], ",")
	r := strings.Split(lines[1], ",")
	if len(h) != len(r) {
		t.Fatalf("見出し %d 列に対して値が %d 列", len(h), len(r))
	}
	i := -1
	for k, name := range h {
		if name == "battery_v" {
			i = k
		}
	}
	if i < 0 {
		t.Fatal("battery_v の列が無い")
	}
	if r[i] != "24.50" {
		t.Errorf("battery_v = %q, 24.50 のはず", r[i])
	}
	t.Logf("列数 %d, battery_v は %d 列目 = %s", len(h), i, r[i])
}
