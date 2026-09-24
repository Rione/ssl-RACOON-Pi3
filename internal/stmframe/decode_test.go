package stmframe

import (
	"errors"
	"fmt"
	"math"
	"math/rand"
	"strings"
	"testing"
)

func mustDecoder(t *testing.T, name string) *Decoder {
	t.Helper()
	p, err := Builtin(name)
	if err != nil {
		t.Fatalf("Builtin(%s): %v", name, err)
	}
	d, err := NewDecoder(p)
	if err != nil {
		t.Fatalf("NewDecoder(%s): %v", name, err)
	}
	return d
}

// makeV1Frame は現行フレーム (rock5a-v1) を組み立てる。
func makeV1Frame(volt, sensor, cap byte, wheels [4]int16) []byte {
	f := make([]byte, 20)
	f[0] = 0xFF
	f[1], f[2], f[3] = volt, sensor, cap
	for i, w := range wheels {
		f[4+2*i] = byte(uint16(w) & 0xFF)
		f[5+2*i] = byte(uint16(w) >> 8)
	}
	f[19] = 0xAA
	return f
}

func TestBuiltinProfilesAreValid(t *testing.T) {
	names := BuiltinNames()
	if len(names) == 0 {
		t.Fatal("no builtin profiles are embedded")
	}
	for _, name := range names {
		p, err := Builtin(name)
		if err != nil {
			t.Errorf("Builtin(%s): %v", name, err)
			continue
		}
		if _, err := NewDecoder(p); err != nil {
			t.Errorf("NewDecoder(%s): %v", name, err)
		}
	}
}

// --- 既存実装との等価性 ---------------------------------------------------
//
// internal/rock5a は //go:build rock5a が付いていて開発 PC では読めないので、
// 既存のバイト配置をここに参照実装として写し、プロファイルと突き合わせる。
// これが崩れたらフレーム解釈が静かに変わったということ。

// legacyParse は internal/rock5a/spi.go の parseRecvBufAt と同じ計算。
func legacyParse(rx []byte, frameOffset int) (volt, sensor, cap byte, wheels [4]int16) {
	off := frameOffset + 1
	volt, sensor, cap = rx[off+0], rx[off+1], rx[off+2]
	wheels[0] = int16(rx[off+3]) | int16(rx[off+4])<<8
	wheels[1] = int16(rx[off+5]) | int16(rx[off+6])<<8
	wheels[2] = int16(rx[off+7]) | int16(rx[off+8])<<8
	wheels[3] = int16(rx[off+9]) | int16(rx[off+10])<<8
	return
}

// legacyValidate は internal/rock5a/frame.go の validateSPIFrameAt と同じ判定。
func legacyValidate(rx []byte, offset int) error {
	const frameSize, recvSize = 20, 11
	if offset < 0 || offset+frameSize > len(rx) {
		return fmt.Errorf("frame out of range at offset %d", offset)
	}
	if rx[offset] != 0xFF {
		return errors.New("header")
	}
	if rx[offset+frameSize-1] != 0xAA {
		return errors.New("footer")
	}
	for i := offset + 1 + recvSize; i < offset+frameSize-1; i++ {
		if rx[i] != 0 {
			return errors.New("padding")
		}
	}
	return nil
}

// legacyFind は internal/rock5a/frame.go の findSPIFrame と同じ探索。
func legacyFind(buf []byte) int {
	last := -1
	for i := 0; i+20 <= len(buf); i++ {
		if legacyValidate(buf, i) == nil {
			last = i
		}
	}
	return last
}

func TestV1MatchesLegacyDecode(t *testing.T) {
	d := mustDecoder(t, "rock5a-v1")
	v := d.NewValues()
	idx := [4]int{}
	for slot, name := range WheelSlotNames {
		i, ok := d.FieldIndex(name)
		if !ok {
			t.Fatalf("profile is missing %s", name)
		}
		idx[slot] = i
	}

	rng := rand.New(rand.NewSource(1))
	for n := 0; n < 2000; n++ {
		wheels := [4]int16{
			int16(rng.Intn(65536) - 32768),
			int16(rng.Intn(65536) - 32768),
			int16(rng.Intn(65536) - 32768),
			int16(rng.Intn(65536) - 32768),
		}
		volt, sensor, capv := byte(rng.Intn(256)), byte(rng.Intn(256)), byte(rng.Intn(256))
		frame := makeV1Frame(volt, sensor, capv, wheels)

		if err := d.Validate(frame, 0); err != nil {
			t.Fatalf("Validate rejected a well-formed frame: %v", err)
		}
		if err := d.Decode(frame, 0, v); err != nil {
			t.Fatalf("Decode: %v", err)
		}

		lv, ls, lc, lw := legacyParse(frame, 0)
		for slot := range wheels {
			// 生値が一致すること (プロファイルは scale 0.01 を掛けている)。
			if got := v.RawAt(idx[slot]); got != int64(lw[slot]) {
				t.Fatalf("wheel slot %d: raw %d, legacy %d", slot, got, lw[slot])
			}
			want := float64(lw[slot]) / 100
			if got := v.At(idx[slot]); math.Abs(got-want) > 1e-12 {
				t.Fatalf("wheel slot %d: %v, want %v", slot, got, want)
			}
		}
		bi, _ := d.FieldIndex(FieldBattery)
		if got, want := v.At(bi), float64(lv)*0.1; math.Abs(got-want) > 1e-12 {
			t.Fatalf("battery: %v, want %v", got, want)
		}
		si, _ := d.FieldIndex(FieldSensorInfo)
		if got := v.RawAt(si); got != int64(ls) {
			t.Fatalf("sensorInfo: %v, want %v", got, ls)
		}
		ci, _ := d.FieldIndex(FieldCapPower)
		if got := v.RawAt(ci); got != int64(lc) {
			t.Fatalf("capPower: %v, want %v", got, lc)
		}
	}
}

// 窓の走査が既存の findSPIFrame と完全に同じ位置を選ぶこと。
func TestV1FindMatchesLegacy(t *testing.T) {
	d := mustDecoder(t, "rock5a-v1")
	rng := rand.New(rand.NewSource(7))

	for n := 0; n < 5000; n++ {
		window := make([]byte, 40)
		switch n % 4 {
		case 0: // 完全にランダム。ほぼ常にフレーム無し。
			rng.Read(window)
		case 1: // 後半にきれいなフレーム。
			rng.Read(window)
			copy(window[20:], makeV1Frame(140, 1, 9, [4]int16{1, -2, 3, -4}))
		case 2: // 位相がずれた位置にフレーム。
			rng.Read(window)
			copy(window[7:], makeV1Frame(141, 2, 8, [4]int16{5, 6, 7, 8}))
		case 3: // 窓に 2 フレーム。
			copy(window[0:], makeV1Frame(139, 0, 7, [4]int16{9, 9, 9, 9}))
			copy(window[20:], makeV1Frame(140, 0, 7, [4]int16{1, 1, 1, 1}))
		}

		want := legacyFind(window)
		m, err := d.Find(window)
		if want < 0 {
			if err == nil {
				t.Fatalf("case %d: Find returned offset %d, legacy found none", n%4, m.Offset)
			}
			if !errors.Is(err, ErrNoFrame) {
				t.Fatalf("case %d: unexpected error %v", n%4, err)
			}
			continue
		}
		if err != nil {
			t.Fatalf("case %d: Find failed (%v) but legacy found offset %d", n%4, err, want)
		}
		if m.Offset != want {
			t.Fatalf("case %d: Find offset %d, legacy %d", n%4, m.Offset, want)
		}
	}
}

// 窓に 2 フレーム入ったことを検出できること。どちらの SPI トランザクションの
// 応答か決まらず、時刻が 1 周期 (8 ms) ずれ得るので、計測時に必ず見る必要がある。
func TestFindReportsAmbiguity(t *testing.T) {
	d := mustDecoder(t, "rock5a-v1")
	window := make([]byte, 40)
	copy(window[0:], makeV1Frame(139, 0, 7, [4]int16{9, 9, 9, 9}))
	copy(window[20:], makeV1Frame(140, 0, 7, [4]int16{1, 1, 1, 1}))

	m, err := d.Find(window)
	if err != nil {
		t.Fatalf("Find: %v", err)
	}
	if m.Count != 2 || !m.Ambiguous() {
		t.Errorf("Match = %+v, want Count 2 and Ambiguous", m)
	}
	if m.Offset != 20 {
		t.Errorf("Find chose offset %d, want the newest frame at 20", m.Offset)
	}

	// 1 フレームだけなら曖昧ではない。
	single := make([]byte, 40)
	copy(single[20:], makeV1Frame(140, 0, 7, [4]int16{1, 1, 1, 1}))
	m, err = d.Find(single)
	if err != nil {
		t.Fatalf("Find: %v", err)
	}
	if m.Ambiguous() {
		t.Errorf("Match = %+v, want unambiguous", m)
	}
}

func TestV1RejectsNonZeroPadding(t *testing.T) {
	d := mustDecoder(t, "rock5a-v1")
	frame := makeV1Frame(140, 0, 0, [4]int16{0, 0, 0, 0})
	frame[15] = 0x01
	if err := d.Validate(frame, 0); err == nil {
		t.Error("Validate accepted a frame with non-zero padding")
	}
}

// IMU プロファイルはバイト 12-19 をすべて使うので、フレームの同期はヘッダとフッタだけが頼りになる。
// その事実をテストで可視化しておく (実機のファーム MainBoard_V26_2 がこの形。誤同期を避けるため、
// Pi 側は前回と同じ位置を優先して読む: internal/rock5a/spi.go)。
func TestImuProfileLosesSyncStrength(t *testing.T) {
	v1 := mustDecoder(t, "rock5a-v1").Profile()
	v2 := mustDecoder(t, "rock5a-v2-imu").Profile()
	if v1.SyncStrength() != 9 {
		t.Errorf("rock5a-v1 sync strength = %d, want 9 (header + footer + 7 padding)", v1.SyncStrength())
	}
	if v2.SyncStrength() != 2 {
		t.Errorf("rock5a-v2-imu sync strength = %d, want 2 (header + footer only)", v2.SyncStrength())
	}
	if v2.SyncStrength() >= v1.SyncStrength() {
		t.Error("expected the IMU profile to have weaker frame sync than the current one")
	}
}

func TestImuProfileScales(t *testing.T) {
	d := mustDecoder(t, "rock5a-v2-imu")
	b, err := d.Bind()
	if err != nil {
		t.Fatalf("Bind: %v", err)
	}
	if !b.HasGyro() || !b.HasAccel() {
		t.Fatal("IMU profile should expose gyro and accel")
	}

	// 実機のファーム (ssl-Circuit MainBoard_V26_2 src/unit/robot.c) の並び:
	// 21 バイト、12 から 加速度 X [1 mg/LSB]・加速度 Y・ヨーの角速度 [900 LSB = 1 rad/s]・姿勢角。
	frame := make([]byte, 21)
	frame[0], frame[20] = 0xFF, 0xAA
	put16(frame, 12, 1000)  // 1 g
	put16(frame, 14, -1000) // -1 g
	put16(frame, 16, 900)   // 1 rad/s

	v := d.NewValues()
	if err := d.Decode(frame, 0, v); err != nil {
		t.Fatalf("Decode: %v", err)
	}
	if got, want := v.At(b.GyroZ), 1.0; math.Abs(got-want) > 1e-6 {
		t.Errorf("gyroZ = %v rad/s, want %v", got, want)
	}
	if got, want := v.At(b.AccelX), 9.80665; math.Abs(got-want) > 1e-3 {
		t.Errorf("accelX = %v m/s^2, want %v (1 g)", got, want)
	}
	if got, want := v.At(b.AccelY), -9.80665; math.Abs(got-want) > 1e-3 {
		t.Errorf("accelY = %v m/s^2, want %v (-1 g)", got, want)
	}
}

func put16(b []byte, off int, v int16) {
	b[off] = byte(uint16(v) & 0xFF)
	b[off+1] = byte(uint16(v) >> 8)
}

// 現行プロファイルには IMU が無い。上位はそれを「非搭載」として扱えること。
func TestV1HasNoImu(t *testing.T) {
	d := mustDecoder(t, "rock5a-v1")
	b, err := d.Bind()
	if err != nil {
		t.Fatalf("Bind: %v", err)
	}
	if b.HasGyro() || b.HasAccel() {
		t.Error("rock5a-v1 must not expose an IMU; the STM firmware does not send one")
	}
	if !b.HasWheels() {
		t.Error("rock5a-v1 must expose all four wheels")
	}
}

func TestProfileValidationRejectsBadDefinitions(t *testing.T) {
	cases := []struct{ name, json, want string }{
		{"no name", `{"frameSize":20,"fields":[{"name":"a","offset":0,"type":"u8","scale":1}]}`, "name is empty"},
		{"bad type", `{"profile":"p","frameSize":20,"fields":[{"name":"a","offset":0,"type":"u24","scale":1}]}`, "unknown type"},
		{"overrun", `{"profile":"p","frameSize":4,"fields":[{"name":"a","offset":3,"type":"i16le","scale":1}]}`, "does not fit"},
		{"zero scale", `{"profile":"p","frameSize":20,"fields":[{"name":"a","offset":0,"type":"u8","scale":0}]}`, "scale"},
		{"dup name", `{"profile":"p","frameSize":20,"fields":[{"name":"a","offset":0,"type":"u8","scale":1},{"name":"a","offset":1,"type":"u8","scale":1}]}`, "duplicate"},
		{"no fields", `{"profile":"p","frameSize":20,"fields":[]}`, "no fields"},
		{"reserved overrun", `{"profile":"p","frameSize":20,"fields":[{"name":"a","offset":0,"type":"u8","scale":1}],"reserved":[{"offset":19,"length":4}]}`, "does not fit"},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			_, err := LoadProfile(strings.NewReader(c.json))
			if err == nil {
				t.Fatalf("expected an error mentioning %q", c.want)
			}
			if !strings.Contains(err.Error(), c.want) {
				t.Errorf("error %q does not mention %q", err, c.want)
			}
		})
	}
}

// 4 輪で timeOffsetMs が食い違うプロファイルは、1 サンプルとして扱えないので弾く。
func TestBindRejectsMismatchedWheelTimeOffsets(t *testing.T) {
	p, err := LoadProfile(strings.NewReader(`{
	  "profile": "bad", "frameSize": 20, "header": "0xFF", "footer": "0xAA",
	  "fields": [
	    {"name":"wheelFL","offset":4,"type":"i16le","scale":0.01,"timeOffsetMs":-4},
	    {"name":"wheelBL","offset":6,"type":"i16le","scale":0.01,"timeOffsetMs":-4},
	    {"name":"wheelBR","offset":8,"type":"i16le","scale":0.01,"timeOffsetMs":-4},
	    {"name":"wheelFR","offset":10,"type":"i16le","scale":0.01,"timeOffsetMs":-1}
	  ]}`))
	if err != nil {
		t.Fatalf("LoadProfile: %v", err)
	}
	d, err := NewDecoder(p)
	if err != nil {
		t.Fatalf("NewDecoder: %v", err)
	}
	if _, err := d.Bind(); err == nil {
		t.Error("Bind accepted wheels with different timeOffsetMs")
	}
}

func TestDecodeDoesNotAllocate(t *testing.T) {
	d := mustDecoder(t, "rock5a-v1")
	v := d.NewValues()
	window := make([]byte, 40)
	copy(window[20:], makeV1Frame(140, 1, 9, [4]int16{100, -200, 300, -400}))

	got := testing.AllocsPerRun(1000, func() {
		m, err := d.Find(window)
		if err != nil {
			t.Fatal(err)
		}
		if err := d.Decode(window, m.Offset, v); err != nil {
			t.Fatal(err)
		}
	})
	if got != 0 {
		t.Errorf("Find+Decode allocated %v times per run, want 0", got)
	}
}

func TestByteJSONRoundTrip(t *testing.T) {
	p, err := Builtin("rock5a-v1")
	if err != nil {
		t.Fatal(err)
	}
	if p.Header == nil || *p.Header != 0xFF {
		t.Errorf("header = %v, want 0xFF", p.Header)
	}
	if p.Footer == nil || *p.Footer != 0xAA {
		t.Errorf("footer = %v, want 0xAA", p.Footer)
	}
}

// Validate (理由付き) と validAt (高速版) は常に同じ判定でなければならない。
// 片方だけ直して静かに食い違うのを防ぐ。
func TestValidateAndFastPathAgree(t *testing.T) {
	rng := rand.New(rand.NewSource(99))
	for _, name := range BuiltinNames() {
		d := mustDecoder(t, name)
		buf := make([]byte, d.FrameSize()*2)
		for n := 0; n < 20000; n++ {
			switch n % 3 {
			case 0:
				rng.Read(buf)
			case 1:
				rng.Read(buf)
				// ヘッダとフッタだけ合わせて、予約領域の判定を効かせる。
				buf[0], buf[d.FrameSize()-1] = 0xFF, 0xAA
			case 2:
				for i := range buf {
					buf[i] = 0
				}
				buf[0], buf[d.FrameSize()-1] = 0xFF, 0xAA
				rng.Read(buf[4:12])
			}
			for off := -1; off <= d.FrameSize()+1; off++ {
				slow := d.Validate(buf, off) == nil
				fast := d.validAt(buf, off)
				if slow != fast {
					t.Fatalf("%s offset %d: Validate=%v validAt=%v (% x)", name, off, slow, fast, buf)
				}
			}
		}
	}
}
