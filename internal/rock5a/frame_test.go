//go:build rock5a

package rock5a

import (
	"math"
	"testing"

	"github.com/Rione/ssl-RACOON-Pi3/internal/state"
)

// buildFrame は試験用に 1 フレーム作る (payload はヘッダの後ろに置く中身)。
func buildFrame(l spiLayout, payload []byte) []byte {
	f := make([]byte, l.FrameSize)
	f[0] = SPIFrameHeader
	copy(f[1:], payload)
	f[l.FrameSize-1] = SPIFrameFooter
	return f
}

func TestWrapSPIFrameLengths(t *testing.T) {
	for _, l := range []spiLayout{spiLayoutV1, spiLayoutV2} {
		f := wrapSPIFrame(l, make([]byte, 18))
		if len(f) != l.FrameSize {
			t.Errorf("%s: frame is %d bytes, want %d", l.Name, len(f), l.FrameSize)
		}
		if f[0] != SPIFrameHeader || f[len(f)-1] != SPIFrameFooter {
			t.Errorf("%s: header/footer missing: % x", l.Name, f)
		}
	}
}

func TestValidateSPIFrame(t *testing.T) {
	// v1: 車輪より後ろは 0 でなければならない
	v1 := buildFrame(spiLayoutV1, make([]byte, spiLayoutV1.PayloadSize))
	if err := validateSPIFrame(spiLayoutV1, v1); err != nil {
		t.Fatalf("v1 frame must be valid: %v", err)
	}
	bad := append([]byte(nil), v1...)
	bad[13] = 0x7F
	if err := validateSPIFrame(spiLayoutV1, bad); err == nil {
		t.Error("v1 must reject a non-zero byte after the wheels")
	}
	// v2: 同じ場所に IMU が入るので、0 でなくてよい
	p := make([]byte, spiLayoutV2.PayloadSize)
	for i := 11; i < len(p); i++ {
		p[i] = 0x7F
	}
	v2 := buildFrame(spiLayoutV2, p)
	if err := validateSPIFrame(spiLayoutV2, v2); err != nil {
		t.Fatalf("v2 frame with IMU bytes must be valid: %v", err)
	}
	// 長さが違えば、もう一方の形では読めない (これが自動判別の根拠)
	if err := validateSPIFrame(spiLayoutV1, v2); err == nil {
		t.Error("a 21-byte frame must not validate as v1")
	}
}

func TestFindSPIFrameInWindow(t *testing.T) {
	l := spiLayoutV2
	win := make([]byte, l.FrameSize*2)
	f := buildFrame(l, make([]byte, l.PayloadSize))
	copy(win[l.FrameSize:], f)
	if got := findSPIFrame(l, win, -1); got != l.FrameSize {
		t.Errorf("found at %d, want %d", got, l.FrameSize)
	}
	if got := findSPIFrame(l, make([]byte, l.FrameSize*2), -1); got != -1 {
		t.Errorf("empty window must not contain a frame, got %d", got)
	}
	// 前回と同じ位置が有効なら、そちらを優先する (IMU 入りは 0 埋めが無く誤同期しやすい)
	copy(win[:l.FrameSize], f)
	if got := findSPIFrame(l, win, 0); got != 0 {
		t.Errorf("preferred offset ignored: got %d, want 0", got)
	}
	if got := findSPIFrame(l, win, 3); got != l.FrameSize {
		t.Errorf("invalid preferred offset must fall back to the search: got %d", got)
	}
}

func TestParseRecvIMU(t *testing.T) {
	l := spiLayoutV2
	p := make([]byte, l.PayloadSize)
	p[0] = 240 // 電圧 24.0 V
	put := func(i int, v int16) { p[i] = byte(uint16(v) & 0xff); p[i+1] = byte(uint16(v) >> 8) }
	put(3, 100)    // FL 1.00 rad/s
	put(11, 250)   // 加速度 X 250 mg
	put(13, -1000) // 加速度 Y -1000 mg = -1 g
	put(15, 900)   // ヨーの角速度 1.0 rad/s
	put(17, 15708) // 姿勢角 1.5708 rad
	d := parseRecvBufAt(l, buildFrame(l, p), 0)
	if !d.HasIMU || d.Volt != 240 || d.FlWheelSpeed != 100 {
		t.Fatalf("unexpected decode: %+v", d)
	}
	applyImu(d)
	if !state.ImuValid {
		t.Fatal("ImuValid must be true")
	}
	// state に入るのは機体の向きに直した後の値 (前が +x、左が +y)。
	// IMU は 90 度回して付いているので、前後 = IMU の Y、左右 = -IMU の X。
	for _, c := range []struct {
		name      string
		got, want float64
	}{
		{"前後 (= IMU の Y)", state.ImuAccelXMS2, -9.80665},
		{"左右 (= -IMU の X)", state.ImuAccelYMS2, -0.25 * 9.80665},
		{"yawRate", state.ImuYawRateRadS, 1.0},
		{"yaw", state.ImuYawRad, 1.5708},
	} {
		if math.Abs(c.got-c.want) > 1e-6 {
			t.Errorf("%s = %g, want %g", c.name, c.got, c.want)
		}
	}
	// IMU の無いファームでは触らない
	d1 := parseRecvBufAt(spiLayoutV1, buildFrame(spiLayoutV1, make([]byte, spiLayoutV1.PayloadSize)), 0)
	applyImu(d1)
	if state.ImuValid {
		t.Error("ImuValid must be false for a v1 frame")
	}
}

func TestLayoutProbeSwitchesAndSettles(t *testing.T) {
	curLayout, layoutMisses, layoutSettled, layoutAnnounce = spiLayoutV1, 0, false, false
	for i := 0; i < spiLayoutProbeCycles-1; i++ {
		updateSPILayout(false)
	}
	if curLayout.FrameSize != spiLayoutV1.FrameSize {
		t.Fatal("must not switch before the probe window is over")
	}
	updateSPILayout(false)
	if curLayout.FrameSize != spiLayoutV2.FrameSize {
		t.Fatalf("must switch to v2, got %s", curLayout.Name)
	}
	updateSPILayout(true)
	if !layoutSettled {
		t.Error("a valid frame must settle the layout")
	}
	for i := 0; i < spiLayoutProbeCycles*2; i++ {
		updateSPILayout(true)
	}
	if curLayout.FrameSize != spiLayoutV2.FrameSize {
		t.Error("must stay on v2 while frames are valid")
	}
}

// 下り (Pi → STM) のフレームの形。SPI_PROTOCOL.md §4: 中身は 18 バイトのままで、
// v2 では 19 バイト目が予備 (0 を送る)。長さは上りと揃える。
func TestSendFrameMatchesTheProtocol(t *testing.T) {
	payload := make([]byte, 18)
	for i := range payload {
		payload[i] = byte(i + 1)
	}
	for _, c := range []struct {
		l    spiLayout
		size int
	}{{spiLayoutV1, 20}, {spiLayoutV2, 21}} {
		f := wrapSPIFrame(c.l, payload)
		if len(f) != c.size {
			t.Errorf("%s: %d bytes, want %d", c.l.Name, len(f), c.size)
			continue
		}
		if f[0] != SPIFrameHeader || f[c.size-1] != SPIFrameFooter {
			t.Errorf("%s: header/footer: % x", c.l.Name, f)
		}
		// status (中身の 18 バイト目) はフレーム位置 18 に来る
		if f[18] != payload[17] {
			t.Errorf("%s: status byte landed at the wrong offset: % x", c.l.Name, f)
		}
		if c.l.HasIMU && f[19] != 0 {
			t.Errorf("v2: frame byte 19 must stay reserved (0), got %02x", f[19])
		}
	}
}

// 電圧の倍率。SPI_PROTOCOL.md: 旧 0.1 V/LSB (×10、26 V 以上で一周する不具合)、
// 修正版 0.2 V/LSB (×5)。21 バイトのフレームには倍率を直す前の中間の版もあるので、
// 40 V を超える読みは中間の版として 0.1 V/LSB で読み直す。
func TestBatteryVoltsScale(t *testing.T) {
	for _, c := range []struct {
		name string
		l    spiLayout
		raw  uint8
		want float64
	}{
		{"v1 24.0V", spiLayoutV1, 240, 24.0},
		{"v2 24.0V (0.2V/LSB)", spiLayoutV2, 120, 24.0},
		{"v2 25.4V", spiLayoutV2, 127, 25.4},
		{"v2 でも古い倍率のファーム (raw 240 は 48V ではなく 24V)", spiLayoutV2, 240, 24.0},
		{"v2 空 (0)", spiLayoutV2, 0, 0},
	} {
		if got := batteryVolts(c.l, c.raw); math.Abs(got-c.want) > 1e-9 {
			t.Errorf("%s: raw %d -> %.2f V, want %.2f V", c.name, c.raw, got, c.want)
		}
	}
}

// IMU の取り付け。実機で確かめた向き (config.go の bodyFromImuAccel のコメント) を固定する。
func TestImuMountingRotation(t *testing.T) {
	// 機体の左側を持ち上げた実測: IMU の X が +5.4、Y はほぼ 0。
	// 機体の約束では「左が +y」なので、左を上げたら左方向の成分は負になる。
	if fwd, left := bodyFromImuAccel(5.4, 0.1); math.Abs(fwd-0.1) > 1e-9 || math.Abs(left+5.4) > 1e-9 {
		t.Errorf("left side up: forward %.2f left %.2f, want ~0 and -5.4", fwd, left)
	}
	// 前 (ドリブラ側) を持ち上げた実測: IMU の Y が -4.4、X はほぼ 0 -> 前方向の成分が負
	if fwd, left := bodyFromImuAccel(-0.1, -4.4); math.Abs(fwd+4.4) > 1e-9 || math.Abs(left-0.1) > 1e-9 {
		t.Errorf("nose up: forward %.2f left %.2f, want -4.4 and ~0", fwd, left)
	}
}
