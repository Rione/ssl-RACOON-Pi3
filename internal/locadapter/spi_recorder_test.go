package locadapter

import (
	"math"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/loclog"
)

func newTestSPIRecorder(t *testing.T, profile string) (*SPIRecorder, *loclog.Clock) {
	t.Helper()
	clock := loclog.NewClock()
	r, err := NewSPIRecorder(clock, nil, profile)
	if err != nil {
		t.Fatalf("NewSPIRecorder: %v", err)
	}
	return r, clock
}

func rxFrame(volt, sensor, capv byte, wheels [4]int16) []byte {
	f := make([]byte, 20)
	f[0], f[19] = 0xFF, 0xAA
	f[1], f[2], f[3] = volt, sensor, capv
	for i, w := range wheels {
		f[4+2*i] = byte(uint16(w) & 0xFF)
		f[5+2*i] = byte(uint16(w) >> 8)
	}
	return f
}

func txFrame(velX, velY, velAng int16, info byte) []byte {
	f := make([]byte, 20)
	f[0], f[19] = 0xFF, 0xAA
	put := func(off int, v int16) {
		f[off] = byte(uint16(v) & 0xFF)
		f[off+1] = byte(uint16(v) >> 8)
	}
	put(1, velX)   // payload[0..1]
	put(3, velY)   // payload[2..3]
	put(5, velAng) // payload[4..5]
	f[18] = info   // payload[17]
	return f
}

func TestSPIRecorderDecodesWheels(t *testing.T) {
	r, _ := newTestSPIRecorder(t, "rock5a-v1")
	now := time.Now()

	// 窓はフレーム 2 個ぶん。1 回目は窓の後半にだけフレームが入る。
	s := r.Record(txFrame(0, 0, 0, 0), rxFrame(140, 0, 9, [4]int16{100, -200, 300, -400}), now, now.Add(200*time.Microsecond))
	if !s.Valid {
		t.Fatal("first transaction produced no valid frame")
	}
	want := [4]float64{1.0, -2.0, 3.0, -4.0} // raw/100 = rad/s
	for i := range want {
		if math.Abs(s.Wheel.Omega[i]-want[i]) > 1e-12 {
			t.Errorf("wheel[%d] = %v rad/s, want %v", i, s.Wheel.Omega[i], want[i])
		}
	}
	wantRaw := [4]int64{100, -200, 300, -400}
	if s.WheelRaw != wantRaw {
		t.Errorf("raw = %v, want %v", s.WheelRaw, wantRaw)
	}
	// rock5a-v1 には IMU が無い。
	if s.IMU.HasGyro || s.IMU.HasAccel {
		t.Error("rock5a-v1 must not report an IMU")
	}
}

// 転送時刻は Tx 前後の中点。公称 8 ms ではなく実測の dt が出ること。
func TestSPIRecorderMeasuresRealDt(t *testing.T) {
	r, clock := newTestSPIRecorder(t, "rock5a-v1")
	base := time.Now()
	rx := rxFrame(140, 0, 9, [4]int16{0, 0, 0, 0})
	tx := txFrame(0, 0, 0, 0)

	s1 := r.Record(tx, rx, base, base.Add(400*time.Microsecond))
	if s1.Dt != 0 {
		t.Errorf("first dt = %v, want 0", s1.Dt)
	}
	if want := clock.StampOf(base.Add(200 * time.Microsecond)); s1.Transfer != want {
		t.Errorf("transfer = %v, want the midpoint %v", s1.Transfer, want)
	}

	// 2 回目は 9.3 ms 後。公称 8 ms ではなくこちらが出なければならない。
	next := base.Add(9300 * time.Microsecond)
	s2 := r.Record(tx, rx, next, next.Add(400*time.Microsecond))
	if s2.Dt != 9300*time.Microsecond {
		t.Errorf("dt = %v, want 9.3ms (the measured interval, not the nominal 8ms)", s2.Dt)
	}
}

func TestSPIRecorderDecodesCommand(t *testing.T) {
	r, _ := newTestSPIRecorder(t, "rock5a-v1")
	now := time.Now()
	s := r.Record(txFrame(1500, -250, 3000, 0b00100001),
		rxFrame(140, 0, 9, [4]int16{0, 0, 0, 0}), now, now.Add(time.Microsecond))

	c := s.Command
	if c.VelXMmS != 1500 || c.VelYMmS != -250 || c.VelAngMradS != 3000 {
		t.Errorf("command = %+v, want VelX 1500 VelY -250 VelAng 3000", c)
	}
	if c.Info != 0b00100001 {
		t.Errorf("info = %08b, want 00100001", c.Info)
	}
}

// 窓に 2 フレーム見えたことを伝えること。どの転送への応答か一意でなくなり、
// 時刻が 1 周期ずれ得るので、遅延を測る局面で必ず要る。
func TestSPIRecorderReportsAmbiguity(t *testing.T) {
	r, _ := newTestSPIRecorder(t, "rock5a-v1")
	now := time.Now()
	tx := txFrame(0, 0, 0, 0)
	rx := rxFrame(140, 0, 9, [4]int16{1, 1, 1, 1})

	s1 := r.Record(tx, rx, now, now.Add(time.Microsecond))
	if s1.Ambiguous {
		t.Error("the first transaction cannot be ambiguous; the window has only one frame")
	}
	// 2 回目は窓に前回ぶんも残るので 2 フレーム見える。
	s2 := r.Record(tx, rx, now.Add(8*time.Millisecond), now.Add(8*time.Millisecond+time.Microsecond))
	if !s2.Ambiguous {
		t.Error("expected the recorder to report two frames in the window")
	}
	if _, _, amb := r.Stats(); amb != 1 {
		t.Errorf("ambiguous count = %d, want 1", amb)
	}
}

func TestSPIRecorderHandlesBrokenFrames(t *testing.T) {
	r, _ := newTestSPIRecorder(t, "rock5a-v1")
	now := time.Now()
	broken := make([]byte, 20)
	for i := range broken {
		broken[i] = 0x5A
	}
	s := r.Record(txFrame(0, 0, 0, 0), broken, now, now.Add(time.Microsecond))
	if s.Valid {
		t.Error("a garbage frame was accepted")
	}
	if _, errs, _ := r.Stats(); errs != 1 {
		t.Errorf("frameErrors = %d, want 1", errs)
	}
	// 壊れたフレームでも転送時刻は取れている。時刻の連続性を切らさない。
	if s.Transfer == 0 {
		t.Error("transfer stamp should still be recorded for a broken frame")
	}
}

// 直前に有効フレームがあっても、古い窓の残骸を新しいフレームとして拾わないこと。
func TestSPIRecorderClearsStaleWindow(t *testing.T) {
	r, _ := newTestSPIRecorder(t, "rock5a-v1")
	now := time.Now()
	tx := txFrame(0, 0, 0, 0)

	r.Record(tx, rxFrame(140, 0, 9, [4]int16{7, 7, 7, 7}), now, now.Add(time.Microsecond))
	// 次は 12 バイトしか返って来なかった (短い読み) 状況。
	short := make([]byte, 12)
	s := r.Record(tx, short, now.Add(8*time.Millisecond), now.Add(8*time.Millisecond+time.Microsecond))
	// 窓の前半に残った前回のフレームは拾えるが、後半のゴミは拾わない。
	if s.Valid && s.Wheel.Omega[0] != 0.07 {
		t.Errorf("picked up a bogus frame: %v", s.Wheel.Omega)
	}
}

func TestSPIRecorderImuProfile(t *testing.T) {
	r, _ := newTestSPIRecorder(t, "rock5a-v2-imu")
	now := time.Now()

	// 実機のファーム (ssl-Circuit MainBoard_V26_2) の並び: 21 バイト、
	// 12 から 加速度 X [1 mg/LSB]・加速度 Y・ヨーの角速度 [900 LSB = 1 rad/s]・姿勢角。
	f := make([]byte, 21)
	f[0], f[20] = 0xFF, 0xAA
	f[1] = 140
	put := func(off int, v int16) {
		f[off] = byte(uint16(v) & 0xFF)
		f[off+1] = byte(uint16(v) >> 8)
	}
	put(12, 1000)  // 1 g
	put(14, -1000) // -1 g
	put(16, 900)   // 1 rad/s

	s := r.Record(txFrame(0, 0, 0, 0), f, now, now.Add(time.Microsecond))
	if !s.Valid {
		t.Fatal("IMU frame was rejected")
	}
	if !s.IMU.HasGyro || !s.IMU.HasAccel {
		t.Fatal("IMU profile did not produce IMU values")
	}
	if math.Abs(s.IMU.GyroZ-1.0) > 1e-6 {
		t.Errorf("gyroZ = %v rad/s, want 1.0", s.IMU.GyroZ)
	}
	if math.Abs(s.IMU.Accel.X-9.80665) > 1e-3 || math.Abs(s.IMU.Accel.Y+9.80665) > 1e-3 {
		t.Errorf("accel = %+v, want (9.80665, -9.80665)", s.IMU.Accel)
	}
	// IMU は車輪より後にサンプルされる想定なので、時刻がずれる。
	if s.IMU.Stamp == s.Wheel.Stamp {
		t.Error("expected the IMU and wheel samples to carry different timeOffsetMs")
	}
}

// 車輪サンプルの時刻は転送時刻より過去になる (timeOffsetMs は負)。
func TestSPIRecorderAppliesTimeOffset(t *testing.T) {
	r, _ := newTestSPIRecorder(t, "rock5a-v1")
	now := time.Now()
	s := r.Record(txFrame(0, 0, 0, 0), rxFrame(140, 0, 9, [4]int16{0, 0, 0, 0}), now, now.Add(time.Microsecond))
	if !s.Valid {
		t.Fatal("no valid frame")
	}
	if s.Wheel.Stamp >= s.Transfer {
		t.Errorf("wheel sample stamp %v is not before the transfer time %v", s.Wheel.Stamp, s.Transfer)
	}
	if got := s.Transfer.Sub(s.Wheel.Stamp); got != 4*time.Millisecond {
		t.Errorf("wheel timeOffset = %v, want 4ms (from the profile)", got)
	}
}

// 計画 §7.3: SPI のホットパスは 1 周期 0 アロケーション。
func TestSPIRecorderDoesNotAllocate(t *testing.T) {
	clock := loclog.NewClock()
	w, err := loclog.NewWriter(clock, loclog.Options{
		Path:           t.TempDir() + "/alloc.mcap",
		QueueSize:      1,
		StatusInterval: time.Hour,
	})
	if err != nil {
		t.Fatal(err)
	}
	defer w.Close()

	r, err := NewSPIRecorder(clock, w, "rock5a-v1")
	if err != nil {
		t.Fatal(err)
	}
	tx := txFrame(1000, 0, 0, 0)
	rx := rxFrame(140, 0, 9, [4]int16{100, -200, 300, -400})
	now := time.Now()

	got := testing.AllocsPerRun(1000, func() {
		r.Record(tx, rx, now, now.Add(200*time.Microsecond))
	})
	if got != 0 {
		t.Errorf("Record allocated %v times per run, want 0", got)
	}
}

func TestNewSPIRecorderRejectsUnknownProfile(t *testing.T) {
	if _, err := NewSPIRecorder(loclog.NewClock(), nil, "does-not-exist"); err == nil {
		t.Error("expected an error for an unknown profile")
	}
}
