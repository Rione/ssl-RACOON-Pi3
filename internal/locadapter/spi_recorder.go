package locadapter

import (
	"fmt"
	"sync/atomic"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi3/internal/loclog"
	"github.com/Rione/ssl-RACOON-Pi3/internal/stmframe"
)

// CommandLayout は下り (Rock5A -> STM) フレームのバイト配置。
//
// 既定値は internal/rock5a/board.go の RegisterLink と一致させてある。
// 指令と実測を突き合わせるために、送ったフレームも一緒に記録する必要がある
// (計画 §8 の符号・スケール同定)。
type CommandLayout struct {
	// FrameOffset は 20 バイトフレーム内でペイロードが始まる位置 (ヘッダの次)。
	FrameOffset int
	// 以下はペイロード先頭からのバイト位置。
	VelXLow, VelXHigh     int
	VelYLow, VelYHigh     int
	VelAngLow, VelAngHigh int
	Dribble, Kick, Chip   int
	Info                  int
}

// DefaultCommandLayout は現行の rock5a の配置を返す。
func DefaultCommandLayout() CommandLayout {
	return CommandLayout{
		FrameOffset: 1,
		VelXLow:     0, VelXHigh: 1,
		VelYLow: 2, VelYHigh: 3,
		VelAngLow: 4, VelAngHigh: 5,
		Dribble: 6, Kick: 7, Chip: 8,
		Info: 17,
	}
}

// Sample は 1 回の SPI トランザクションから取れたセンサ値。
type Sample struct {
	// Valid はフレームが取れたか。false なら他のフィールドは無意味。
	Valid bool
	// Transfer は転送時刻 (Tx 前後の中点)。
	Transfer localization.Stamp
	// Dt は前回の転送からの実測間隔。初回は 0。
	Dt time.Duration
	// Ambiguous は受信窓に複数のフレームが見えたか。
	// 真なら、この応答がどの転送に対するものか一意でない。
	Ambiguous bool

	// Wheel は SPI フレーム上の並び (FL, BL, BR, FR) の角速度。
	Wheel localization.WheelSample
	// WheelRaw はスケール適用前の生値。同定はこちらから行う。
	WheelRaw [localization.NumWheels]int64

	// IMU。プロファイルに定義が無ければ HasGyro / HasAccel は false。
	IMU localization.ImuSample

	// Command はこの転送で送った指令。
	Command loclog.CommandRecord
}

// SPIRecorder は SPI の生フレームを MCAP へ記録し、センサ値を取り出す。
//
// 既存の internal/rock5a の受信処理は変更しない。記録は観測に徹し、
// 記録側の不具合が走行機能を巻き込まないようにする。
//
// 受信窓は rock5a と独立に持つ。両者のフレーム探索が同じ結果を出すことは
// internal/stmframe の既存実装との等価性テストで担保している。
type SPIRecorder struct {
	clock  *loclog.Clock
	rec    *loclog.Writer
	dec    *stmframe.Decoder
	bind   *stmframe.Binding
	values *stmframe.Values
	layout CommandLayout

	window []byte

	prevTransfer localization.Stamp
	hasPrev      bool

	transactions atomic.Int64
	frameErrors  atomic.Int64
	ambiguous    atomic.Int64
}

// NewSPIRecorder はプロファイルを解決して記録器を作る。
// profileSpec は埋め込み名 (例 "rock5a-v1") かファイルパス。空なら既定。
func NewSPIRecorder(clock *loclog.Clock, rec *loclog.Writer, profileSpec string) (*SPIRecorder, error) {
	p, err := stmframe.Resolve(profileSpec)
	if err != nil {
		return nil, err
	}
	dec, err := stmframe.NewDecoder(p)
	if err != nil {
		return nil, err
	}
	bind, err := dec.Bind()
	if err != nil {
		return nil, err
	}
	return &SPIRecorder{
		clock:  clock,
		rec:    rec,
		dec:    dec,
		bind:   bind,
		values: dec.NewValues(),
		layout: DefaultCommandLayout(),
		// 既存実装に合わせてフレーム 2 個ぶんの窓を持つ。SPI は位相がずれる。
		window: make([]byte, dec.FrameSize()*2),
	}, nil
}

// SetCommandLayout は下りフレームのバイト配置を差し替える。
func (r *SPIRecorder) SetCommandLayout(l CommandLayout) { r.layout = l }

// ProfileName は使用中のプロファイル名を返す。
func (r *SPIRecorder) ProfileName() string { return r.dec.Profile().Name }

// Stats は記録の健全性を返す。
func (r *SPIRecorder) Stats() (transactions, frameErrors, ambiguous int64) {
	return r.transactions.Load(), r.frameErrors.Load(), r.ambiguous.Load()
}

// Record は 1 回の SPI トランザクションを記録し、取れたセンサ値を返す。
//
// before / after は conn.Tx() の直前・直後に取った時刻。その中点を転送時刻とする。
// time.Ticker の公称 8 ms は、受信が遅れると間隔を詰めたりティックを落としたり
// するので使わない (計画 §5.2)。
//
// ホットパスから呼ばれるので、正常系ではアロケートしない。
func (r *SPIRecorder) Record(tx, rx []byte, before, after time.Time) Sample {
	r.transactions.Add(1)

	transfer := r.clock.Midpoint(before, after)
	var dt time.Duration
	if r.hasPrev {
		dt = transfer.Sub(r.prevTransfer)
	}
	r.prevTransfer = transfer
	r.hasPrev = true

	r.pushWindow(rx)

	out := Sample{Transfer: transfer, Dt: dt}
	out.Command = r.decodeCommand(tx, transfer)

	spi := loclog.SPIRecord{
		TxStartNs:  int64(r.clock.StampOf(before)),
		TxEndNs:    int64(r.clock.StampOf(after)),
		TransferNs: int64(transfer),
		DtNs:       int64(dt),
		Profile:    r.dec.Profile().Name,
	}

	m, err := r.dec.Find(r.window)
	if err != nil {
		r.frameErrors.Add(1)
		spi.FrameError = err.Error()
		r.log(transfer, spi, tx)
		return out
	}
	spi.FrameOffsetBytes = m.Offset
	spi.FrameCount = m.Count
	spi.FrameValid = true
	if m.Ambiguous() {
		r.ambiguous.Add(1)
		out.Ambiguous = true
	}

	if err := r.dec.Decode(r.window, m.Offset, r.values); err != nil {
		r.frameErrors.Add(1)
		spi.FrameValid = false
		spi.FrameError = err.Error()
		r.log(transfer, spi, tx)
		return out
	}

	out.Valid = true
	r.fillSample(&out, transfer)

	r.log(transfer, spi, tx)
	r.logSensors(&out, transfer)
	return out
}

func (r *SPIRecorder) fillSample(out *Sample, transfer localization.Stamp) {
	// 車輪は SPI 上の並びのまま返す。論理輪番号への並べ替えは
	// localization.Kinematics.SlotsToLogical が設定に従って行う。
	r.fillWheelSample(out, transfer)
	r.fillImuSample(out, transfer)
}

func (r *SPIRecorder) log(transfer localization.Stamp, spi loclog.SPIRecord, tx []byte) {
	if r.rec == nil {
		return
	}
	r.rec.LogSPI(transfer, spi, tx, r.window)
}

func (r *SPIRecorder) logSensors(s *Sample, transfer localization.Stamp) {
	if r.rec == nil {
		return
	}
	wheel := loclog.WheelRecord{
		SampleNs:    int64(s.Wheel.Stamp),
		TransferNs:  int64(transfer),
		WheelFLRadS: s.Wheel.Omega[0],
		WheelBLRadS: s.Wheel.Omega[1],
		WheelBRRadS: s.Wheel.Omega[2],
		WheelFRRadS: s.Wheel.Omega[3],
		WheelFLRaw:  s.WheelRaw[0],
		WheelBLRaw:  s.WheelRaw[1],
		WheelBRRaw:  s.WheelRaw[2],
		WheelFRRaw:  s.WheelRaw[3],
	}
	if r.bind.Battery >= 0 {
		wheel.BatteryV = r.values.At(r.bind.Battery)
	}
	if r.bind.CapPower >= 0 {
		wheel.CapPower = r.values.RawAt(r.bind.CapPower)
	}
	if r.bind.SensorInfo >= 0 {
		wheel.SensorInfo = r.values.RawAt(r.bind.SensorInfo)
	}
	r.rec.LogWheel(transfer, wheel)

	if s.IMU.HasGyro || s.IMU.HasAccel {
		r.rec.LogIMU(transfer, loclog.ImuRecord{
			SampleNs:   int64(s.IMU.Stamp),
			TransferNs: int64(transfer),
			GyroZRadS:  s.IMU.GyroZ,
			AccelXMS2:  s.IMU.Accel.X,
			AccelYMS2:  s.IMU.Accel.Y,
			HasGyro:    s.IMU.HasGyro,
			HasAccel:   s.IMU.HasAccel,
		})
	}

	r.rec.LogCommand(transfer, s.Command)
	r.rec.LogTiming(transfer, loclog.TimingRecord{DtNs: int64(s.Dt)})
}

// pushWindow は受信窓を 1 フレームぶんずらして新しい受信を末尾に入れる。
// internal/rock5a/frame.go の pushSPIRxWindow と同じ動き。
func (r *SPIRecorder) pushWindow(rx []byte) {
	n := r.dec.FrameSize()
	copy(r.window, r.window[n:])
	tail := r.window[n:]
	for i := range tail {
		tail[i] = 0
	}
	copy(tail, rx)
}

func (r *SPIRecorder) decodeCommand(tx []byte, transfer localization.Stamp) loclog.CommandRecord {
	c := loclog.CommandRecord{TransferNs: int64(transfer)}
	l := r.layout
	get := func(i int) (byte, bool) {
		idx := l.FrameOffset + i
		if idx < 0 || idx >= len(tx) {
			return 0, false
		}
		return tx[idx], true
	}
	i16 := func(lo, hi int) int16 {
		a, ok1 := get(lo)
		b, ok2 := get(hi)
		if !ok1 || !ok2 {
			return 0
		}
		return int16(uint16(a) | uint16(b)<<8)
	}
	c.VelXMmS = i16(l.VelXLow, l.VelXHigh)
	c.VelYMmS = i16(l.VelYLow, l.VelYHigh)
	c.VelAngMradS = i16(l.VelAngLow, l.VelAngHigh)
	if v, ok := get(l.Dribble); ok {
		c.Dribble = v
	}
	if v, ok := get(l.Kick); ok {
		c.Kick = v
	}
	if v, ok := get(l.Chip); ok {
		c.Chip = v
	}
	if v, ok := get(l.Info); ok {
		c.Info = v
	}
	return c
}

// DescribeProfile は起動時ログ用の 1 行を返す。
func (r *SPIRecorder) DescribeProfile() string {
	p := r.dec.Profile()
	imu := "no IMU"
	if r.bind.HasGyro() || r.bind.HasAccel() {
		imu = fmt.Sprintf("gyro=%v accel=%v", r.bind.HasGyro(), r.bind.HasAccel())
	}
	return fmt.Sprintf("profile=%s frame=%dB fields=%d syncStrength=%dB (%s)",
		p.Name, p.FrameSize, len(p.Fields), p.SyncStrength(), imu)
}

// ObserveSPI は link.SPIObserver を満たす。リンク goroutine から呼ばれる。
//
// SPIRecorder は goroutine 安全ではない。受信窓・デコード結果・前回時刻を
// 保持しているので、SPI ループの 1 本からのみ呼ぶこと (計画 §7.2)。
func (r *SPIRecorder) ObserveSPI(tx, rx []byte, before, after time.Time) {
	r.Record(tx, rx, before, after)
}
