package loclog

import (
	"bufio"
	"encoding/base64"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"sync"
	"sync/atomic"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
	"github.com/foxglove/mcap/go/mcap"
)

// 記録が詰まっても推定と制御は絶対に止めない (計画 §6.3)。
// Log* はすべてノンブロッキングで、キューが満杯なら捨てて件数を数える。

// 受信窓とフレームの上限。union レコードを固定長にしてアロケーションを避ける。
const (
	maxTxBytes = 64
	maxRxBytes = 128
)

// Options は Writer の設定。
type Options struct {
	// Path は出力先の .mcap ファイル。
	Path string
	// QueueSize は記録キューの深さ。0 なら既定値。
	QueueSize int
	// ChunkSize は MCAP のチャンク目標サイズ [bytes]。0 なら既定値。
	ChunkSize int64
	// Compression は "lz4" / "zstd" / "none"。空なら lz4。
	// Rock5A の CPU を食わないよう既定は lz4。
	Compression string
	// StatusInterval は /recorder/status を出す間隔。0 なら 1 秒。
	StatusInterval time.Duration
	// Metadata は MCAP の metadata へ追記する任意の情報 (ロボット ID など)。
	Metadata map[string]string
	// StmProfile は SPI フレームの解釈に使ったプロファイル名。
	StmProfile string
}

func (o *Options) withDefaults() {
	if o.QueueSize <= 0 {
		// 125 Hz の SPI + 60 Hz の vision に対して数秒ぶんの余裕。
		o.QueueSize = 4096
	}
	if o.ChunkSize <= 0 {
		o.ChunkSize = 1 << 20
	}
	if o.Compression == "" {
		o.Compression = "lz4"
	}
	if o.StatusInterval <= 0 {
		o.StatusInterval = time.Second
	}
}

// record はキューを流れる 1 件。
//
// interface{} で渡すと呼び出し側 (= SPI のホットパス) でボックス化の
// アロケーションが起きるので、高頻度チャンネルは固定長の共用体で持つ。
// JSON 化はすべて記録 goroutine 側で行う。
type record struct {
	ch    Channel
	stamp localization.Stamp

	spi    SPIRecord
	cmd    CommandRecord
	wheel  WheelRecord
	imu    ImuRecord
	timing TimingRecord
	vmeta  VisionMetaRecord

	txBuf [maxTxBytes]byte
	txLen int
	rxBuf [maxRxBytes]byte
	rxLen int

	// blob は vision の protobuf など可変長のペイロード。
	// 呼び出し元 (vision 受信 goroutine) がコピー済みのものを渡す。
	blob []byte
	// any は低頻度チャンネル用の逃げ道。ボックス化が起きるので
	// SPI のホットパスからは使わない。
	any any
}

// Writer は MCAP へ書き出す記録器。
type Writer struct {
	clock *Clock
	opts  Options

	queue chan record
	done  chan struct{}
	wg    sync.WaitGroup

	closeOnce sync.Once
	closeErr  error

	// 統計。Log* から atomic で触る。
	written     atomic.Int64
	dropped     atomic.Int64
	writeErrors atomic.Int64
	bytes       atomic.Int64
}

// NewWriter は MCAP ファイルを開き、記録 goroutine を起動する。
func NewWriter(clock *Clock, opts Options) (*Writer, error) {
	if clock == nil {
		return nil, fmt.Errorf("loclog: nil clock")
	}
	opts.withDefaults()
	if opts.Path == "" {
		return nil, fmt.Errorf("loclog: empty output path")
	}
	if dir := filepath.Dir(opts.Path); dir != "" {
		if err := os.MkdirAll(dir, 0o755); err != nil {
			return nil, fmt.Errorf("loclog: create %s: %w", dir, err)
		}
	}

	w := &Writer{
		clock: clock,
		opts:  opts,
		queue: make(chan record, opts.QueueSize),
		done:  make(chan struct{}),
	}

	sink, err := newSink(clock, opts)
	if err != nil {
		return nil, err
	}

	w.wg.Add(1)
	go w.run(sink)
	return w, nil
}

// DefaultPath は日時を含むログのパスを組み立てる。
// 実機は電源を落として止めるので、起動ごとに別ファイルにする。
func DefaultPath(dir string, robotID uint32) string {
	name := fmt.Sprintf("racoon-loc-%s-id%d.mcap", time.Now().Format("20060102-150405"), robotID)
	return filepath.Join(dir, name)
}

// --- 記録 API (すべてノンブロッキング) ------------------------------------

// LogSPI は SPI 1 トランザクションを記録する。アロケートしない。
func (w *Writer) LogSPI(stamp localization.Stamp, rec SPIRecord, tx, rx []byte) {
	var r record
	r.ch = ChSPI
	r.stamp = stamp
	r.spi = rec
	r.txLen = copy(r.txBuf[:], tx)
	r.rxLen = copy(r.rxBuf[:], rx)
	w.push(r)
}

// LogWheel は 4 輪のサンプルを記録する。アロケートしない。
func (w *Writer) LogWheel(stamp localization.Stamp, rec WheelRecord) {
	w.push(record{ch: ChWheel, stamp: stamp, wheel: rec})
}

// LogIMU は IMU のサンプルを記録する。アロケートしない。
func (w *Writer) LogIMU(stamp localization.Stamp, rec ImuRecord) {
	w.push(record{ch: ChIMU, stamp: stamp, imu: rec})
}

// LogCommand は STM へ送った指令を記録する。アロケートしない。
func (w *Writer) LogCommand(stamp localization.Stamp, rec CommandRecord) {
	w.push(record{ch: ChOutCommand, stamp: stamp, cmd: rec})
}

// LogTiming はループの実測時間を記録する。アロケートしない。
func (w *Writer) LogTiming(stamp localization.Stamp, rec TimingRecord) {
	w.push(record{ch: ChEstTiming, stamp: stamp, timing: rec})
}

// LogVisionMeta は vision パケットの時刻まわりを記録する。
func (w *Writer) LogVisionMeta(stamp localization.Stamp, rec VisionMetaRecord) {
	w.push(record{ch: ChVisionMeta, stamp: stamp, vmeta: rec})
}

// LogVisionPacket は SSL_WrapperPacket の生バイト列を記録する。
//
// data は呼び出し元が保持を諦めたコピーでなければならない。
// Writer は受け取ったスライスをそのまま保持する。
func (w *Writer) LogVisionPacket(stamp localization.Stamp, data []byte) {
	w.push(record{ch: ChVision, stamp: stamp, blob: data})
}

// LogJSON は任意の値を JSON として記録する。
//
// 値のボックス化でアロケーションが起きるので、低頻度のチャンネル
// (推定出力・目標位置・送信指令など) にだけ使う。
func (w *Writer) LogJSON(ch Channel, stamp localization.Stamp, v any) {
	w.push(record{ch: ch, stamp: stamp, any: v})
}

func (w *Writer) push(r record) {
	select {
	case w.queue <- r:
	default:
		w.dropped.Add(1)
	}
}

// Stats は記録自体の健全性を返す。
func (w *Writer) Stats() StatusRecord {
	return StatusRecord{
		UptimeNs:      int64(w.clock.Now()),
		Written:       w.written.Load(),
		Dropped:       w.dropped.Load(),
		WriteErrors:   w.writeErrors.Load(),
		QueueDepth:    len(w.queue),
		QueueCapacity: cap(w.queue),
		BytesWritten:  w.bytes.Load(),
	}
}

// Close はキューを流し切って MCAP を閉じる。
func (w *Writer) Close() error {
	w.closeOnce.Do(func() {
		close(w.done)
		w.wg.Wait()
	})
	return w.closeErr
}

// --- 記録 goroutine -------------------------------------------------------

func (w *Writer) run(s *sink) {
	defer w.wg.Done()

	status := time.NewTicker(w.opts.StatusInterval)
	defer status.Stop()

	drain := func() {
		for {
			select {
			case r := <-w.queue:
				w.write(s, r)
			default:
				return
			}
		}
	}

	for {
		select {
		case r := <-w.queue:
			w.write(s, r)
		case <-status.C:
			w.write(s, record{ch: ChRecorderStatus, stamp: w.clock.Now(), any: w.Stats()})
			// チャンクを閉じるたびに flush する。実機は電源を落として止めるので、
			// 書き残しがあると最後の数秒が丸ごと消える (計画 §6.1)。
			if err := s.flush(); err != nil {
				w.writeErrors.Add(1)
			}
		case <-w.done:
			drain()
			// 最後の状態を必ず残す。記録が落ちたことがログの外にしか無いと、
			// 後日ログだけ見る人に穴が見えない。
			w.write(s, record{ch: ChRecorderStatus, stamp: w.clock.Now(), any: w.Stats()})
			w.closeErr = s.close()
			return
		}
	}
}

func (w *Writer) write(s *sink, r record) {
	n, err := s.write(w, r)
	if err != nil {
		w.writeErrors.Add(1)
		return
	}
	w.written.Add(1)
	w.bytes.Add(int64(n))
}

// sink は MCAP ファイルそのもの。記録 goroutine からのみ触る。
type sink struct {
	file   *os.File
	buf    *bufio.Writer
	mcapW  *mcap.Writer
	clock  *Clock
	seq    [numChannels]uint32
	chanID [numChannels]uint16

	// scratch は JSON 化の作業領域。記録 goroutine 専有なので使い回せる。
	scratch []byte
}

func newSink(clock *Clock, opts Options) (*sink, error) {
	f, err := os.Create(opts.Path)
	if err != nil {
		return nil, fmt.Errorf("loclog: create %s: %w", opts.Path, err)
	}
	buf := bufio.NewWriterSize(f, 256*1024)

	compression := mcap.CompressionLZ4
	switch opts.Compression {
	case "zstd":
		compression = mcap.CompressionZSTD
	case "none":
		compression = mcap.CompressionNone
	case "lz4":
	default:
		f.Close()
		return nil, fmt.Errorf("loclog: unknown compression %q", opts.Compression)
	}

	mw, err := mcap.NewWriter(buf, &mcap.WriterOptions{
		Chunked:     true,
		ChunkSize:   opts.ChunkSize,
		Compression: compression,
		IncludeCRC:  true,
	})
	if err != nil {
		f.Close()
		return nil, fmt.Errorf("loclog: mcap writer: %w", err)
	}
	if err := mw.WriteHeader(&mcap.Header{Profile: "", Library: "ssl-RACOON-Pi3/loclog"}); err != nil {
		f.Close()
		return nil, fmt.Errorf("loclog: write header: %w", err)
	}

	s := &sink{file: f, buf: buf, mcapW: mw, clock: clock, scratch: make([]byte, 0, 4096)}

	// 起動時のアンカーを metadata に書く。読む側はこれで壁時計へ引き直せる。
	meta := map[string]string{
		"clock_epoch_ns_at_start":      strconv.FormatInt(clock.EpochWallNs(), 10),
		"clock_monotonic_ns_at_start":  "0",
		"clock_epoch_rfc3339_at_start": time.Unix(0, clock.EpochWallNs()).Format(time.RFC3339Nano),
		"stm_profile":                  opts.StmProfile,
	}
	for k, v := range opts.Metadata {
		meta[k] = v
	}
	if err := mw.WriteMetadata(&mcap.Metadata{Name: "recording", Metadata: meta}); err != nil {
		f.Close()
		return nil, fmt.Errorf("loclog: write metadata: %w", err)
	}

	if err := s.declareChannels(); err != nil {
		f.Close()
		return nil, err
	}
	return s, nil
}

func (s *sink) declareChannels() error {
	// JSON チャンネルはスキーマ無し (schema_id = 0) で宣言する。MCAP 仕様上は
	// 有効で、Foxglove の Raw Message / Plot パネルはそのまま読める。
	// 各フィールドが何かは名前に埋めた単位で判別できるようにしてある。
	visionSchemaID, err := s.writeVisionSchema()
	if err != nil {
		return err
	}

	for ch := Channel(0); ch < numChannels; ch++ {
		var schemaID uint16
		if ch == ChVision {
			schemaID = visionSchemaID
		}
		id := uint16(ch) + 1
		s.chanID[ch] = id
		if err := s.mcapW.WriteChannel(&mcap.Channel{
			ID:              id,
			SchemaID:        schemaID,
			Topic:           ch.Topic(),
			MessageEncoding: ch.Encoding(),
			Metadata:        map[string]string{},
		}); err != nil {
			return fmt.Errorf("loclog: declare channel %s: %w", ch.Topic(), err)
		}
	}
	return nil
}

func (s *sink) write(w *Writer, r record) (int, error) {
	var data []byte
	var err error

	switch r.ch {
	case ChVision:
		data = r.blob
	case ChSPI:
		rec := r.spi
		rec.TxBase64 = base64.StdEncoding.EncodeToString(r.txBuf[:r.txLen])
		rec.RxBase64 = base64.StdEncoding.EncodeToString(r.rxBuf[:r.rxLen])
		data, err = s.marshal(rec)
	case ChWheel:
		data, err = s.marshal(r.wheel)
	case ChIMU:
		data, err = s.marshal(r.imu)
	case ChOutCommand:
		data, err = s.marshal(r.cmd)
	case ChEstTiming:
		data, err = s.marshal(r.timing)
	case ChVisionMeta:
		data, err = s.marshal(r.vmeta)
	default:
		data, err = s.marshal(r.any)
	}
	if err != nil {
		return 0, err
	}

	s.seq[r.ch]++
	logTime := uint64(s.clock.WallNs(r.stamp))
	if err := s.mcapW.WriteMessage(&mcap.Message{
		ChannelID:   s.chanID[r.ch],
		Sequence:    s.seq[r.ch],
		LogTime:     logTime,
		PublishTime: logTime,
		Data:        data,
	}); err != nil {
		return 0, err
	}
	return len(data), nil
}

func (s *sink) marshal(v any) ([]byte, error) {
	b, err := json.Marshal(v)
	if err != nil {
		return nil, fmt.Errorf("loclog: marshal: %w", err)
	}
	return b, nil
}

func (s *sink) flush() error { return s.buf.Flush() }

func (s *sink) close() error {
	if err := s.mcapW.Close(); err != nil {
		s.buf.Flush()
		s.file.Close()
		return fmt.Errorf("loclog: close mcap: %w", err)
	}
	if err := s.buf.Flush(); err != nil {
		s.file.Close()
		return fmt.Errorf("loclog: flush: %w", err)
	}
	if err := s.file.Sync(); err != nil {
		s.file.Close()
		return fmt.Errorf("loclog: sync: %w", err)
	}
	return s.file.Close()
}
