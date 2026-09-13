package loclog

import (
	"encoding/base64"
	"encoding/json"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi2/proto/pb_gen"
	"github.com/foxglove/mcap/go/mcap"
	"google.golang.org/protobuf/proto"
)

func newTestWriter(t *testing.T) (*Writer, *Clock, string) {
	t.Helper()
	clock := NewClock()
	path := filepath.Join(t.TempDir(), "test.mcap")
	w, err := NewWriter(clock, Options{
		Path:           path,
		StatusInterval: time.Hour, // 定期ステータスがテストへ紛れ込まないようにする
		StmProfile:     "rock5a-v1",
		Metadata:       map[string]string{"robot_id": "3"},
	})
	if err != nil {
		t.Fatalf("NewWriter: %v", err)
	}
	return w, clock, path
}

type readback struct {
	byTopic map[string][]json.RawMessage
	raw     map[string][][]byte
	seq     map[string][]uint32
	logTime map[string][]uint64
	info    *mcap.Info
}

func readAll(t *testing.T, path string) readback {
	t.Helper()
	f, err := os.Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer f.Close()

	r, err := mcap.NewReader(f)
	if err != nil {
		t.Fatalf("mcap.NewReader: %v", err)
	}
	defer r.Close()

	info, err := r.Info()
	if err != nil {
		t.Fatalf("Info: %v", err)
	}

	it, err := r.Messages()
	if err != nil {
		t.Fatalf("Messages: %v", err)
	}
	out := readback{
		byTopic: map[string][]json.RawMessage{},
		raw:     map[string][][]byte{},
		seq:     map[string][]uint32{},
		logTime: map[string][]uint64{},
		info:    info,
	}
	err = mcap.Range(it, func(_ *mcap.Schema, ch *mcap.Channel, m *mcap.Message) error {
		data := append([]byte(nil), m.Data...)
		out.raw[ch.Topic] = append(out.raw[ch.Topic], data)
		out.seq[ch.Topic] = append(out.seq[ch.Topic], m.Sequence)
		out.logTime[ch.Topic] = append(out.logTime[ch.Topic], m.LogTime)
		if ch.MessageEncoding == "json" {
			out.byTopic[ch.Topic] = append(out.byTopic[ch.Topic], json.RawMessage(data))
		}
		return nil
	})
	if err != nil {
		t.Fatalf("Range: %v", err)
	}
	return out
}

func TestWriterRoundTrip(t *testing.T) {
	w, clock, path := newTestWriter(t)

	tx := []byte{0xFF, 1, 2, 3, 0xAA}
	rx := []byte{0xFF, 140, 0, 9, 0xAA}
	stamp := clock.Now()
	w.LogSPI(stamp, SPIRecord{
		TransferNs:       int64(stamp),
		DtNs:             8_000_000,
		Profile:          "rock5a-v1",
		FrameOffsetBytes: 20,
		FrameCount:       1,
		FrameValid:       true,
	}, tx, rx)
	w.LogWheel(stamp, WheelRecord{SampleNs: int64(stamp) - 4_000_000, WheelFLRadS: 1.25, WheelFLRaw: 125})
	w.LogTiming(stamp, TimingRecord{DtNs: 8_100_000, LoopNs: 250_000})
	w.LogVisionMeta(stamp, VisionMetaRecord{CameraID: 2, FrameNumber: 100, FrameGap: 1, SelfSeen: true, SelfXMm: 1500})

	packet := &pb_gen.SSL_WrapperPacket{
		Detection: &pb_gen.SSL_DetectionFrame{
			FrameNumber: proto.Uint32(100),
			TCapture:    proto.Float64(1234.5),
			TSent:       proto.Float64(1234.51),
			CameraId:    proto.Uint32(2),
		},
	}
	blob, err := proto.Marshal(packet)
	if err != nil {
		t.Fatalf("marshal packet: %v", err)
	}
	w.LogVisionPacket(stamp, blob)

	if err := w.Close(); err != nil {
		t.Fatalf("Close: %v", err)
	}

	got := readAll(t, path)

	// --- SPI の生フレームがそのまま戻ること ---
	spis := got.byTopic["/in/spi"]
	if len(spis) != 1 {
		t.Fatalf("/in/spi has %d messages, want 1", len(spis))
	}
	var spi SPIRecord
	if err := json.Unmarshal(spis[0], &spi); err != nil {
		t.Fatalf("unmarshal spi: %v", err)
	}
	if want := base64.StdEncoding.EncodeToString(tx); spi.TxBase64 != want {
		t.Errorf("tx_base64 = %q, want %q", spi.TxBase64, want)
	}
	if want := base64.StdEncoding.EncodeToString(rx); spi.RxBase64 != want {
		t.Errorf("rx_base64 = %q, want %q", spi.RxBase64, want)
	}
	if spi.Profile != "rock5a-v1" || spi.FrameOffsetBytes != 20 || !spi.FrameValid {
		t.Errorf("spi record round trip lost fields: %+v", spi)
	}

	// --- vision は生の protobuf がビット単位で戻ること ---
	visions := got.raw["/in/vision"]
	if len(visions) != 1 {
		t.Fatalf("/in/vision has %d messages, want 1", len(visions))
	}
	var back pb_gen.SSL_WrapperPacket
	if err := proto.Unmarshal(visions[0], &back); err != nil {
		t.Fatalf("unmarshal recorded vision packet: %v", err)
	}
	if back.GetDetection().GetFrameNumber() != 100 || back.GetDetection().GetCameraId() != 2 {
		t.Errorf("vision packet round trip: %+v", back.GetDetection())
	}

	// --- 単位付きのフィールド名で書けていること ---
	wheels := got.byTopic["/sensors/wheel"]
	if len(wheels) != 1 {
		t.Fatalf("/sensors/wheel has %d messages, want 1", len(wheels))
	}
	var m map[string]any
	if err := json.Unmarshal(wheels[0], &m); err != nil {
		t.Fatal(err)
	}
	for _, key := range []string{"wheelFL_rad_s", "wheelFL_raw", "sample_ns", "battery_V"} {
		if _, ok := m[key]; !ok {
			t.Errorf("/sensors/wheel is missing %q (keys: %v)", key, keysOf(m))
		}
	}
}

func keysOf(m map[string]any) []string {
	out := make([]string, 0, len(m))
	for k := range m {
		out = append(out, k)
	}
	return out
}

// 起動時のアンカーが metadata に入っていること。読む側はこれで壁時計へ引き直す。
func TestWriterWritesClockAnchor(t *testing.T) {
	w, clock, path := newTestWriter(t)
	w.LogTiming(clock.Now(), TimingRecord{})
	if err := w.Close(); err != nil {
		t.Fatalf("Close: %v", err)
	}

	f, err := os.Open(path)
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	r, err := mcap.NewReader(f)
	if err != nil {
		t.Fatal(err)
	}
	defer r.Close()
	info, err := r.Info()
	if err != nil {
		t.Fatal(err)
	}
	if len(info.MetadataIndexes) == 0 {
		t.Fatal("no metadata records were written")
	}
	md, err := r.GetMetadata(info.MetadataIndexes[0].Offset)
	if err != nil {
		t.Fatal(err)
	}
	for _, key := range []string{"clock_epoch_ns_at_start", "clock_monotonic_ns_at_start", "stm_profile"} {
		if _, ok := md.Metadata[key]; !ok {
			t.Errorf("metadata is missing %q (have %v)", key, md.Metadata)
		}
	}
	if md.Metadata["robot_id"] != "3" {
		t.Errorf("caller metadata was dropped: %v", md.Metadata)
	}
	if md.Metadata["clock_epoch_ns_at_start"] == "0" {
		t.Error("clock anchor is zero")
	}
}

// vision チャンネルに FileDescriptorSet が付いていること。
// これがあれば Foxglove が単体でデコードできる。
func TestWriterEmbedsVisionSchema(t *testing.T) {
	w, clock, path := newTestWriter(t)
	w.LogVisionPacket(clock.Now(), []byte{})
	if err := w.Close(); err != nil {
		t.Fatal(err)
	}
	got := readAll(t, path)

	var visionChannel *mcap.Channel
	for _, ch := range got.info.Channels {
		if ch.Topic == "/in/vision" {
			visionChannel = ch
		}
	}
	if visionChannel == nil {
		t.Fatal("/in/vision channel was not declared")
	}
	if visionChannel.MessageEncoding != "protobuf" {
		t.Errorf("/in/vision encoding = %q, want protobuf", visionChannel.MessageEncoding)
	}
	schema := got.info.Schemas[visionChannel.SchemaID]
	if schema == nil {
		t.Fatal("/in/vision has no schema")
	}
	if schema.Name != "SSL_WrapperPacket" || schema.Encoding != "protobuf" {
		t.Errorf("schema = %q/%q, want SSL_WrapperPacket/protobuf", schema.Name, schema.Encoding)
	}
	if len(schema.Data) == 0 {
		t.Error("schema data (FileDescriptorSet) is empty")
	}
}

// チャンネルごとに連番が採番されること (欠落検出に使う)。
func TestWriterSequencesPerChannel(t *testing.T) {
	w, clock, path := newTestWriter(t)
	for i := 0; i < 5; i++ {
		w.LogWheel(clock.Now(), WheelRecord{})
	}
	for i := 0; i < 3; i++ {
		w.LogTiming(clock.Now(), TimingRecord{})
	}
	if err := w.Close(); err != nil {
		t.Fatal(err)
	}
	got := readAll(t, path)

	if want := []uint32{1, 2, 3, 4, 5}; !equalSeq(got.seq["/sensors/wheel"], want) {
		t.Errorf("/sensors/wheel sequences = %v, want %v", got.seq["/sensors/wheel"], want)
	}
	if want := []uint32{1, 2, 3}; !equalSeq(got.seq["/est/timing"], want) {
		t.Errorf("/est/timing sequences = %v, want %v", got.seq["/est/timing"], want)
	}
}

func equalSeq(a, b []uint32) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

// 記録が詰まっても推定と制御を止めない。捨てた件数は必ず数える。
func TestWriterDropsInsteadOfBlocking(t *testing.T) {
	clock := NewClock()
	path := filepath.Join(t.TempDir(), "drop.mcap")
	w, err := NewWriter(clock, Options{Path: path, QueueSize: 1, StatusInterval: time.Hour})
	if err != nil {
		t.Fatal(err)
	}

	// キュー容量をはるかに超える量を一気に投げる。ブロックしたらテストが
	// タイムアウトするので、止まらないこと自体が検証になる。
	const n = 100000
	done := make(chan struct{})
	go func() {
		for i := 0; i < n; i++ {
			w.LogWheel(clock.Now(), WheelRecord{})
		}
		close(done)
	}()
	select {
	case <-done:
	case <-time.After(30 * time.Second):
		t.Fatal("LogWheel blocked; the recorder must never stall the caller")
	}

	stats := w.Stats()
	if stats.Dropped == 0 {
		t.Error("expected some messages to be dropped with a queue of 1")
	}
	if err := w.Close(); err != nil {
		t.Fatal(err)
	}

	got := readAll(t, path)
	// 落としたことがログの中に残っていること。
	statuses := got.byTopic["/recorder/status"]
	if len(statuses) == 0 {
		t.Fatal("no /recorder/status was written")
	}
	var last StatusRecord
	if err := json.Unmarshal(statuses[len(statuses)-1], &last); err != nil {
		t.Fatal(err)
	}
	if last.Dropped == 0 {
		t.Errorf("/recorder/status did not record the drops: %+v", last)
	}
	if last.QueueCapacity != 1 {
		t.Errorf("queueCapacity_count = %d, want 1", last.QueueCapacity)
	}
}

// 計画 §7.3: SPI のホットパスから呼ぶ記録 API は 0 アロケーションであること。
func TestHotPathLogsDoNotAllocate(t *testing.T) {
	clock := NewClock()
	path := filepath.Join(t.TempDir(), "alloc.mcap")
	// キューを 1 にして「捨てる側」の経路も込みで測る。
	w, err := NewWriter(clock, Options{Path: path, QueueSize: 1, StatusInterval: time.Hour})
	if err != nil {
		t.Fatal(err)
	}
	defer w.Close()

	tx := make([]byte, 20)
	rx := make([]byte, 40)
	stamp := clock.Now()

	got := testing.AllocsPerRun(1000, func() {
		w.LogSPI(stamp, SPIRecord{FrameValid: true}, tx, rx)
		w.LogWheel(stamp, WheelRecord{WheelFLRadS: 1})
		w.LogIMU(stamp, ImuRecord{})
		w.LogTiming(stamp, TimingRecord{DtNs: 8_000_000})
	})
	if got != 0 {
		t.Errorf("hot-path logging allocated %v times per run, want 0", got)
	}
}

func TestClockMonotonicAndAnchor(t *testing.T) {
	start := time.Now()
	c := NewClockAt(start)

	if c.EpochWallNs() != start.UnixNano() {
		t.Errorf("EpochWallNs = %d, want %d", c.EpochWallNs(), start.UnixNano())
	}
	if got := c.StampOf(start); got != 0 {
		t.Errorf("StampOf(start) = %v, want 0", got)
	}

	before := start.Add(10 * time.Millisecond)
	after := start.Add(12 * time.Millisecond)
	if got, want := c.Midpoint(before, after), localization.Stamp(11*time.Millisecond); got != want {
		t.Errorf("Midpoint = %v, want %v", got, want)
	}

	s := localization.Stamp(5 * time.Second)
	if got, want := c.WallNs(s), start.UnixNano()+int64(5*time.Second); got != want {
		t.Errorf("WallNs = %d, want %d", got, want)
	}

	// Now() は単調に増える。
	prev := c.Now()
	for i := 0; i < 100; i++ {
		now := c.Now()
		if now < prev {
			t.Fatalf("Now() went backwards: %v -> %v", prev, now)
		}
		prev = now
	}
}

func TestUnknownCompressionIsRejected(t *testing.T) {
	_, err := NewWriter(NewClock(), Options{
		Path:        filepath.Join(t.TempDir(), "x.mcap"),
		Compression: "brotli",
	})
	if err == nil {
		t.Error("expected an error for an unsupported compression format")
	}
}
