// Package locadapter は自己位置推定をハードウェア・通信へ繋ぐ層。
//
// 推定コア (localization / timesync / stmframe / loclog) はここに依存しない。
// ハードウェアに触るものだけをこの層へ閉じ込める (計画 §7.1)。
package locadapter

import (
	"fmt"
	"log"
	"net"
	"sync"
	"sync/atomic"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi3/internal/loclog"
	"github.com/Rione/ssl-RACOON-Pi3/internal/timesync"
	"github.com/Rione/ssl-RACOON-Pi3/proto/pb_gen"
	"google.golang.org/protobuf/proto"
)

// DefaultVisionAddress は SSL-Vision のマルチキャスト。ロボットへ直送される。
// 出典: ssl-RAVEN/app/config/network.yaml
const DefaultVisionAddress = "224.5.23.2:10694"

// Team は自機の色。SSL-Vision は robots_blue / robots_yellow を分けて送り、
// robot_id はチーム内で採番されるので、色が分からないと自機を特定できない。
type Team uint8

// チーム色。
const (
	TeamBlue Team = iota
	TeamYellow
)

func (t Team) String() string {
	if t == TeamYellow {
		return "yellow"
	}
	return "blue"
}

// ParseTeam は "blue" / "yellow" を解釈する。
func ParseTeam(s string) (Team, error) {
	switch s {
	case "blue", "b", "BLUE":
		return TeamBlue, nil
	case "yellow", "y", "YELLOW":
		return TeamYellow, nil
	}
	return TeamBlue, fmt.Errorf("unknown team %q (want blue or yellow)", s)
}

// VisionConfig は vision 受信の設定。
type VisionConfig struct {
	// Address はマルチキャストの宛先。空なら DefaultVisionAddress。
	Address string
	// Interface は受信に使う NIC 名。空ならシステム既定。
	//
	// Rock5A は有線と無線を両方持つので、マルチキャストの join 先を
	// 取り違えると「パケットが一切来ない」という形で静かに失敗する。
	Interface string
	// RobotID は自機の ID (DIP スイッチ由来)。
	RobotID uint32
	// Team は自機の色。
	Team Team
	// ReadBufferBytes はソケットの受信バッファ。0 なら 1 MB。
	ReadBufferBytes int
}

// VisionStats はマルチキャストの健全性。P1 の必須計測項目 (計画 §11 / D-1)。
type VisionStats struct {
	// Packets は受信したパケット総数。
	Packets int64
	// DecodeErrors は protobuf として読めなかった数。
	DecodeErrors int64
	// LostFrames は frame_number の欠番から数えた損失フレーム数。
	LostFrames int64
	// Reordered は frame_number が戻った回数 (順序逆転または重複)。
	Reordered int64
	// SelfSeen は自機が写っていたフレーム数。
	SelfSeen int64
	// Dropped は推定器が詰まっていて捨てた観測数。
	Dropped int64
	// BadTimestamps は t_capture が 0 や負だった数 (計画 §5.5)。
	BadTimestamps int64
}

// LossRate は欠番率を返す。
func (s VisionStats) LossRate() float64 {
	total := s.LostFrames + s.Packets
	if total == 0 {
		return 0
	}
	return float64(s.LostFrames) / float64(total)
}

// VisionReceiver は SSL-Vision のマルチキャストを受け、自機の観測を取り出す。
type VisionReceiver struct {
	cfg   VisionConfig
	clock *loclog.Clock
	sync  timesync.Provider
	rec   *loclog.Writer

	// out は推定器へ渡すチャンネル。バッファ長 1 でノンブロッキング送信する。
	// フィルタが詰まっても受信側は止まらない (計画 §7.2)。
	out chan localization.VisionPose

	// latest は制御・監視がロックなしで読むための最新観測。
	latest atomic.Pointer[localization.VisionPose]

	mu       sync.Mutex
	lastSeen map[uint32]uint32 // camera_id -> 直近の frame_number
	seenAny  map[uint32]bool

	packets       atomic.Int64
	decodeErrors  atomic.Int64
	lostFrames    atomic.Int64
	reordered     atomic.Int64
	selfSeen      atomic.Int64
	dropped       atomic.Int64
	badTimestamps atomic.Int64
}

// NewVisionReceiver は受信器を作る。
//
// sync は camera_id ごとに独立した推定器を貸し出す Provider である。
// カメラごとに露光から送信までの処理遅延が違うので、1 つの推定器に混ぜると
// 凸包の下端が一番速いカメラに引きずられる (計画 §5.3)。
// nil なら到着時刻をそのまま使う。
func NewVisionReceiver(clock *loclog.Clock, sync timesync.Provider, rec *loclog.Writer, cfg VisionConfig) *VisionReceiver {
	if cfg.Address == "" {
		cfg.Address = DefaultVisionAddress
	}
	if cfg.ReadBufferBytes <= 0 {
		cfg.ReadBufferBytes = 1 << 20
	}
	if sync == nil {
		sync = timesync.ArrivalProvider{}
	}
	return &VisionReceiver{
		cfg:   cfg,
		clock: clock,
		sync:  sync,
		rec:   rec,
		// **バッファ 1 では足りない。** vision は約 116 Hz、吸い上げるのは
		// SPI の 125 Hz なので、ジッタのたびに落ちる。8 周期ぶん持たせる。
		// それでも詰まったら捨てて数える (計画 §7.2)。
		out:      make(chan localization.VisionPose, 8),
		lastSeen: make(map[uint32]uint32),
		seenAny:  make(map[uint32]bool),
	}
}

// Observations は自機の観測が流れるチャンネルを返す。
func (v *VisionReceiver) Observations() <-chan localization.VisionPose { return v.out }

// Latest は最後に取れた自機の観測を返す。
func (v *VisionReceiver) Latest() (localization.VisionPose, bool) {
	p := v.latest.Load()
	if p == nil {
		return localization.VisionPose{}, false
	}
	return *p, true
}

// Stats は受信の健全性を返す。
func (v *VisionReceiver) Stats() VisionStats {
	return VisionStats{
		Packets:       v.packets.Load(),
		DecodeErrors:  v.decodeErrors.Load(),
		LostFrames:    v.lostFrames.Load(),
		Reordered:     v.reordered.Load(),
		SelfSeen:      v.selfSeen.Load(),
		Dropped:       v.dropped.Load(),
		BadTimestamps: v.badTimestamps.Load(),
	}
}

// Run はマルチキャストを購読し、done が閉じるまで受信し続ける。
func (v *VisionReceiver) Run(done <-chan struct{}) error {
	addr, err := net.ResolveUDPAddr("udp4", v.cfg.Address)
	if err != nil {
		return fmt.Errorf("vision: resolve %s: %w", v.cfg.Address, err)
	}

	var iface *net.Interface
	if v.cfg.Interface != "" {
		iface, err = net.InterfaceByName(v.cfg.Interface)
		if err != nil {
			return fmt.Errorf("vision: interface %s: %w", v.cfg.Interface, err)
		}
	}

	conn, err := net.ListenMulticastUDP("udp4", iface, addr)
	if err != nil {
		return fmt.Errorf("vision: join %s: %w", v.cfg.Address, err)
	}
	defer conn.Close()

	if err := conn.SetReadBuffer(v.cfg.ReadBufferBytes); err != nil {
		// 上限に当たっただけのことが多いので、致命扱いにはしない。
		log.Printf("[VISION] SetReadBuffer(%d) failed: %v", v.cfg.ReadBufferBytes, err)
	}
	log.Printf("[VISION] Listening on %s (team=%s id=%d)", v.cfg.Address, v.cfg.Team, v.cfg.RobotID)

	go func() {
		<-done
		conn.Close() // 読み込み中の ReadFromUDP を解く
	}()

	buf := make([]byte, 65536)
	for {
		select {
		case <-done:
			return nil
		default:
		}

		n, _, err := conn.ReadFromUDP(buf)
		recv := v.clock.Now()
		if err != nil {
			select {
			case <-done:
				return nil
			default:
			}
			log.Printf("[VISION] read: %v", err)
			continue
		}
		if n == 0 {
			continue
		}
		v.handlePacket(buf[:n], recv)
	}
}

// handlePacket は 1 パケットを処理する。Run から切り離してあるのはテストのため。
func (v *VisionReceiver) handlePacket(data []byte, recv localization.Stamp) {
	v.packets.Add(1)

	// 生のパケットをそのまま残す。デコード方法が変わっても後から読み直せる。
	if v.rec != nil {
		v.rec.LogVisionPacket(recv, append([]byte(nil), data...))
	}

	var pkt pb_gen.SSL_WrapperPacket
	if err := proto.Unmarshal(data, &pkt); err != nil {
		v.decodeErrors.Add(1)
		return
	}
	det := pkt.GetDetection()
	if det == nil {
		return // geometry だけのパケット
	}

	meta := loclog.VisionMetaRecord{
		CameraID:    det.GetCameraId(),
		FrameNumber: det.GetFrameNumber(),
		TCaptureS:   det.GetTCapture(),
		TSentS:      det.GetTSent(),
		RecvNs:      int64(recv),
		PacketBytes: len(data),
	}
	// t_sent - t_capture は同一クロック内の差なので、同期なしで正確に測れる
	// vision の処理遅延である (計画 §5.1 / D-2)。
	meta.ProcessingS = meta.TSentS - meta.TCaptureS

	meta.FrameGap, meta.LostFrames, meta.TotalFrames = v.trackFrameNumber(det.GetCameraId(), det.GetFrameNumber())

	obs, ok := v.selfObservation(det)
	if ok {
		meta.SelfSeen = true
		meta.SelfXMm = float64(obs.x)
		meta.SelfYMm = float64(obs.y)
		meta.SelfThetaRad = float64(obs.theta)
		meta.SelfConfidence = float64(obs.confidence)
	}

	// t_capture が不正な場合は到着時刻へフォールバックし、発生率を残す。
	capture := det.GetTCapture()
	valid := capture > 0
	if !valid {
		v.badTimestamps.Add(1)
	}

	var mapped localization.Stamp
	var q timesync.Quality
	if valid {
		// camera_id ごとの推定器へ通す。
		clock := v.sync.For(det.GetCameraId())
		remote := timesync.SecondsToStamp(capture)
		clock.Observe(remote, recv)
		mapped, q = clock.ToLocal(remote, recv)
	} else {
		mapped = recv
	}
	meta.MappedNs = int64(mapped)
	meta.Mapped = q.Valid
	meta.SkewPpm = q.SkewPpm
	meta.OffsetNs = q.OffsetNs

	if v.rec != nil {
		v.rec.LogVisionMeta(recv, meta)
	}
	if !ok {
		return
	}

	v.selfSeen.Add(1)
	pose := localization.VisionPose{
		Stamp:    mapped,
		Arrival:  recv,
		TCapture: capture,
		TSent:    det.GetTSent(),
		Pose: localization.Pose2{
			// SSL-Vision は mm。コア内部は SI なので、ここが唯一の変換点。
			X:     float64(obs.x) / 1000,
			Y:     float64(obs.y) / 1000,
			Theta: localization.WrapAngle(float64(obs.theta)),
		},
		Confidence:  float64(obs.confidence),
		CameraID:    det.GetCameraId(),
		FrameNumber: det.GetFrameNumber(),
		Mapped:      q.Valid,
	}
	v.latest.Store(&pose)

	select {
	case v.out <- pose:
	default:
		// フィルタが詰まっていても受信側は止めない。捨てた件数は数える。
		v.dropped.Add(1)
	}
}

type selfObs struct {
	x, y, theta, confidence float32
}

func (v *VisionReceiver) selfObservation(det *pb_gen.SSL_DetectionFrame) (selfObs, bool) {
	robots := det.GetRobotsBlue()
	if v.cfg.Team == TeamYellow {
		robots = det.GetRobotsYellow()
	}
	best := selfObs{}
	found := false
	for _, r := range robots {
		if r.GetRobotId() != v.cfg.RobotID {
			continue
		}
		// 同一フレームに同じ ID が複数出たら、confidence が高い方を採る。
		if found && r.GetConfidence() <= best.confidence {
			continue
		}
		best = selfObs{
			x:          r.GetX(),
			y:          r.GetY(),
			theta:      r.GetOrientation(),
			confidence: r.GetConfidence(),
		}
		found = true
	}
	return best, found
}

// trackFrameNumber は camera_id ごとに frame_number の連続性を追う。
//
// マルチキャストは ACK も再送もなく、多くの AP では最低基本レートで送出される
// ためユニキャストより損失率が明確に高い。欠番率の実測は P1 の必須項目 (計画 §3.7)。
func (v *VisionReceiver) trackFrameNumber(cameraID, frameNumber uint32) (gap, lost, total int64) {
	v.mu.Lock()
	defer v.mu.Unlock()

	prev, seen := v.lastSeen[cameraID]
	v.lastSeen[cameraID] = frameNumber
	if !v.seenAny[cameraID] {
		v.seenAny[cameraID] = true
	}

	if !seen {
		return 1, v.lostFrames.Load(), 1
	}

	gap = int64(frameNumber) - int64(prev)
	switch {
	case gap > 1:
		v.lostFrames.Add(gap - 1)
	case gap <= 0:
		// 順序逆転か重複。直近の値を巻き戻さないよう prev を戻す。
		v.reordered.Add(1)
		v.lastSeen[cameraID] = prev
	}
	return gap, v.lostFrames.Load(), v.packets.Load()
}

// Sync はクロック推定器の Provider を返す。監視用。
func (v *VisionReceiver) Sync() timesync.Provider { return v.sync }

// LogStats は受信の健全性を /recorder/status とは別に記録する。
func (v *VisionReceiver) LogStats() {
	if v.rec == nil {
		return
	}
	stats := v.Stats()
	v.rec.LogJSON(loclog.ChRecorderStatus, v.clock.Now(), struct {
		Source string `json:"source"`
		VisionStats
		LossRate float64 `json:"lossRate"`
	}{Source: "vision", VisionStats: stats, LossRate: stats.LossRate()})

	// クロック推定の状態をカメラごとに残す。時刻同期は静かに壊れるので、
	// 壊れたことを検出できる仕組みを最初から入れる (計画 §5.5)。
	multi, ok := v.sync.(*timesync.MultiSync)
	if !ok {
		return
	}
	for _, id := range multi.Cameras() {
		st, ok := multi.StatsFor(id)
		if !ok {
			continue
		}
		v.rec.LogJSON(loclog.ChVisionMeta, v.clock.Now(), struct {
			Source   string  `json:"source"`
			CameraID uint32  `json:"camera_id"`
			SkewPpm  float64 `json:"skew_ppm"`
			OffsetNs int64   `json:"offset_ns"`
			SpanNs   int64   `json:"span_ns"`
			Samples  int     `json:"samples_count"`
			Fits     int64   `json:"fits_count"`
			Resets   int64   `json:"resets_count"`
			Freezes  int64   `json:"freezes_count"`
			Valid    bool    `json:"valid"`
			Frozen   bool    `json:"frozen"`
		}{"timesync", id, st.SkewPpm, st.OffsetNs, st.SpanNs,
			st.Samples, st.Fits, st.Resets, st.Freezes, st.Valid, st.Frozen})
	}
}

// RunVisionStatsLogger は一定間隔で受信統計を記録する。
func RunVisionStatsLogger(done <-chan struct{}, v *VisionReceiver, interval time.Duration) {
	if interval <= 0 {
		interval = time.Second
	}
	t := time.NewTicker(interval)
	defer t.Stop()
	for {
		select {
		case <-done:
			return
		case <-t.C:
			v.LogStats()
		}
	}
}
