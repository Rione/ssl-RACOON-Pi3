package locadapter

import (
	"math"
	"testing"

	"github.com/Rione/ssl-RACOON-Pi2/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi2/internal/loclog"
	"github.com/Rione/ssl-RACOON-Pi2/proto/pb_gen"
	"google.golang.org/protobuf/proto"
)

func newTestReceiver(t *testing.T, team Team, id uint32) *VisionReceiver {
	t.Helper()
	// rec = nil。記録は loclog 側でテスト済みなので、ここでは受信ロジックだけ見る。
	return NewVisionReceiver(loclog.NewClock(), nil, nil, VisionConfig{RobotID: id, Team: team})
}

func packet(cameraID, frameNumber uint32, tCapture, tSent float64, blue, yellow []*pb_gen.SSL_DetectionRobot) []byte {
	pkt := &pb_gen.SSL_WrapperPacket{
		Detection: &pb_gen.SSL_DetectionFrame{
			FrameNumber:  proto.Uint32(frameNumber),
			TCapture:     proto.Float64(tCapture),
			TSent:        proto.Float64(tSent),
			CameraId:     proto.Uint32(cameraID),
			RobotsBlue:   blue,
			RobotsYellow: yellow,
		},
	}
	data, err := proto.Marshal(pkt)
	if err != nil {
		panic(err)
	}
	return data
}

func robot(id uint32, x, y, theta, conf float32) *pb_gen.SSL_DetectionRobot {
	return &pb_gen.SSL_DetectionRobot{
		Confidence:  proto.Float32(conf),
		RobotId:     proto.Uint32(id),
		X:           proto.Float32(x),
		Y:           proto.Float32(y),
		Orientation: proto.Float32(theta),
		PixelX:      proto.Float32(0),
		PixelY:      proto.Float32(0),
	}
}

func TestVisionExtractsSelfAndConvertsToMeters(t *testing.T) {
	v := newTestReceiver(t, TeamBlue, 3)
	data := packet(0, 1, 100.0, 100.01,
		[]*pb_gen.SSL_DetectionRobot{robot(1, -100, 200, 0.5, 0.9), robot(3, 1500, -2250, 1.25, 0.95)},
		[]*pb_gen.SSL_DetectionRobot{robot(3, 9999, 9999, 0, 0.99)})
	v.handlePacket(data, 0)

	got, ok := v.Latest()
	if !ok {
		t.Fatal("no observation was produced")
	}
	// SSL-Vision は mm、コア内部は SI。ここが唯一の変換点。
	if math.Abs(got.Pose.X-1.5) > 1e-6 || math.Abs(got.Pose.Y-(-2.25)) > 1e-6 {
		t.Errorf("pose = (%v, %v) m, want (1.5, -2.25)", got.Pose.X, got.Pose.Y)
	}
	if math.Abs(got.Pose.Theta-1.25) > 1e-6 {
		t.Errorf("theta = %v, want 1.25", got.Pose.Theta)
	}
	if got.FrameNumber != 1 || got.CameraID != 0 {
		t.Errorf("frame/camera = %d/%d, want 1/0", got.FrameNumber, got.CameraID)
	}
}

// 色を取り違えると別のロボットを自機だと思い込む。そこが効いていることを固定する。
func TestVisionRespectsTeamColor(t *testing.T) {
	data := packet(0, 1, 100.0, 100.01,
		[]*pb_gen.SSL_DetectionRobot{robot(3, 1000, 0, 0, 0.9)},
		[]*pb_gen.SSL_DetectionRobot{robot(3, -1000, 0, 0, 0.9)})

	blue := newTestReceiver(t, TeamBlue, 3)
	blue.handlePacket(data, 0)
	b, ok := blue.Latest()
	if !ok || math.Abs(b.Pose.X-1.0) > 1e-6 {
		t.Errorf("blue receiver got %+v, want x = 1.0 m", b.Pose)
	}

	yellow := newTestReceiver(t, TeamYellow, 3)
	yellow.handlePacket(data, 0)
	y, ok := yellow.Latest()
	if !ok || math.Abs(y.Pose.X-(-1.0)) > 1e-6 {
		t.Errorf("yellow receiver got %+v, want x = -1.0 m", y.Pose)
	}
}

func TestVisionIgnoresFramesWithoutSelf(t *testing.T) {
	v := newTestReceiver(t, TeamBlue, 3)
	v.handlePacket(packet(0, 1, 100, 100.01,
		[]*pb_gen.SSL_DetectionRobot{robot(5, 0, 0, 0, 0.9)}, nil), 0)

	if _, ok := v.Latest(); ok {
		t.Error("an observation was produced even though the robot was not in the frame")
	}
	if s := v.Stats(); s.Packets != 1 || s.SelfSeen != 0 {
		t.Errorf("stats = %+v, want 1 packet and 0 self sightings", s)
	}
}

// 同じ ID が二重に出たら confidence の高い方を採る。
func TestVisionPrefersHigherConfidenceDuplicate(t *testing.T) {
	v := newTestReceiver(t, TeamBlue, 3)
	v.handlePacket(packet(0, 1, 100, 100.01, []*pb_gen.SSL_DetectionRobot{
		robot(3, 1000, 0, 0, 0.30),
		robot(3, 2000, 0, 0, 0.91),
		robot(3, 3000, 0, 0, 0.55),
	}, nil), 0)

	got, ok := v.Latest()
	if !ok {
		t.Fatal("no observation")
	}
	if math.Abs(got.Pose.X-2.0) > 1e-6 {
		t.Errorf("x = %v m, want 2.0 (the highest-confidence detection)", got.Pose.X)
	}
}

// frame_number の欠番でマルチキャストの損失率を測れること (P1 の必須項目)。
func TestVisionCountsLostFrames(t *testing.T) {
	v := newTestReceiver(t, TeamBlue, 3)
	// カメラ 0: 1, 2, 5 (3 と 4 が欠落), 6
	for _, fn := range []uint32{1, 2, 5, 6} {
		v.handlePacket(packet(0, fn, 100, 100.01, nil, nil), 0)
	}
	if got := v.Stats().LostFrames; got != 2 {
		t.Errorf("lostFrames = %d, want 2", got)
	}

	// カメラ 1 は独立に追う。カメラをまたいで欠番と誤判定しないこと。
	for _, fn := range []uint32{1000, 1001, 1002} {
		v.handlePacket(packet(1, fn, 100, 100.01, nil, nil), 0)
	}
	if got := v.Stats().LostFrames; got != 2 {
		t.Errorf("lostFrames = %d after a second camera, want still 2", got)
	}
	if got := v.Stats().Packets; got != 7 {
		t.Errorf("packets = %d, want 7", got)
	}
}

func TestVisionCountsReordering(t *testing.T) {
	v := newTestReceiver(t, TeamBlue, 3)
	for _, fn := range []uint32{10, 11, 9, 12} {
		v.handlePacket(packet(0, fn, 100, 100.01, nil, nil), 0)
	}
	s := v.Stats()
	if s.Reordered != 1 {
		t.Errorf("reordered = %d, want 1", s.Reordered)
	}
	// 逆転したフレームで基準を巻き戻すと、次の 12 が「+3」に見えて
	// 欠番を過大評価する。巻き戻さないこと。
	if s.LostFrames != 0 {
		t.Errorf("lostFrames = %d, want 0; a reordered packet must not be counted as loss", s.LostFrames)
	}

	// 重複も同じ扱い。
	v.handlePacket(packet(0, 12, 100, 100.01, nil, nil), 0)
	if got := v.Stats().Reordered; got != 2 {
		t.Errorf("reordered = %d after a duplicate, want 2", got)
	}
}

func TestVisionCountsBadTimestamps(t *testing.T) {
	v := newTestReceiver(t, TeamBlue, 3)
	arrival := localization.Stamp(12345)
	v.handlePacket(packet(0, 1, 0, 0, []*pb_gen.SSL_DetectionRobot{robot(3, 0, 0, 0, 0.9)}, nil), arrival)

	if got := v.Stats().BadTimestamps; got != 1 {
		t.Errorf("badTimestamps = %d, want 1", got)
	}
	got, ok := v.Latest()
	if !ok {
		t.Fatal("a bad t_capture must not discard the observation")
	}
	if got.Stamp != arrival {
		t.Errorf("stamp = %v, want the arrival time %v as fallback", got.Stamp, arrival)
	}
}

func TestVisionDecodeErrors(t *testing.T) {
	v := newTestReceiver(t, TeamBlue, 3)
	v.handlePacket([]byte{0xDE, 0xAD, 0xBE, 0xEF}, 0)
	if got := v.Stats().DecodeErrors; got != 1 {
		t.Errorf("decodeErrors = %d, want 1", got)
	}
}

// フィルタが読まなくても受信が止まらないこと (計画 §7.2)。
func TestVisionDoesNotBlockWhenConsumerStalls(t *testing.T) {
	v := newTestReceiver(t, TeamBlue, 3)
	data := packet(0, 1, 100, 100.01, []*pb_gen.SSL_DetectionRobot{robot(3, 0, 0, 0, 0.9)}, nil)
	for i := 0; i < 1000; i++ {
		v.handlePacket(data, 0)
	}
	if got := v.Stats().Dropped; got == 0 {
		t.Error("expected observations to be dropped when nothing reads the channel")
	}
	// 捨てていても最新値は必ず更新されている。
	if _, ok := v.Latest(); !ok {
		t.Error("Latest() must still be available when the channel is full")
	}
}

func TestParseTeam(t *testing.T) {
	if got, err := ParseTeam("yellow"); err != nil || got != TeamYellow {
		t.Errorf("ParseTeam(yellow) = %v, %v", got, err)
	}
	if got, err := ParseTeam("blue"); err != nil || got != TeamBlue {
		t.Errorf("ParseTeam(blue) = %v, %v", got, err)
	}
	if _, err := ParseTeam("red"); err == nil {
		t.Error("expected an error for an unknown team")
	}
}

func TestVisionLossRate(t *testing.T) {
	s := VisionStats{Packets: 90, LostFrames: 10}
	if got := s.LossRate(); math.Abs(got-0.1) > 1e-9 {
		t.Errorf("LossRate = %v, want 0.1", got)
	}
	if got := (VisionStats{}).LossRate(); got != 0 {
		t.Errorf("LossRate of an empty stat = %v, want 0", got)
	}
}
