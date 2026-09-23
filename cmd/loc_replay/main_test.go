package main

import (
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi3/internal/loclog"
	"github.com/Rione/ssl-RACOON-Pi3/internal/locsim"
)

// 合成したセンサ列を実機と同じ形式の MCAP へ書き、それをリプレイできること。
//
// **ログの書式と読み口が食い違っていないか**を固定するためのテスト。
// 実機のログが手に入ってから気づくのでは遅い。
func TestReplayRoundTrip(t *testing.T) {
	cfg := locsim.DefaultConfig()
	tr := locsim.FigureEight(localization.Vec2{}, 1.2, 4*time.Second,
		locsim.HeadingParams{Mode: locsim.HeadingSpin, SpinRate: 2.0}, 8*time.Second)
	s, err := locsim.Generate(tr, cfg, 5)
	if err != nil {
		t.Fatal(err)
	}

	path := filepath.Join(t.TempDir(), "replay-test.mcap")
	writeLog(t, path, s)

	// -compare まで含めて通ること。出力の中身は目視で使うものなので、
	// ここでは「エラーにならない」ことだけを固定する。
	if err := run(path, "", "auto", 135, true); err != nil {
		t.Fatalf("run: %v", err)
	}
}

func writeLog(t *testing.T, path string, s *locsim.Sensors) {
	t.Helper()
	clock := loclog.NewClock()
	w, err := loclog.NewWriter(clock, loclog.Options{Path: path, Compression: "none"})
	if err != nil {
		t.Fatalf("NewWriter: %v", err)
	}

	for _, ws := range s.Wheels {
		w.LogWheel(ws.Stamp, loclog.WheelRecord{
			SampleNs:    int64(ws.Stamp),
			TransferNs:  int64(ws.Stamp),
			WheelFLRadS: ws.Omega[0],
			WheelBLRadS: ws.Omega[1],
			WheelBRRadS: ws.Omega[2],
			WheelFRRadS: ws.Omega[3],
			BatteryV:    23.4,
		})
	}
	for _, v := range s.Vision {
		w.LogVisionMeta(v.Arrival, loclog.VisionMetaRecord{
			CameraID:     v.CameraID,
			FrameNumber:  v.FrameNumber,
			TCaptureS:    v.TCapture,
			TSentS:       v.TSent,
			RecvNs:       int64(v.Arrival),
			MappedNs:     int64(v.Stamp),
			Mapped:       true,
			FrameGap:     1,
			SelfSeen:     true,
			SelfXMm:      v.Pose.X * 1000,
			SelfYMm:      v.Pose.Y * 1000,
			SelfThetaRad: v.Pose.Theta,
		})
	}
	if err := w.Close(); err != nil {
		t.Fatalf("Close: %v", err)
	}
	if st, err := os.Stat(path); err != nil || st.Size() == 0 {
		t.Fatalf("log was not written: %v", err)
	}
}
