//go:build pi4 || rock5a

package app

import (
	"log"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/link"
	"github.com/Rione/ssl-RACOON-Pi3/internal/locadapter"
	"github.com/Rione/ssl-RACOON-Pi3/internal/loclog"
	"github.com/Rione/ssl-RACOON-Pi3/internal/state"
	"github.com/Rione/ssl-RACOON-Pi3/internal/timesync"
)

// startLocalization は自己位置推定の計測基盤 (計画 P1) を起動する。
//
// -loclog が指定されていなければ何もしない。既存の走行機能は
// フックが登録されていない状態と完全に同じ動きになる。
//
// P1 で取りたいのは次の 3 つ (計画 §11):
//
//	(a) 指令 vs 実測による符号・ホイール順序・速度スケールの確定
//	(b) frame_number によるマルチキャスト欠落率
//	(c) SPI 周期のジッタとレイテンシの内訳
//
// 生の 20 バイトをそのまま記録しておけば、STM のバイト配置が確定した後で
// 過去ログを再デコードできる。仕様確定を待たずに収集を始められる (計画 §6.1)。
func startLocalization(done <-chan struct{}, myID uint32) func() {
	if state.LocLogDir == "" {
		return func() {}
	}

	clock := loclog.NewClock()
	path := loclog.DefaultPath(state.LocLogDir, myID)

	writer, err := loclog.NewWriter(clock, loclog.Options{
		Path:       path,
		StmProfile: state.LocProfile,
		Metadata: map[string]string{
			"robot_id":    itoa(int(myID)),
			"version":     state.Version,
			"mac_address": state.MACAddress,
			"team":        state.LocTeam,
		},
	})
	if err != nil {
		// 記録が始められなくても走行機能は落とさない。
		log.Printf("[LOC] recording disabled: %v", err)
		return func() {}
	}
	log.Printf("[LOC] recording to %s", path)

	recorder, err := locadapter.NewSPIRecorder(clock, writer, state.LocProfile)
	if err != nil {
		log.Printf("[LOC] SPI recording disabled: %v", err)
	} else {
		log.Printf("[LOC] SPI %s", recorder.DescribeProfile())
		link.SetSPIObserver(recorder)
	}

	startVision(done, clock, writer, myID)
	startIdentification(clock, writer)

	return func() {
		link.SetSPIObserver(nil)
		link.SetVelocityOverride(nil)
		if err := writer.Close(); err != nil {
			log.Printf("[LOC] closing the recording failed: %v", err)
			return
		}
		s := writer.Stats()
		log.Printf("[LOC] recording closed: %s (%d written, %d dropped, %d errors, %d bytes)",
			path, s.Written, s.Dropped, s.WriteErrors, s.BytesWritten)
	}
}

func startVision(done <-chan struct{}, clock *loclog.Clock, writer *loclog.Writer, myID uint32) {
	team, err := locadapter.ParseTeam(state.LocTeam)
	if err != nil {
		// 色を取り違えると別のロボットを自機だと思い込む。黙って続けない。
		log.Printf("[LOC] vision disabled: %v", err)
		return
	}

	// 凸包法によるクロック推定を camera_id ごとに持つ (計画 §5.3)。
	// 推定が立つまでは到着時刻へフォールバックし、vision_meta の mapped で
	// どちらを使ったか分かるようにしてある。
	sync := timesync.NewConvexHullProvider(timesync.Config{})

	receiver := locadapter.NewVisionReceiver(clock, sync, writer, locadapter.VisionConfig{
		Address:   state.LocVisionAddr,
		Interface: state.LocVisionIface,
		RobotID:   myID,
		Team:      team,
	})
	go func() {
		if err := receiver.Run(done); err != nil {
			log.Printf("[LOC] vision receiver stopped: %v", err)
		}
	}()
	go locadapter.RunVisionStatsLogger(done, receiver, time.Second)
}

func startIdentification(clock *loclog.Clock, writer *loclog.Writer) {
	if !state.LocIdent {
		return
	}
	driver := locadapter.NewIdentDriver(clock, writer, locadapter.ExciteConfig{})
	log.Printf("[LOC] *** IDENTIFICATION MODE: the robot will drive itself for %v ***", driver.Total())
	log.Printf("[LOC] *** clear the area. the emergency stop still works. ***")
	link.SetVelocityOverride(driver)
}

func itoa(v int) string {
	if v == 0 {
		return "0"
	}
	neg := v < 0
	if neg {
		v = -v
	}
	var buf [20]byte
	i := len(buf)
	for v > 0 {
		i--
		buf[i] = byte('0' + v%10)
		v /= 10
	}
	if neg {
		i--
		buf[i] = '-'
	}
	return string(buf[i:])
}
