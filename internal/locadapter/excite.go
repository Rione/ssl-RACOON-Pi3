package locadapter

import (
	"fmt"
	"time"
)

// 符号・ホイール順序・寸法の同定のための加振 (計画 §8 / §12-A)。
//
// SPI の下りは VelX / VelY / VelAng の 3 つしか持たないので、「1 輪ずつ回す」
// ことができない。代わりに 3 自由度を順に、複数の振幅で加振し、
// 4 輪の応答行列から localization.Identify がまとめて解く。
//
// 振幅を振るのは、定速 1 点だけだと各係数が 1 点でしか拘束されず、
// エンコーダの量子化とモータのデッドバンドに弱くなるため。

// ExciteConfig は加振シーケンスの設定。
type ExciteConfig struct {
	// Segment は 1 区間の長さ。既定 1.2 秒。
	Segment time.Duration
	// Rest は区間の間に挟む停止の長さ。既定 0.8 秒。
	//
	// 停止を挟むのは、次の区間の立ち上がりを前の区間から切り離すためと、
	// 停止中のデータが ZUPT / ZARU (P5) の検証にそのまま使えるため。
	Rest time.Duration
	// TransSpeed は並進の基準速度 [m/s]。既定 1.0。
	TransSpeed float64
	// YawRate は回転の基準角速度 [rad/s]。既定 4.0。
	YawRate float64
	// Amplitudes は基準速度に掛ける係数。既定 {0.4, 0.7, 1.0}。
	Amplitudes []float64
}

func (c *ExciteConfig) withDefaults() {
	if c.Segment <= 0 {
		c.Segment = 1200 * time.Millisecond
	}
	if c.Rest <= 0 {
		c.Rest = 800 * time.Millisecond
	}
	if c.TransSpeed <= 0 {
		c.TransSpeed = 1.0
	}
	if c.YawRate <= 0 {
		c.YawRate = 4.0
	}
	if len(c.Amplitudes) == 0 {
		c.Amplitudes = []float64{0.4, 0.7, 1.0}
	}
}

// ExciteCommand は 1 時点の指令。
type ExciteCommand struct {
	// VX / VY はロボット系の並進速度 [m/s]。
	VX, VY float64
	// Omega はヨーレート [rad/s]。
	Omega float64
	// Segment は現在の区間の番号。
	Segment int
	// Label は人が読むための区間名。
	Label string
	// Resting は停止区間か。
	Resting bool
	// Done はシーケンスが終わったか。真なら全部 0 を返す。
	Done bool
}

type exciteSegment struct {
	vx, vy, omega float64
	label         string
}

// Exciter は経過時間から指令を返す。時計を持たないので決定論的にテストできる。
type Exciter struct {
	cfg      ExciteConfig
	segments []exciteSegment
	total    time.Duration
}

// NewExciter は加振シーケンスを組み立てる。
func NewExciter(cfg ExciteConfig) *Exciter {
	cfg.withDefaults()
	e := &Exciter{cfg: cfg}

	axes := []struct {
		name          string
		vx, vy, omega float64
	}{
		{"+x", 1, 0, 0},
		{"-x", -1, 0, 0},
		{"+y", 0, 1, 0},
		{"-y", 0, -1, 0},
		{"+yaw", 0, 0, 1},
		{"-yaw", 0, 0, 1}, // omega の符号は下で反転する
	}

	for i, a := range axes {
		sign := 1.0
		if a.name == "-yaw" {
			sign = -1
		}
		for _, amp := range cfg.Amplitudes {
			e.segments = append(e.segments, exciteSegment{
				vx:    a.vx * amp * cfg.TransSpeed,
				vy:    a.vy * amp * cfg.TransSpeed,
				omega: a.omega * amp * cfg.YawRate * sign,
				label: fmt.Sprintf("%s x%.1f", a.name, amp),
			})
			e.segments = append(e.segments, exciteSegment{label: "rest"})
		}
		_ = i
	}

	for _, s := range e.segments {
		e.total += e.durationOf(s)
	}
	return e
}

func (e *Exciter) durationOf(s exciteSegment) time.Duration {
	if s.label == "rest" {
		return e.cfg.Rest
	}
	return e.cfg.Segment
}

// Total はシーケンス全体の長さを返す。
func (e *Exciter) Total() time.Duration { return e.total }

// Segments は区間数を返す。
func (e *Exciter) Segments() int { return len(e.segments) }

// At は開始からの経過時間に対応する指令を返す。
func (e *Exciter) At(elapsed time.Duration) ExciteCommand {
	if elapsed < 0 {
		return ExciteCommand{Label: "rest", Resting: true}
	}
	acc := time.Duration(0)
	for i, s := range e.segments {
		d := e.durationOf(s)
		if elapsed < acc+d {
			return ExciteCommand{
				VX: s.vx, VY: s.vy, Omega: s.omega,
				Segment: i,
				Label:   s.label,
				Resting: s.label == "rest",
			}
		}
		acc += d
	}
	return ExciteCommand{Segment: len(e.segments), Label: "done", Resting: true, Done: true}
}

// ToFrameUnits は SPI フレームの単位 (mm/s, mrad/s) へ直す。
func (c ExciteCommand) ToFrameUnits() (velX, velY, velAng int16) {
	return clampInt16(c.VX * 1000), clampInt16(c.VY * 1000), clampInt16(c.Omega * 1000)
}

func clampInt16(v float64) int16 {
	switch {
	case v > 32767:
		return 32767
	case v < -32768:
		return -32768
	}
	return int16(v)
}
