package locadapter

import (
	"math"
	"testing"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

func TestExciterCoversAllThreeAxes(t *testing.T) {
	e := NewExciter(ExciteConfig{})

	var maxVX, maxVY, maxOmega float64
	var minVX, minVY, minOmega float64
	for tt := time.Duration(0); tt < e.Total(); tt += 8 * time.Millisecond {
		c := e.At(tt)
		maxVX, minVX = math.Max(maxVX, c.VX), math.Min(minVX, c.VX)
		maxVY, minVY = math.Max(maxVY, c.VY), math.Min(minVY, c.VY)
		maxOmega, minOmega = math.Max(maxOmega, c.Omega), math.Min(minOmega, c.Omega)
	}
	// 3 自由度すべてを両方向に振っていること。片方向だけだと
	// モータのデッドバンドの非対称がそのまま係数に化ける。
	if maxVX <= 0 || minVX >= 0 {
		t.Errorf("vx range [%v, %v] does not cover both directions", minVX, maxVX)
	}
	if maxVY <= 0 || minVY >= 0 {
		t.Errorf("vy range [%v, %v] does not cover both directions", minVY, maxVY)
	}
	if maxOmega <= 0 || minOmega >= 0 {
		t.Errorf("omega range [%v, %v] does not cover both directions", minOmega, maxOmega)
	}
}

// 各区間では 1 自由度だけを動かすこと。混ぜると軸ごとの寄与が分離できない。
func TestExciterMovesOneAxisAtATime(t *testing.T) {
	e := NewExciter(ExciteConfig{})
	for tt := time.Duration(0); tt < e.Total(); tt += 20 * time.Millisecond {
		c := e.At(tt)
		nonZero := 0
		for _, v := range []float64{c.VX, c.VY, c.Omega} {
			if v != 0 {
				nonZero++
			}
		}
		if nonZero > 1 {
			t.Fatalf("t=%v (%s): %d axes are active at once (%+v)", tt, c.Label, nonZero, c)
		}
	}
}

func TestExciterStartsAndEndsAtRest(t *testing.T) {
	e := NewExciter(ExciteConfig{})
	if c := e.At(-time.Second); !c.Resting || c.VX != 0 {
		t.Errorf("before the start: %+v, want rest", c)
	}
	end := e.At(e.Total() + time.Second)
	if !end.Done || end.VX != 0 || end.VY != 0 || end.Omega != 0 {
		t.Errorf("after the end: %+v, want a finished, zero command", end)
	}
	// 最後の区間は停止。指令を切った瞬間に走り去らない。
	last := e.At(e.Total() - time.Millisecond)
	if !last.Resting {
		t.Errorf("the final segment is %+v, want a rest so the robot is stopped when the sequence ends", last)
	}
}

func TestExciterFrameUnits(t *testing.T) {
	c := ExciteCommand{VX: 1.234, VY: -0.5, Omega: 3.75}
	vx, vy, va := c.ToFrameUnits()
	if vx != 1234 || vy != -500 || va != 3750 {
		t.Errorf("ToFrameUnits = (%d, %d, %d), want (1234, -500, 3750)", vx, vy, va)
	}

	// 飽和しても巻き返らないこと。
	big := ExciteCommand{VX: 100, VY: -100}
	vx, vy, _ = big.ToFrameUnits()
	if vx != 32767 || vy != -32768 {
		t.Errorf("clamping failed: (%d, %d)", vx, vy)
	}
}

// このシーケンスを実際に流したら同定が解けること。加振設計そのものの検証。
func TestExciterProducesIdentifiableData(t *testing.T) {
	want := localization.DefaultGeometry()
	want.WheelSlotOrder = [localization.NumWheels]int{
		localization.WheelFR, localization.WheelBL, localization.WheelBR, localization.WheelFL,
	}
	want.WheelSigns = [localization.NumWheels]float64{-1, -1, -1, -1}
	k, err := localization.NewKinematics(want)
	if err != nil {
		t.Fatal(err)
	}

	e := NewExciter(ExciteConfig{})
	const dt = 8 * time.Millisecond
	// モータの追従 (tau = 40 ms) と整定待ち (150 ms) を模す。
	alpha := 1 - math.Exp(-float64(dt)/float64(40*time.Millisecond))
	var actual [3]float64
	var lastSegment = -1
	var segmentStart time.Duration

	var samples []localization.IdentSample
	for tt := time.Duration(0); tt < e.Total(); tt += dt {
		c := e.At(tt)
		if c.Segment != lastSegment {
			lastSegment, segmentStart = c.Segment, tt
		}
		// SPI フレームの量子化を通す。
		vx16, vy16, va16 := c.ToFrameUnits()
		target := [3]float64{float64(vx16) / 1000, float64(vy16) / 1000, float64(va16) / 1000}
		for i := range actual {
			actual[i] += alpha * (target[i] - actual[i])
		}
		if tt-segmentStart < 150*time.Millisecond {
			continue // 整定待ち
		}

		logical := k.WheelFromBody(actual[0], actual[1], actual[2])
		var s localization.IdentSample
		s.VX, s.VY, s.Omega = target[0], target[1], target[2]
		for slot := 0; slot < localization.NumWheels; slot++ {
			v := logical[want.WheelSlotOrder[slot]]
			s.WheelSlots[slot] = math.Round(v*100) / 100 // エンコーダの量子化
		}
		samples = append(samples, s)
	}

	res, err := localization.Identify(samples, localization.RefCommand, localization.IdentOptions{})
	if err != nil {
		t.Fatalf("the excitation sequence does not yield an identifiable dataset: %v", err)
	}
	if res.Geometry.WheelSlotOrder != want.WheelSlotOrder {
		t.Errorf("wheelSlotOrder = %v, want %v", res.Geometry.WheelSlotOrder, want.WheelSlotOrder)
	}
	if res.Geometry.WheelSigns != want.WheelSigns {
		t.Errorf("wheelSigns = %v, want %v", res.Geometry.WheelSigns, want.WheelSigns)
	}
	for _, w := range res.Warnings {
		t.Logf("warning: %s", w)
	}
}
