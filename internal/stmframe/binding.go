package stmframe

import (
	"fmt"
	"time"
)

// 上位が参照するフィールド名。プロファイルはこの名前でセンサを公開する。
const (
	FieldWheelFL = "wheelFL"
	FieldWheelBL = "wheelBL"
	FieldWheelBR = "wheelBR"
	FieldWheelFR = "wheelFR"

	FieldGyroZ  = "gyroZ"
	FieldAccelX = "accelX"
	FieldAccelY = "accelY"

	FieldBattery    = "battery"
	FieldSensorInfo = "sensorInfo"
	FieldCapPower   = "capPower"
)

// WheelSlotNames は SPI フレーム上の並び順のフィールド名。
// 出典: MainBoard_V26_1_1/src/unit/robot.c:158 (FL, BL, BR, FR)。
var WheelSlotNames = [4]string{FieldWheelFL, FieldWheelBL, FieldWheelBR, FieldWheelFR}

// Binding はプロファイルのフィールド名を、起動時に 1 度だけ位置へ解決した結果。
// ホットパスからマップ参照を追い出すために使う。
//
// 見つからなかったフィールドは -1 になり、そのセンサは非搭載として扱われる
// (計画 §7.5 の設計原則)。
type Binding struct {
	// WheelSlots は SPI フレーム上の並び順の車輪フィールド位置。
	WheelSlots [4]int
	// GyroZ / AccelX / AccelY は IMU のフィールド位置。未搭載なら -1。
	GyroZ  int
	AccelX int
	AccelY int

	// Battery / SensorInfo / CapPower は既存の機体状態。未定義なら -1。
	Battery    int
	SensorInfo int
	CapPower   int

	// WheelTimeOffset は車輪サンプルの時刻ずれ。4 輪で同一であることを要求する。
	WheelTimeOffset time.Duration
	// ImuTimeOffset は IMU サンプルの時刻ずれ。
	ImuTimeOffset time.Duration
}

// HasWheels は 4 輪すべてが定義されているかを返す。
func (b *Binding) HasWheels() bool {
	for _, i := range b.WheelSlots {
		if i < 0 {
			return false
		}
	}
	return true
}

// HasGyro はジャイロが定義されているかを返す。
func (b *Binding) HasGyro() bool { return b.GyroZ >= 0 }

// HasAccel は加速度計の 2 軸が揃っているかを返す。
func (b *Binding) HasAccel() bool { return b.AccelX >= 0 && b.AccelY >= 0 }

// Bind はデコーダのフィールド名を位置へ解決する。
func (d *Decoder) Bind() (*Binding, error) {
	b := &Binding{
		GyroZ: -1, AccelX: -1, AccelY: -1,
		Battery: -1, SensorInfo: -1, CapPower: -1,
	}
	lookup := func(name string) int {
		if i, ok := d.FieldIndex(name); ok {
			return i
		}
		return -1
	}
	for slot, name := range WheelSlotNames {
		b.WheelSlots[slot] = lookup(name)
	}
	b.GyroZ = lookup(FieldGyroZ)
	b.AccelX = lookup(FieldAccelX)
	b.AccelY = lookup(FieldAccelY)
	b.Battery = lookup(FieldBattery)
	b.SensorInfo = lookup(FieldSensorInfo)
	b.CapPower = lookup(FieldCapPower)

	if !b.HasWheels() {
		return nil, fmt.Errorf("stmframe: profile %s does not define all four wheels (%v)",
			d.profile.Name, WheelSlotNames)
	}

	// 4 輪の時刻ずれが揃っていないと、1 つの WheelSample として扱えない。
	b.WheelTimeOffset = d.TimeOffset(b.WheelSlots[0])
	for _, i := range b.WheelSlots[1:] {
		if d.TimeOffset(i) != b.WheelTimeOffset {
			return nil, fmt.Errorf("stmframe: profile %s gives the four wheels different timeOffsetMs; "+
				"they are sampled together and must share one offset", d.profile.Name)
		}
	}
	if b.HasGyro() {
		b.ImuTimeOffset = d.TimeOffset(b.GyroZ)
	} else if b.HasAccel() {
		b.ImuTimeOffset = d.TimeOffset(b.AccelX)
	}
	return b, nil
}
