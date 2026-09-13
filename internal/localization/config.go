package localization

import (
	"encoding/json"
	"fmt"
	"io"
	"math"
	"os"
)

// GeometryConfig は 4 輪オムニの機体パラメータ。
//
// ここの値はすべて未確定である (計画 §3.3 / §12-A)。車輪半径・モーメントアーム・
// 取付角が 4 つのコードベースで食い違っており、符号規約も世代で反転している。
// ハードコードせず必ず設定ファイルから読むのは、P1 の同定結果で差し替えられる
// ようにするため。
//
//	| パラメータ       | 旧世代 STM | 新世代 STM | RAVEN | 本リポジトリ |
//	| 車輪半径         | 27 mm      | 30 mm      | 26 mm | 30 mm        |
//	| モーメントアーム | 85 mm      | 75 mm      | 90 mm | --           |
//	| 取付角           | 55/135/-135/-55 | 同左  | 60/135/-135/-60 | -- |
type GeometryConfig struct {
	// WheelAnglesDeg は各車輪の取付角 [deg]。ワールドではなくロボット系で、
	// +x (前方) から反時計回り。並びは論理輪番号 (FL, BL, BR, FR) の順。
	WheelAnglesDeg [NumWheels]float64 `json:"wheelAnglesDeg"`

	// WheelRadiusM は車輪半径 [m]。個体差の較正 (§8) のために輪ごとに持つ。
	WheelRadiusM [NumWheels]float64 `json:"wheelRadiusM"`

	// MomentArmM は機体中心から車輪接地点までの距離 [m]。回転の腕の長さ。
	MomentArmM float64 `json:"momentArmM"`

	// WheelSlotOrder は「SPI フレーム上の並び」から「論理輪番号」への写像。
	// WheelSlotOrder[slot] = 論理輪番号。
	//
	// 計画 §12-A4 の通り、世代間で FL と FR が入れ替わっている疑いがある。
	// 恒等写像 [0,1,2,3] が既定だが、P1 の同定で確定させること。
	WheelSlotOrder [NumWheels]int `json:"wheelSlotOrder"`

	// WheelSigns は各車輪の符号 [+1 or -1]。
	//
	// 計画 §3.4 の通り、基準形は旧世代 STM と RAVEN が一致する
	//
	//	v_i = sin(a_i)*vx - cos(a_i)*vy - R*omega
	//
	// で、新世代 STM (omni_drive.c) だけが完全に符号反転している。
	// どちらが実機の「エンコーダが返す値」と整合するかは未検証であり、
	// これが「RAVEN の EKF は効果が測れなかった」の最有力の原因候補である。
	// P1 で指令と実測を突き合わせて確定させること。
	WheelSigns [NumWheels]float64 `json:"wheelSigns"`
}

// DefaultGeometry は暫定の機体パラメータを返す。
//
// 実機で動いている新世代ファームウェア (MainBoard_V26_1_1) の parammeter.h に
// 合わせてあるが、あくまで 4 候補のうちの 1 つにすぎない。符号は計画 §4.5 の
// 基準形 (旧世代 STM / RAVEN と同じ) を既定にしてある。
//
// この値のまま §10 の精度目標を評価してはならない。
func DefaultGeometry() GeometryConfig {
	const r = 0.030 // m。候補: 26 / 27 / 30 mm
	return GeometryConfig{
		WheelAnglesDeg: [NumWheels]float64{55, 135, -135, -55},
		WheelRadiusM:   [NumWheels]float64{r, r, r, r},
		MomentArmM:     0.075, // 候補: 75 / 85 / 89 / 90 mm
		WheelSlotOrder: [NumWheels]int{WheelFL, WheelBL, WheelBR, WheelFR},
		WheelSigns:     [NumWheels]float64{1, 1, 1, 1},
	}
}

// Validate は設定が運動学として成立するかを調べる。
func (g *GeometryConfig) Validate() error {
	if g.MomentArmM <= 0 || math.IsNaN(g.MomentArmM) {
		return fmt.Errorf("momentArmM must be positive, got %v", g.MomentArmM)
	}
	var seen [NumWheels]bool
	for i := 0; i < NumWheels; i++ {
		if g.WheelRadiusM[i] <= 0 || math.IsNaN(g.WheelRadiusM[i]) {
			return fmt.Errorf("wheelRadiusM[%d] must be positive, got %v", i, g.WheelRadiusM[i])
		}
		if g.WheelSigns[i] != 1 && g.WheelSigns[i] != -1 {
			return fmt.Errorf("wheelSigns[%d] must be +1 or -1, got %v", i, g.WheelSigns[i])
		}
		if math.IsNaN(g.WheelAnglesDeg[i]) {
			return fmt.Errorf("wheelAnglesDeg[%d] is NaN", i)
		}
		slot := g.WheelSlotOrder[i]
		if slot < 0 || slot >= NumWheels {
			return fmt.Errorf("wheelSlotOrder[%d] = %d is out of range", i, slot)
		}
		if seen[slot] {
			return fmt.Errorf("wheelSlotOrder maps two slots to wheel %d", slot)
		}
		seen[slot] = true
	}
	return nil
}

// LoadGeometry は JSON から機体パラメータを読む。
// 未指定のフィールドは DefaultGeometry の値が残る。
func LoadGeometry(r io.Reader) (GeometryConfig, error) {
	g := DefaultGeometry()
	dec := json.NewDecoder(r)
	dec.DisallowUnknownFields()
	if err := dec.Decode(&g); err != nil {
		return GeometryConfig{}, fmt.Errorf("decode geometry: %w", err)
	}
	if err := g.Validate(); err != nil {
		return GeometryConfig{}, err
	}
	return g, nil
}

// LoadGeometryFile は path の JSON から機体パラメータを読む。
// path が空なら DefaultGeometry を返す。
func LoadGeometryFile(path string) (GeometryConfig, error) {
	if path == "" {
		return DefaultGeometry(), nil
	}
	f, err := os.Open(path)
	if err != nil {
		return GeometryConfig{}, fmt.Errorf("open geometry %s: %w", path, err)
	}
	defer f.Close()
	return LoadGeometry(f)
}
