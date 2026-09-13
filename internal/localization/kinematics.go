package localization

import (
	"fmt"
	"math"

	"gonum.org/v1/gonum/mat"
)

// Kinematics は 4 輪オムニの運動学。起動時に 1 度だけ構築し、以降は再利用する。
//
// 観測モデルは 4 次元のまま扱う (計画 §4.5)。擬似逆で 3 次元に潰すのは
// BodyFromWheel を明示的に呼んだときだけで、潰すと 4 輪それぞれの雑音が混ざり
// 個別の車輪異常が見えなくなるため、フィルタの観測には使わない。
type Kinematics struct {
	cfg GeometryConfig

	// m は body 速度から車輪角速度への写像。omega_wheel = m * [vx, vy, omega]。
	// 単位は [rad/s] / [m/s, m/s, rad/s]。
	m [NumWheels][3]float64

	// pinv は m の左擬似逆 (m^T m)^-1 m^T。最小二乗解を与える。
	// 起動時に 1 度だけ計算し、以降ホットパスでは配列参照だけになる。
	pinv [3][NumWheels]float64
}

// NewKinematics は機体パラメータから運動学を構築する。
func NewKinematics(cfg GeometryConfig) (*Kinematics, error) {
	if err := cfg.Validate(); err != nil {
		return nil, err
	}
	k := &Kinematics{cfg: cfg}

	for i := 0; i < NumWheels; i++ {
		a := cfg.WheelAnglesDeg[i] * math.Pi / 180
		s, c := math.Sincos(a)
		// 基準形 (旧世代 STM / RAVEN): v_i = sin(a)*vx - cos(a)*vy - R*omega [m/s]
		// これを車輪半径で割って角速度にし、符号規約を掛ける。
		inv := cfg.WheelSigns[i] / cfg.WheelRadiusM[i]
		k.m[i][0] = s * inv
		k.m[i][1] = -c * inv
		k.m[i][2] = -cfg.MomentArmM * inv
	}

	if err := k.computePseudoInverse(); err != nil {
		return nil, err
	}
	return k, nil
}

func (k *Kinematics) computePseudoInverse() error {
	m := mat.NewDense(NumWheels, 3, nil)
	for i := 0; i < NumWheels; i++ {
		for j := 0; j < 3; j++ {
			m.Set(i, j, k.m[i][j])
		}
	}

	var mtm mat.SymDense
	mtm.SymOuterK(1, m.T())

	var chol mat.Cholesky
	if ok := chol.Factorize(&mtm); !ok {
		// 取付角が縮退している (例: 全輪が平行) と起きる。設定ミスの検出。
		return fmt.Errorf("wheel configuration is degenerate: m^T m is not positive definite")
	}

	var inv mat.SymDense
	if err := chol.InverseTo(&inv); err != nil {
		return fmt.Errorf("invert m^T m: %w", err)
	}

	var pinv mat.Dense
	pinv.Mul(&inv, m.T())
	for i := 0; i < 3; i++ {
		for j := 0; j < NumWheels; j++ {
			k.pinv[i][j] = pinv.At(i, j)
		}
	}
	return nil
}

// Config は構築に使った機体パラメータを返す。
func (k *Kinematics) Config() GeometryConfig { return k.cfg }

// Row は論理輪 i の観測行 [d/dvx, d/dvy, d/domega] を返す。
// フィルタの観測ヤコビアンにそのまま入る。
func (k *Kinematics) Row(i int) [3]float64 { return k.m[i] }

// WheelFromBody は body 速度 [m/s, m/s, rad/s] から 4 輪の角速度 [rad/s] を作る。
// 返り値は論理輪番号 (FL, BL, BR, FR) の順。アロケートしない。
func (k *Kinematics) WheelFromBody(vx, vy, omega float64) [NumWheels]float64 {
	var out [NumWheels]float64
	for i := 0; i < NumWheels; i++ {
		out[i] = k.m[i][0]*vx + k.m[i][1]*vy + k.m[i][2]*omega
	}
	return out
}

// BodyFromWheel は 4 輪の角速度 [rad/s] から body 速度を最小二乗で復元する。
// 入力は論理輪番号の順。アロケートしない。
//
// 冗長性 (4 式 3 未知数) を潰すので、フィルタの観測には使わない。
// 診断・同定・初期値の生成用。
func (k *Kinematics) BodyFromWheel(w [NumWheels]float64) (vx, vy, omega float64) {
	for j := 0; j < NumWheels; j++ {
		vx += k.pinv[0][j] * w[j]
		vy += k.pinv[1][j] * w[j]
		omega += k.pinv[2][j] * w[j]
	}
	return vx, vy, omega
}

// Residual は実測の車輪角速度と、与えた body 速度から期待される値との差を返す。
//
// この残差は捨てる情報ではない (計画 §4.5)。スリップ状態の推定に効くほか、
// 特定の 1 輪だけ残差が続けばモータ・エンコーダの故障検出になる。
func (k *Kinematics) Residual(measured [NumWheels]float64, vx, vy, omega float64) [NumWheels]float64 {
	expected := k.WheelFromBody(vx, vy, omega)
	var out [NumWheels]float64
	for i := 0; i < NumWheels; i++ {
		out[i] = measured[i] - expected[i]
	}
	return out
}

// SlotsToLogical は SPI フレーム上の並びの 4 輪値を、論理輪番号の並びへ入れ替える。
// 計画 §12-A4 (FL / FR 入れ替わりの疑い) を設定だけで吸収するための経路。
func (k *Kinematics) SlotsToLogical(slots [NumWheels]float64) [NumWheels]float64 {
	var out [NumWheels]float64
	for slot := 0; slot < NumWheels; slot++ {
		out[k.cfg.WheelSlotOrder[slot]] = slots[slot]
	}
	return out
}
