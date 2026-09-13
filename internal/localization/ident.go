package localization

import (
	"fmt"
	"math"
	"sort"

	"gonum.org/v1/gonum/mat"
)

// 機体パラメータの同定 (計画 §8 / §12-A)。
//
// SPI の下りには車輪個別の指令が無い (VelX / VelY / VelAng の 3 つだけ) ため、
// 「1 輪ずつ既知方向に回す」ことができない。代わりに 3 自由度を順に加振し、
// 4 輪の応答行列から符号・ホイール順序・寸法をまとめて解く。
//
// 各スロット j について、基準となる body 速度との線形関係を最小二乗で求める:
//
//	omega_j = a_j*vx + b_j*vy + c_j*omega
//
// 運動学の基準形 omega_i = s*(sin(A)*vx - cos(A)*vy - R*omega)/r と見比べると
//
//	(a, b) = s/r * ( sin(A), -cos(A) )   ->  |(a,b)| = 1/r,  atan2(a, -b) = A or A+pi
//	c      = -s*R/r                      ->  R = -c*r*s
//
// つまり車輪半径・取付角・符号・モーメントアームがすべて同時に出る。
// どの取付角に最も近いかでスロットと論理輪の対応も決まる。

// IdentSample は同定の 1 サンプル。
type IdentSample struct {
	// 基準となる body 速度 [m/s, m/s, rad/s]。
	VX, VY, Omega float64
	// WheelSlots は SPI フレーム上の並びの実測角速度 [rad/s]。
	WheelSlots [NumWheels]float64
}

// IdentReference は基準速度に何を使ったか。結果の読み方が変わるので必ず残す。
type IdentReference string

const (
	// RefCommand は RAVEN から来た速度指令を基準にする。
	//
	// 実機があればすぐ実行できるが、同定されるのは「STM 自身の逆運動学と
	// エンコーダ経路の整合性」であって、絶対的な機体寸法ではない。
	// STM 側の運動学パラメータが間違っていれば、その誤差ごと吸収してしまう。
	//
	// 符号とホイール順序 (§12-A4 / A-5) の確定にはこれで十分。
	// 寸法 (§12-A1..A3) の絶対値を出すには RefVision が要る。
	RefCommand IdentReference = "command"

	// RefVision は SSL-Vision から復元した body 速度を基準にする。
	// 絶対寸法まで同定できるが、vision の微分なので雑音と遅延に弱い。
	RefVision IdentReference = "vision"
)

// SlotResult は 1 スロットの同定結果。
type SlotResult struct {
	// Slot は SPI フレーム上の位置 (0..3)。
	Slot int
	// A, B, C は最小二乗で求めた係数。
	A, B, C float64

	// Wheel は割り当てられた論理輪番号 (WheelFL など)。
	Wheel int
	// Sign は符号規約 (+1 / -1)。
	Sign float64
	// AngleDeg は実測から出た取付角 [deg]。
	AngleDeg float64
	// AngleErrorDeg は最も近い候補角からのずれ [deg]。
	AngleErrorDeg float64
	// RadiusM は実測から出た車輪半径 [m]。
	RadiusM float64
	// MomentArmM は実測から出たモーメントアーム [m]。
	MomentArmM float64

	// RMS は当てはめの残差 RMS [rad/s]。
	RMS float64
	// R2 は決定係数。1 に近いほど線形モデルで説明できている。
	R2 float64
}

// IdentResult は同定全体の結果。
type IdentResult struct {
	Reference IdentReference
	Samples   int

	Slots [NumWheels]SlotResult

	// Geometry は同定結果から組み立てた機体パラメータ。
	Geometry GeometryConfig

	// Excitation は 3 自由度それぞれの加振の大きさ (基準速度の RMS)。
	// どれかが小さいと、その軸の係数が決まらない。
	ExcitationVX, ExcitationVY, ExcitationOmega float64
	// ConditionNumber は正規方程式の条件数。大きいほど解が不安定。
	ConditionNumber float64

	// Warnings は結果を鵜呑みにしてはいけない理由。
	Warnings []string
}

// CandidateAnglesDeg は取付角の候補。§3.3 の 2 説を両方入れてある。
var CandidateAnglesDeg = []struct {
	Wheel int
	Deg   float64
}{
	{WheelFL, 55}, {WheelBL, 135}, {WheelBR, -135}, {WheelFR, -55},
	{WheelFL, 60}, {WheelFR, -60},
}

// IdentOptions は同定の設定。
type IdentOptions struct {
	// MinSamples はこれ未満なら同定しない。0 なら 200。
	MinSamples int
	// MinExcitation は各軸の加振の下限。0 なら vx/vy は 0.05 m/s、omega は 0.2 rad/s。
	MinExcitationTrans float64
	MinExcitationOmega float64
	// MaxConditionNumber を超えたら警告する。0 なら 100。
	MaxConditionNumber float64
	// MaxAngleErrorDeg を超えたら警告する。0 なら 10 deg。
	MaxAngleErrorDeg float64
}

func (o *IdentOptions) withDefaults() {
	if o.MinSamples <= 0 {
		o.MinSamples = 200
	}
	if o.MinExcitationTrans <= 0 {
		o.MinExcitationTrans = 0.05
	}
	if o.MinExcitationOmega <= 0 {
		o.MinExcitationOmega = 0.2
	}
	if o.MaxConditionNumber <= 0 {
		o.MaxConditionNumber = 100
	}
	if o.MaxAngleErrorDeg <= 0 {
		o.MaxAngleErrorDeg = 10
	}
}

// Identify は加振ログから機体パラメータを同定する。
func Identify(samples []IdentSample, ref IdentReference, opts IdentOptions) (IdentResult, error) {
	opts.withDefaults()

	res := IdentResult{Reference: ref, Samples: len(samples)}
	if len(samples) < opts.MinSamples {
		return res, fmt.Errorf("identify: %d samples is below the minimum of %d", len(samples), opts.MinSamples)
	}

	// 加振の大きさ。どれかが小さいとその軸の係数が決まらない。
	var sx, sy, sw float64
	for _, s := range samples {
		sx += s.VX * s.VX
		sy += s.VY * s.VY
		sw += s.Omega * s.Omega
	}
	n := float64(len(samples))
	res.ExcitationVX = math.Sqrt(sx / n)
	res.ExcitationVY = math.Sqrt(sy / n)
	res.ExcitationOmega = math.Sqrt(sw / n)

	if res.ExcitationVX < opts.MinExcitationTrans {
		res.Warnings = append(res.Warnings, fmt.Sprintf(
			"vx excitation is only %.3f m/s; the vx column of the response matrix is poorly determined", res.ExcitationVX))
	}
	if res.ExcitationVY < opts.MinExcitationTrans {
		res.Warnings = append(res.Warnings, fmt.Sprintf(
			"vy excitation is only %.3f m/s; the vy column of the response matrix is poorly determined", res.ExcitationVY))
	}
	if res.ExcitationOmega < opts.MinExcitationOmega {
		res.Warnings = append(res.Warnings, fmt.Sprintf(
			"omega excitation is only %.3f rad/s; the moment arm cannot be identified", res.ExcitationOmega))
	}

	// 正規方程式 (3x3) を 1 度だけ作り、4 スロットで使い回す。
	var ata [3][3]float64
	for _, s := range samples {
		u := [3]float64{s.VX, s.VY, s.Omega}
		for i := 0; i < 3; i++ {
			for j := 0; j < 3; j++ {
				ata[i][j] += u[i] * u[j]
			}
		}
	}
	normal := mat.NewSymDense(3, []float64{
		ata[0][0], ata[0][1], ata[0][2],
		ata[0][1], ata[1][1], ata[1][2],
		ata[0][2], ata[1][2], ata[2][2],
	})

	var chol mat.Cholesky
	if ok := chol.Factorize(normal); !ok {
		return res, fmt.Errorf("identify: the excitation does not span all three degrees of freedom; " +
			"drive vx, vy and omega separately before solving")
	}
	res.ConditionNumber = chol.Cond()
	if res.ConditionNumber > opts.MaxConditionNumber {
		res.Warnings = append(res.Warnings, fmt.Sprintf(
			"condition number %.1f is high; the three excitations are not independent enough", res.ConditionNumber))
	}

	for slot := 0; slot < NumWheels; slot++ {
		sr, err := solveSlot(slot, samples, &chol)
		if err != nil {
			return res, err
		}
		res.Slots[slot] = sr
	}

	assignWheels(&res, opts)
	res.Geometry = buildGeometry(&res)

	if err := res.Geometry.Validate(); err != nil {
		res.Warnings = append(res.Warnings, "identified geometry is not usable: "+err.Error())
	}
	return res, nil
}

func solveSlot(slot int, samples []IdentSample, chol *mat.Cholesky) (SlotResult, error) {
	var atb [3]float64
	var sumY, sumY2 float64
	for _, s := range samples {
		y := s.WheelSlots[slot]
		atb[0] += s.VX * y
		atb[1] += s.VY * y
		atb[2] += s.Omega * y
		sumY += y
		sumY2 += y * y
	}

	var coef mat.VecDense
	if err := chol.SolveVecTo(&coef, mat.NewVecDense(3, atb[:])); err != nil {
		return SlotResult{}, fmt.Errorf("identify: solve slot %d: %w", slot, err)
	}
	sr := SlotResult{Slot: slot, A: coef.AtVec(0), B: coef.AtVec(1), C: coef.AtVec(2)}

	// 残差と決定係数。
	n := float64(len(samples))
	meanY := sumY / n
	var ssRes, ssTot float64
	for _, s := range samples {
		pred := sr.A*s.VX + sr.B*s.VY + sr.C*s.Omega
		d := s.WheelSlots[slot] - pred
		ssRes += d * d
		dm := s.WheelSlots[slot] - meanY
		ssTot += dm * dm
	}
	sr.RMS = math.Sqrt(ssRes / n)
	if ssTot > 0 {
		sr.R2 = 1 - ssRes/ssTot
	}

	// (a, b) = s/r * (sin A, -cos A)
	norm := math.Hypot(sr.A, sr.B)
	if norm > 0 {
		sr.RadiusM = 1 / norm
	}
	// atan2(a, -b) は s = +1 のとき A、s = -1 のとき A + pi。
	sr.AngleDeg = math.Atan2(sr.A, -sr.B) * 180 / math.Pi
	return sr, nil
}

// assignWheels は各スロットを、最も近い候補角の論理輪へ割り当てる。
func assignWheels(res *IdentResult, opts IdentOptions) {
	type fit struct {
		slot, wheel int
		sign        float64
		errDeg      float64
		angleDeg    float64
	}
	var fits []fit
	for slot := 0; slot < NumWheels; slot++ {
		phi := res.Slots[slot].AngleDeg
		for _, c := range CandidateAnglesDeg {
			for _, sign := range []float64{1, -1} {
				// sign = -1 なら測定角は候補角 + 180 度にずれる。
				want := c.Deg
				if sign < 0 {
					want = c.Deg + 180
				}
				e := math.Abs(wrapDeg(phi - want))
				fits = append(fits, fit{slot, c.Wheel, sign, e, c.Deg})
			}
		}
	}
	// 誤差の小さい組から貪欲に確定する。1 スロット 1 論理輪。
	sort.Slice(fits, func(i, j int) bool { return fits[i].errDeg < fits[j].errDeg })

	usedSlot := [NumWheels]bool{}
	usedWheel := [NumWheels]bool{}
	assigned := 0
	for _, f := range fits {
		if assigned == NumWheels {
			break
		}
		if usedSlot[f.slot] || usedWheel[f.wheel] {
			continue
		}
		usedSlot[f.slot] = true
		usedWheel[f.wheel] = true
		assigned++

		sr := &res.Slots[f.slot]
		sr.Wheel = f.wheel
		sr.Sign = f.sign
		sr.AngleErrorDeg = f.errDeg
		// R = -c * r * s
		sr.MomentArmM = -sr.C * sr.RadiusM * f.sign

		if f.errDeg > opts.MaxAngleErrorDeg {
			res.Warnings = append(res.Warnings, fmt.Sprintf(
				"slot %d measured %.1f deg, %.1f deg away from the nearest candidate; the wheel assignment is a guess",
				f.slot, sr.AngleDeg, f.errDeg))
		}
		if sr.MomentArmM <= 0 {
			res.Warnings = append(res.Warnings, fmt.Sprintf(
				"slot %d gives a non-positive moment arm (%.4f m); the omega excitation or the sign is wrong",
				f.slot, sr.MomentArmM))
		}
	}
}

func buildGeometry(res *IdentResult) GeometryConfig {
	g := DefaultGeometry()
	var armSum float64
	var armCount int
	for slot := 0; slot < NumWheels; slot++ {
		sr := res.Slots[slot]
		g.WheelSlotOrder[slot] = sr.Wheel
		g.WheelSigns[sr.Wheel] = sr.Sign
		if sr.RadiusM > 0 {
			g.WheelRadiusM[sr.Wheel] = sr.RadiusM
		}
		// 取付角は実測値そのものではなく、符号を取り除いた値を使う。
		angle := sr.AngleDeg
		if sr.Sign < 0 {
			angle = wrapDeg(angle - 180)
		}
		g.WheelAnglesDeg[sr.Wheel] = angle
		if sr.MomentArmM > 0 {
			armSum += sr.MomentArmM
			armCount++
		}
	}
	if armCount > 0 {
		g.MomentArmM = armSum / float64(armCount)
	}
	return g
}

// wrapDeg は角度 [deg] を (-180, 180] へ正規化する。
func wrapDeg(d float64) float64 {
	return WrapAngle(d*math.Pi/180) * 180 / math.Pi
}

// Summary は人が読む 1 行要約を返す。
func (r SlotResult) Summary() string {
	return fmt.Sprintf(
		"slot %d -> %s  sign %+.0f  angle %+7.2f deg (err %.2f)  r %.2f mm  arm %.2f mm  R2 %.4f  rms %.4f rad/s",
		r.Slot, wheelName(r.Wheel), r.Sign, r.AngleDeg, r.AngleErrorDeg,
		r.RadiusM*1000, r.MomentArmM*1000, r.R2, r.RMS)
}

func wheelName(i int) string {
	switch i {
	case WheelFL:
		return "FL"
	case WheelBL:
		return "BL"
	case WheelBR:
		return "BR"
	case WheelFR:
		return "FR"
	}
	return "??"
}
