package localization

import "math"

// SO(2) / SE(2) の多様体演算。
//
// 姿勢誤差を接空間で扱うことで、角度の巻き戻し処理が不要になり
// ±pi 付近の不連続が消える (計画 §4.2)。

// WrapAngle は角度を (-pi, pi] に正規化する。
func WrapAngle(a float64) float64 {
	if a > -math.Pi && a <= math.Pi {
		return a // 大半のケースで三角関数も除算も踏まない
	}
	a = math.Mod(a+math.Pi, 2*math.Pi)
	if a <= 0 {
		a += 2 * math.Pi
	}
	return a - math.Pi
}

// AngleDiff は a - b を (-pi, pi] に正規化して返す。
func AngleDiff(a, b float64) float64 { return WrapAngle(a - b) }

// Rotate は v をワールド系へ theta だけ回した結果を返す (R(theta)*v)。
func Rotate(theta float64, v Vec2) Vec2 {
	s, c := math.Sincos(theta)
	return Vec2{X: c*v.X - s*v.Y, Y: s*v.X + c*v.Y}
}

// RotateInv は v をロボット系へ戻す (R(theta)^T * v)。
func RotateInv(theta float64, v Vec2) Vec2 {
	s, c := math.Sincos(theta)
	return Vec2{X: c*v.X + s*v.Y, Y: -s*v.X + c*v.Y}
}

// se2Eps は exp / log の級数展開へ切り替える閾値。
// theta がこれより小さいと (1-cos)/theta などが桁落ちする。
const se2Eps = 1e-8

// ExpSE2 は se(2) の接ベクトル xi = [omega, vx, vy] を SE(2) の要素へ写す。
//
// omega は回転量 [rad]、(vx, vy) は「回転しながら進んだ」並進 [m]。
// 定速旋回する車輪オドメトリの 1 ステップ積分がちょうどこの形になるので、
// 前進差分より高次の項まで正しく積める。
func ExpSE2(omega, vx, vy float64) Pose2 {
	var a, b float64 // V 行列の成分: V = [[a, -b], [b, a]]
	if math.Abs(omega) < se2Eps {
		// sin(w)/w  および  (1-cos(w))/w  のテイラー展開
		a = 1 - omega*omega/6
		b = omega/2 - omega*omega*omega/24
	} else {
		s, c := math.Sincos(omega)
		a = s / omega
		b = (1 - c) / omega
	}
	return Pose2{
		X:     a*vx - b*vy,
		Y:     b*vx + a*vy,
		Theta: WrapAngle(omega),
	}
}

// LogSE2 は ExpSE2 の逆写像。Pose2 を接ベクトル [omega, vx, vy] へ戻す。
func LogSE2(p Pose2) (omega, vx, vy float64) {
	omega = WrapAngle(p.Theta)
	// V^-1 = [[a, b], [-b, a]] で a = (w/2)*cot(w/2), b = w/2。
	//
	// 教科書通りの a = w*sin(w)/(2*(1-cos(w))) は使わない。分母が w^2 の速さで
	// 0 に近づくので、w = 1e-6 程度で既に桁落ちして有効数字が 4 桁まで落ちる。
	// 半角に直した cot 形はその打ち消しが起きない。
	var a, b float64
	half := omega / 2
	if math.Abs(half) < se2Eps {
		a = 1 - omega*omega/12 // x*cot(x) = 1 - x^2/3 - ... (x = w/2)
	} else {
		a = half / math.Tan(half)
	}
	b = half
	vx = a*p.X + b*p.Y
	vy = -b*p.X + a*p.Y
	return omega, vx, vy
}

// Compose は姿勢 a に相対姿勢 b を右から合成する (a ⊕ b)。
// b は a のロボット系で表された相対移動。
func Compose(a, b Pose2) Pose2 {
	d := Rotate(a.Theta, Vec2{X: b.X, Y: b.Y})
	return Pose2{
		X:     a.X + d.X,
		Y:     a.Y + d.Y,
		Theta: WrapAngle(a.Theta + b.Theta),
	}
}

// Between は a から見た b の相対姿勢を返す (a^-1 ⊕ b)。
func Between(a, b Pose2) Pose2 {
	d := RotateInv(a.Theta, Vec2{X: b.X - a.X, Y: b.Y - a.Y})
	return Pose2{
		X:     d.X,
		Y:     d.Y,
		Theta: AngleDiff(b.Theta, a.Theta),
	}
}
