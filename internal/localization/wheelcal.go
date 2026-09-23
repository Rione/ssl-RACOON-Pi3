package localization

import (
	"fmt"
	"math"

	"gonum.org/v1/gonum/mat"
)

// 車輪のログだけで幾何を較正する (docs/self-localization-research-20260923.md §2.4, §2.5)。
//
// 滑りが無ければ n^T w = 0 が厳密に成り立つので、多数のサンプル w_t に対して
//
//	minimize  sum_t (n^T w_t)^2 = n^T W n,   W = sum_t w_t w_t^T
//
// を ||n|| = 1 の下で解けばよい。**答えは W の最小固有ベクトルで、閉形式である。**
// 格子探索も反復も要らない。
//
// **等方な車輪雑音はこの推定を偏らせない。** 雑音が独立で分散 sigma^2 なら
// W = W_true + N*sigma^2*I となり、**固有ベクトルは変わらず固有値が一様にずれるだけ**である。
// さらに最小固有値そのものが N*sigma^2 に収束するので、
// **車輪雑音の標準偏差が同時に実測できる** (docs 研究 §2.4 の 3 番目の使い道)。
//
// vision も時刻合わせも速度推定も使わないので、遅延・微分雑音・時刻同期の誤差が
// 一切入らない。PoC の vision 基準の同定 (条件数 14) より素性が良い。

// WheelNullFit は車輪ログから求めた左零ベクトル。
type WheelNullFit struct {
	// Samples は使ったサンプル数。
	Samples int
	// N は推定した左零ベクトル (論理輪番号の順、最大成分の絶対値が 1)。
	N [NumWheels]float64
	// Eigenvalues は W/Samples の固有値 (昇順)。
	Eigenvalues [NumWheels]float64
	// ResidualRMS は正規化した n (ノルム 1) での残差 RMS [rad/s]。
	// = sqrt(最小固有値)。滑っていなければ車輪雑音そのものになる。
	ResidualRMS float64
	// Identifiability は 2 番目に小さい固有値 / 最小固有値。
	//
	// **これが小さい (5 未満) と零方向が決まっていない。** 加振が足りない
	// (例: 前進しかしていない) と起きる。前進・横・旋回を混ぜること。
	Identifiability float64
	// MeasuredWheelNoise は最小固有値から出した 1 輪あたりの雑音 [rad/s]。
	// 滑りと幾何誤差が無ければ ResidualRMS と一致する。
	MeasuredWheelNoise float64
}

// FitNullVector は車輪ログ (論理輪番号の順) から左零ベクトルを推定する。
//
// samples は 1 周期ぶんの 4 輪角速度 [rad/s] の並び。滑っている周期を
// あらかじめ除いておくと精度が上がるが、必須ではない (最小二乗なので
// 外れ値に引っ張られる点には注意)。
func FitNullVector(samples [][NumWheels]float64) (WheelNullFit, error) {
	if len(samples) < 20 {
		return WheelNullFit{}, fmt.Errorf("fit null vector: need at least 20 samples, got %d", len(samples))
	}

	data := make([]float64, NumWheels*NumWheels)
	for _, w := range samples {
		for i := 0; i < NumWheels; i++ {
			for j := 0; j < NumWheels; j++ {
				data[i*NumWheels+j] += w[i] * w[j]
			}
		}
	}
	n := float64(len(samples))
	for i := range data {
		data[i] /= n
	}
	sym := mat.NewSymDense(NumWheels, data)

	var eig mat.EigenSym
	if ok := eig.Factorize(sym, true); !ok {
		return WheelNullFit{}, fmt.Errorf("fit null vector: eigen decomposition failed")
	}
	vals := eig.Values(nil) // 昇順
	var vecs mat.Dense
	eig.VectorsTo(&vecs)

	out := WheelNullFit{Samples: len(samples)}
	for i := 0; i < NumWheels; i++ {
		out.Eigenvalues[i] = vals[i]
	}
	if vals[0] < 0 {
		vals[0] = 0
	}
	out.ResidualRMS = math.Sqrt(vals[0])
	out.MeasuredWheelNoise = out.ResidualRMS
	if vals[0] > 0 {
		out.Identifiability = vals[1] / vals[0]
	} else {
		out.Identifiability = math.Inf(1)
	}

	maxAbs := 0.0
	for i := 0; i < NumWheels; i++ {
		v := vecs.At(i, 0)
		out.N[i] = v
		if a := math.Abs(v); a > maxAbs {
			maxAbs = a
		}
	}
	if maxAbs == 0 {
		return WheelNullFit{}, fmt.Errorf("fit null vector: null vector is zero")
	}
	// Redundancy と同じ規約: 最大成分の絶対値を 1 にし、最初の有意な成分を正にする。
	for i := 0; i < NumWheels; i++ {
		out.N[i] /= maxAbs
	}
	for i := 0; i < NumWheels; i++ {
		if math.Abs(out.N[i]) > nullVectorEps {
			if out.N[i] < 0 {
				for j := 0; j < NumWheels; j++ {
					out.N[j] = -out.N[j]
				}
			}
			break
		}
	}
	return out, nil
}

// WheelAngleFit は左零ベクトルから読み取った取付角。
type WheelAngleFit struct {
	// PhiDeg は前輪の取付角 [deg] (FL が +phi、FR が -phi)。
	PhiDeg float64
	// ThetaDeg は基準にした後輪の取付角 [deg]。入力そのまま。
	ThetaDeg float64
	// RadiusRatioFrontRear は前輪半径 / 後輪半径。1 から離れるなら
	// 前後で半径が違うか、角度の仮定が合っていない。
	RadiusRatioFrontRear float64
	// FrontImbalance / RearImbalance は左右対称性の破れ。
	//
	//	front = (|d_FL| - |d_FR|) / (|d_FL| + |d_FR|)
	//
	// 0 から離れるなら左右で車輪半径が違う。0.02 (2%) 以上なら要調査。
	FrontImbalance float64
	RearImbalance  float64
}

// AnglesFromNullVector は左零ベクトルから前輪の取付角を読み取る。
//
// 閉形式 n_i = (r_i/s_i)*(sinθ, -sinφ, +sinφ, -sinθ) を逆に解く。
// **後輪の角度 thetaDeg を既知として与える** (すべての出典が 135 度で一致しているため)。
// 前後の半径が等しいと仮定すると
//
//	sinφ = sinθ * |d_BL + (-d_BR)| / |d_FL + (-d_FR)|
//
// となり、phi が決まる。signs は GeometryConfig.WheelSigns。
//
// **55 度説と 60 度説はここで分かれる。** 同じ車輪ログに対し
// |n_FL| が 0.863 なら 55 度、0.816 なら 60 度 (後輪 135 度・等半径のとき)。
func AnglesFromNullVector(n [NumWheels]float64, signs [NumWheels]float64, thetaDeg float64) (WheelAngleFit, error) {
	var d [NumWheels]float64
	for i := 0; i < NumWheels; i++ {
		if signs[i] == 0 {
			return WheelAngleFit{}, fmt.Errorf("angles from null vector: wheelSigns[%d] is zero", i)
		}
		d[i] = n[i] * signs[i]
	}

	// d_FL = +r_f sinθ, d_FR = -r_f sinθ  ->  front = (|d_FL|+|d_FR|)/2 = r_f*|sinθ|
	// d_BL = -r_b sinφ, d_BR = +r_b sinφ  ->  rear  = (|d_BL|+|d_BR|)/2 = r_b*|sinφ|
	front := 0.5 * (math.Abs(d[WheelFL]) + math.Abs(d[WheelFR]))
	rear := 0.5 * (math.Abs(d[WheelBL]) + math.Abs(d[WheelBR]))
	if front <= 0 {
		return WheelAngleFit{}, fmt.Errorf("angles from null vector: front components are zero")
	}

	theta := thetaDeg * math.Pi / 180
	// 等半径を仮定すると r_f = r_b が消えて sinφ = sinθ * rear / front。
	sinPhi := math.Abs(math.Sin(theta)) * rear / front
	if sinPhi > 1 {
		return WheelAngleFit{}, fmt.Errorf("angles from null vector: sin(phi) = %.4f > 1; geometry assumption is wrong", sinPhi)
	}

	fit := WheelAngleFit{
		PhiDeg:   math.Asin(sinPhi) * 180 / math.Pi,
		ThetaDeg: thetaDeg,
	}
	if s := math.Abs(d[WheelFL]) + math.Abs(d[WheelFR]); s > 0 {
		fit.FrontImbalance = (math.Abs(d[WheelFL]) - math.Abs(d[WheelFR])) / s
	}
	if s := math.Abs(d[WheelBL]) + math.Abs(d[WheelBR]); s > 0 {
		fit.RearImbalance = (math.Abs(d[WheelBL]) - math.Abs(d[WheelBR])) / s
	}
	// 半径比は角度を仮定した上での残り。等半径を仮定して phi を出しているので
	// ここは常に 1 になる。角度を固定して半径比を出したい場合のために式を残す。
	fit.RadiusRatioFrontRear = 1
	return fit, nil
}

// PredictedNullVectorMagnitude は、後輪 thetaDeg・前輪 phiDeg・等半径・
// 全輪同符号の機体で期待される |n_FL| (最大成分で正規化後) を返す。
//
// 55 度で 0.863、60 度で 0.816。実測の |n_FL| をこれと比べれば角度が決まる。
func PredictedNullVectorMagnitude(phiDeg, thetaDeg float64) float64 {
	sp := math.Abs(math.Sin(phiDeg * math.Pi / 180))
	st := math.Abs(math.Sin(thetaDeg * math.Pi / 180))
	if sp == 0 && st == 0 {
		return 0
	}
	return st / math.Max(sp, st)
}

// RadiiFromNullVector は**取付角とモーメントアームを信用できる値に固定した上で**、
// 車輪ログだけから 4 輪の有効転がり半径の比を求める。
//
// **これが較正の正しい切り分けである** (docs/self-localization-research-20260923.md §2.6)。
// 何がどの情報で決まるかを分けると:
//
//	取付角・アーム    機械加工で決まる。**CAD が権威**。データから解かない
//	半径の比 (3 自由度) **車輪ログだけで決まる**。vision も時刻合わせも要らない
//	半径の絶対値      vision が要る。フィルタの k_v がオンラインで吸収する
//
// 12 個を一度に当てはめると条件が悪い。PoC の vision 基準の同定は条件数 14.3、
// 残差 1.0-1.2 rad/s (= 車輪雑音とほぼ同じ) で、**モーメントアームが 49-71 mm と
// 出た** (真値 78.45 mm)。取付角もそこで 4 度ぶんずれて出ている。
//
// 仕組み: 左零ベクトル n は n^T D = 0 を満たす。D の行は (s_i/r_i)(sin a_i, -cos a_i, -R)
// なので、g_i := n_i * s_i / r_i と置くと
//
//	sum_i g_i * (sin a_i, -cos a_i, -R) = 0
//
// となり、**g は半径に依らない行列の左零ベクトル**になる。つまり
//
//	r_i = n_i * s_i / g_i
//
// で半径の比が直接出る。角度とアームが分かっていれば g は閉形式で計算できる。
func RadiiFromNullVector(n [NumWheels]float64, g GeometryConfig, meanRadiusM float64) ([NumWheels]float64, error) {
	if meanRadiusM <= 0 {
		return [NumWheels]float64{}, fmt.Errorf("radii from null vector: meanRadiusM must be positive")
	}
	// 半径に依らない行列 M0 の左零ベクトルを求める。
	m0 := mat.NewDense(NumWheels, 3, nil)
	for i := 0; i < NumWheels; i++ {
		a := g.WheelAnglesDeg[i] * math.Pi / 180
		s, c := math.Sincos(a)
		m0.Set(i, 0, s)
		m0.Set(i, 1, -c)
		m0.Set(i, 2, -g.MomentArmM)
	}
	var svd mat.SVD
	if ok := svd.Factorize(m0, mat.SVDFull); !ok {
		return [NumWheels]float64{}, fmt.Errorf("radii from null vector: SVD failed")
	}
	sv := svd.Values(nil)
	if len(sv) < 3 || sv[2] <= 0 {
		return [NumWheels]float64{}, fmt.Errorf("radii from null vector: wheel angles are degenerate")
	}
	var u mat.Dense
	svd.UTo(&u)

	var out [NumWheels]float64
	var sum float64
	for i := 0; i < NumWheels; i++ {
		gi := u.At(i, NumWheels-1)
		if math.Abs(gi) < nullVectorEps {
			return [NumWheels]float64{}, fmt.Errorf("radii from null vector: wheel %d does not appear in the null direction", i)
		}
		if g.WheelSigns[i] == 0 {
			return [NumWheels]float64{}, fmt.Errorf("radii from null vector: wheelSigns[%d] is zero", i)
		}
		out[i] = n[i] * g.WheelSigns[i] / gi
		sum += out[i]
	}
	if sum == 0 {
		return [NumWheels]float64{}, fmt.Errorf("radii from null vector: degenerate solution")
	}
	// **比しか決まらない。** 絶対値は vision (あるいはフィルタの k_v) の仕事。
	scale := meanRadiusM * float64(NumWheels) / sum
	for i := 0; i < NumWheels; i++ {
		out[i] *= scale
		if out[i] <= 0 {
			return [NumWheels]float64{}, fmt.Errorf("radii from null vector: wheel %d came out non-positive (%v); "+
				"the fixed angles or the signs are wrong", i, out[i])
		}
	}
	return out, nil
}

// CalibrateRadii は車輪ログから、角度を固定したまま半径を較正した設定を返す。
//
// base の取付角・アーム・符号・並びはそのまま使い、半径だけ差し替える。
// meanRadiusM は半径の平均 (絶対値は車輪ログからは決まらないので与える)。
func CalibrateRadii(samples [][NumWheels]float64, base GeometryConfig, meanRadiusM float64) (GeometryConfig, WheelNullFit, error) {
	fit, err := FitNullVector(samples)
	if err != nil {
		return base, WheelNullFit{}, err
	}
	if fit.Identifiability < 5 {
		return base, fit, fmt.Errorf("calibrate radii: identifiability %.1f is too low; "+
			"the log must contain forward, sideways and rotational motion", fit.Identifiability)
	}
	radii, err := RadiiFromNullVector(fit.N, base, meanRadiusM)
	if err != nil {
		return base, fit, err
	}
	out := base
	out.WheelRadiusM = radii
	if err := out.Validate(); err != nil {
		return base, fit, err
	}
	return out, fit, nil
}
