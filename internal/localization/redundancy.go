package localization

import (
	"fmt"
	"math"

	"gonum.org/v1/gonum/mat"
)

// 4 輪の冗長性から作る整合残差 (docs/self-localization-research-20260923.md §2.4)。
//
// 速度結合行列 M は 4x3 で階数 3 なので、左零空間が 1 次元ある。その方向 n は
//
//	n^T M = 0
//
// を満たすので、滑りが無ければ**どんな運動に対しても** n^T w = 0 が厳密に成り立つ。
// w は 4 輪の実測角速度そのものなので、**速度推定も vision も時刻合わせも要らない**。
//
// 前後で取付角が違う機体 (α = (φ, θ, -θ, -φ)) では閉形式で書ける:
//
//	n_i = (r_i / s_i) * m_i,   m = ( sinθ, -sinφ, +sinφ, -sinθ )
//
// φ = θ を入れると Rojas (Omnidirectional Control, 2005) の対称ロボット用の
// (1, -1, 1, -1) に戻る。ここでは任意の配置に効くよう SVD で数値的に求め、
// 閉形式との一致をテストで固定してある。
//
// 使い道は 3 つ:
//
//   - **vision なしの幾何較正**: 残差を最小にする角度・半径比を探す。
//     55 度説と 60 度説で |n_1| が 0.863 対 0.816 (5.8% 差) になるので、
//     車輪のログだけで判定できる。
//   - **スリップ検出**: 正規化残差の二乗を chi^2(1) で検定する。
//   - **車輪雑音の実測**: 滑っていない区間の残差の分散がそのまま WheelNoise になる。
//
// R (モーメントアーム) には依存しない。角度と半径の比だけで決まる。
// nullVectorEps は左零ベクトルの成分を「有意」とみなす下限。
const nullVectorEps = 1e-9

type Redundancy struct {
	// n は正規化した左零ベクトル (論理輪番号の順、最大成分の絶対値が 1)。
	n [NumWheels]float64
	// norm2 は sum(n_i^2)。単位分散の車輪雑音に対する残差の分散。
	norm2 float64
	// cond は M の条件数。大きすぎると n が数値的に決まらない。
	cond float64
}

// NewRedundancy は運動学から整合残差を構築する。
func NewRedundancy(k *Kinematics) (*Redundancy, error) {
	m := mat.NewDense(NumWheels, 3, nil)
	for i := 0; i < NumWheels; i++ {
		row := k.Row(i)
		for j := 0; j < 3; j++ {
			m.Set(i, j, row[j])
		}
	}

	var svd mat.SVD
	if ok := svd.Factorize(m, mat.SVDFull); !ok {
		return nil, fmt.Errorf("redundancy: SVD failed")
	}
	sv := svd.Values(nil)
	if len(sv) < 3 || sv[2] <= 0 {
		return nil, fmt.Errorf("redundancy: wheel configuration is degenerate")
	}

	var u mat.Dense
	svd.UTo(&u)

	r := &Redundancy{cond: sv[0] / sv[2]}
	// 4 列目 (特異値ゼロに対応する左特異ベクトル) が左零空間。
	maxAbs := 0.0
	for i := 0; i < NumWheels; i++ {
		v := u.At(i, NumWheels-1)
		r.n[i] = v
		if a := math.Abs(v); a > maxAbs {
			maxAbs = a
		}
	}
	if maxAbs == 0 {
		return nil, fmt.Errorf("redundancy: null vector is zero")
	}
	// 最大成分の絶対値を 1 に正規化し、**最初の有意な成分が正**になるよう向きを固定する。
	// SVD が返す符号は任意で、成分の大きさが同点だと選び方が浮動小数の丸めで揺れるため、
	// 「最大成分の符号」ではなく「最初の成分の符号」を基準にする。
	// 残差の符号が反転するだけなので意味は変わらないが、再現性のために固定しておく。
	for i := 0; i < NumWheels; i++ {
		r.n[i] /= maxAbs
	}
	for i := 0; i < NumWheels; i++ {
		if math.Abs(r.n[i]) > nullVectorEps {
			if r.n[i] < 0 {
				for j := 0; j < NumWheels; j++ {
					r.n[j] = -r.n[j]
				}
			}
			break
		}
	}
	for i := 0; i < NumWheels; i++ {
		r.norm2 += r.n[i] * r.n[i]
	}
	return r, nil
}

// NullVector は正規化した左零ベクトルを返す (論理輪番号の順)。
func (r *Redundancy) NullVector() [NumWheels]float64 { return r.n }

// ConditionNumber は速度結合行列の条件数を返す。
func (r *Redundancy) ConditionNumber() float64 { return r.cond }

// Residual は整合残差 n^T w を返す [rad/s]。入力は論理輪番号の順。
// 滑りが無く、幾何が正しければゼロになる。アロケートしない。
func (r *Redundancy) Residual(logical [NumWheels]float64) float64 {
	var sum float64
	for i := 0; i < NumWheels; i++ {
		sum += r.n[i] * logical[i]
	}
	return sum
}

// NormalizedResidual は残差を車輪雑音で割った値を返す。
//
// 各輪の雑音が独立で標準偏差 sigma なら、残差の標準偏差は sigma*sqrt(sum n_i^2)
// になる。返り値の二乗が自由度 1 のカイ二乗統計量。
func (r *Redundancy) NormalizedResidual(logical [NumWheels]float64, sigma float64) float64 {
	if sigma <= 0 || r.norm2 <= 0 {
		return 0
	}
	return r.Residual(logical) / (sigma * math.Sqrt(r.norm2))
}

// ResidualVariance は残差の期待分散を返す [rad^2/s^2]。
//
// **車輪雑音は速度に比例する分を持つ** (オムニ車輪の polygon 効果、研究 §3.7)
// ので、輪ごとに分散が違う。定数の sigma で割ると、速く回っているときに
// 「いつも滑っている」と誤判定する。
//
//	var = sum_i n_i^2 * ( sigma0^2 + (coef * omega_i)^2 )
func (r *Redundancy) ResidualVariance(logical [NumWheels]float64, sigma0, coef float64) float64 {
	var v float64
	for i := 0; i < NumWheels; i++ {
		s2 := sigma0 * sigma0
		if coef > 0 {
			sp := coef * logical[i]
			s2 += sp * sp
		}
		v += r.n[i] * r.n[i] * s2
	}
	return v
}

// NoiseScale は sqrt(sum n_i^2) を返す。
//
// 滑っていない区間の残差の標準偏差をこれで割れば、1 輪あたりの雑音の
// 標準偏差 [rad/s] が実測できる。
func (r *Redundancy) NoiseScale() float64 { return math.Sqrt(r.norm2) }

// ClosedFormNullVector は α = (φ, θ, -θ, -φ) の機体に対する左零ベクトルを
// 閉形式で返す (論理輪番号の順、正規化前)。
//
//	n_i = (r_i / s_i) * ( sinθ, -sinφ, +sinφ, -sinθ )
//
// 数値解 (NewRedundancy) との一致をテストで固定するために公開してある。
// 実行時の経路では使わない。
func ClosedFormNullVector(cfg GeometryConfig) [NumWheels]float64 {
	phi := cfg.WheelAnglesDeg[WheelFL] * math.Pi / 180
	theta := cfg.WheelAnglesDeg[WheelBL] * math.Pi / 180
	base := [NumWheels]float64{
		WheelFL: math.Sin(theta),
		WheelBL: -math.Sin(phi),
		WheelBR: math.Sin(phi),
		WheelFR: -math.Sin(theta),
	}
	var out [NumWheels]float64
	for i := 0; i < NumWheels; i++ {
		out[i] = base[i] * cfg.WheelRadiusM[i] / cfg.WheelSigns[i]
	}
	return out
}

// SlipMonitor は整合残差でスリップを監視する。
//
// Yu ほか (IROS 2023) はスリップ状態そのものを chi^2(3) で検定しているが、
// こちらは**推定に一切依存しない** 1 自由度の検定になる。閾値は
// 自由度 1 のカイ二乗分布から取る (既定 6.63 = 99% 点)。
//
// 単一の goroutine からのみ触ること。アロケートしない。
type SlipMonitor struct {
	red    *Redundancy
	sigma0 float64
	coef   float64
	// thresh は正規化残差の二乗のしきい値。
	thresh float64

	// 残差の統計 (滑っていないと判定した周期のみ)。
	sum, sum2 float64
	// predVar は同じ周期での期待分散の総和。実測との比が雑音モデルのずれを表す。
	predVar float64
	count   int64
	// slipCount はしきい値を超えた周期の数。
	slipCount int64
	last      float64
	lastNorm  float64
}

// NewSlipMonitor はスリップ監視を作る。
//
// sigma0 は速度に依存しない車輪雑音 [rad/s]、coef は角速度に比例する分の係数。
// NoiseConfig の WheelNoise / WheelNoiseSpeedCoef をそのまま渡す。
// chi2Threshold が 0 なら 6.63 (自由度 1 の 99% 点) を使う。
func NewSlipMonitor(red *Redundancy, sigma0, coef, chi2Threshold float64) *SlipMonitor {
	if chi2Threshold <= 0 {
		chi2Threshold = 6.63
	}
	return &SlipMonitor{red: red, sigma0: sigma0, coef: coef, thresh: chi2Threshold}
}

// Observe は 1 周期ぶんの車輪速を取り込み、スリップと判定したら true を返す。
func (m *SlipMonitor) Observe(logical [NumWheels]float64) bool {
	m.last = m.red.Residual(logical)
	varr := m.red.ResidualVariance(logical, m.sigma0, m.coef)
	if varr > 0 {
		m.lastNorm = m.last / math.Sqrt(varr)
	} else {
		m.lastNorm = 0
	}
	if m.lastNorm*m.lastNorm > m.thresh {
		m.slipCount++
		return true
	}
	m.sum += m.last
	m.sum2 += m.last * m.last
	m.predVar += varr
	m.count++
	return false
}

// Residual は直近の残差 [rad/s] を返す。
func (m *SlipMonitor) Residual() float64 { return m.last }

// NormalizedResidual は直近の正規化残差を返す。
func (m *SlipMonitor) NormalizedResidual() float64 { return m.lastNorm }

// SlipCount はスリップと判定した周期の数を返す。
func (m *SlipMonitor) SlipCount() int64 { return m.slipCount }

// SlipRate はスリップと判定した周期の割合を返す。
//
// **これが 0.5 を超え続けるなら、滑っているのではなく幾何が間違っている。**
// 雑音は平均ゼロなので、残差がいつも閾値を超えるのは n が実機と合っていない
// ときだけである。ResidualBias と併せて見ること。
func (m *SlipMonitor) SlipRate() float64 {
	total := m.count + m.slipCount
	if total == 0 {
		return 0
	}
	return float64(m.slipCount) / float64(total)
}

// NoiseScaleFactor は「実測の残差の広がり / 雑音モデルが予測する広がり」を返す。
//
// **1.0 なら WheelNoise と WheelNoiseSpeedCoef が実機と合っている。**
// 1 より大きければ雑音を小さく見積もりすぎ、小さければ大きく見積もりすぎ。
// 合成データの掃引ではなく実測でこの 2 つを決めるための量である。
func (m *SlipMonitor) NoiseScaleFactor() (float64, bool) {
	if m.count < 30 || m.predVar <= 0 {
		return 0, false
	}
	return math.Sqrt(m.sum2 / m.predVar), true
}

// MeasuredWheelNoise は滑っていない区間から実測した 1 輪あたりの雑音の
// 標準偏差 [rad/s] を返す。サンプルが足りなければ false。
//
// **速度に比例する分が無い (coef = 0) 場合にのみ、そのまま WheelNoise になる。**
// 比例分があるときは速度で変わる量なので、NoiseScaleFactor を使うこと。
func (m *SlipMonitor) MeasuredWheelNoise() (float64, bool) {
	if m.count < 30 {
		return 0, false
	}
	n := float64(m.count)
	mean := m.sum / n
	varr := (m.sum2 - n*mean*mean) / (n - 1)
	if varr <= 0 {
		return 0, false
	}
	scale := m.red.NoiseScale()
	if scale <= 0 {
		return 0, false
	}
	return math.Sqrt(varr) / scale, true
}

// ResidualBias は滑っていない区間の残差の平均 [rad/s] を返す。
//
// **ゼロから離れているなら幾何が間違っている。** 雑音は平均ゼロなので、
// 偏りが残るのは n が実機と合っていないときだけである。
func (m *SlipMonitor) ResidualBias() (float64, bool) {
	if m.count < 30 {
		return 0, false
	}
	return m.sum / float64(m.count), true
}
