package localization

import "math"

// 観測更新。車輪 (4 次元) と vision (3 次元) の両方をここで扱う。

// UpdateInfo は 1 回の観測更新の診断情報。/est/innovation に出す。
type UpdateInfo struct {
	// Applied は更新が実際に適用されたか。共分散が壊れていれば false。
	Applied bool
	// Dim は観測の次元。
	Dim int
	// Innovation は残差。
	Innovation [maxObs]float64
	// Normalized は正規化イノベーション sqrt(nu^T S^-1 nu)。
	Normalized float64
	// HuberWeight は Huber の重み。1 なら減衰していない。
	HuberWeight float64
	// RScale は適応 R が公称値の何倍になっているか (vision のみ、位置成分)。
	RScale float64
	// ParamsFrozen はこの更新で機体パラメータの補正を止めたか。
	ParamsFrozen bool
}

// updateWheels は 4 輪の角速度 (論理輪番号の順) で更新する。
//
// 観測モデル (機体パラメータの倍率つき):
//
//	u          = Kv * (v_body + slip)                 並進の倍率
//	omega_i    = g_i * [ sin(a_i + Ka)*u_x - cos(a_i + Ka)*u_y - R*Kw*omega ]
//	g_i        = sign_i / r_i
//
// Kv / Kw / Ka は状態なので、**CAD と同定値の食い違い (研究 §2.3) を
// オンラインで吸収する**。公称は Kv = Kw = 1, Ka = 0。
//
// **4 次元のまま扱う。** 擬似逆で 3 次元に潰すと 4 輪それぞれの雑音が混ざり、
// 個別の車輪異常が見えなくなる。冗長性の残差はスリップ推定に効く情報でもある
// (計画 §4.5)。
func (f *eskf) updateWheels(logical [NumWheels]float64) UpdateInfo {
	const m = NumWheels
	n := f.cfg.Noise
	vb := f.x.VelBody()
	sin, cos := math.Sincos(f.x.Theta)

	// 並進の入力 (倍率をかける前の body 速度 + スリップ)。
	ux := vb.X
	uy := vb.Y
	if n.EnableSlip {
		ux += f.x.Slip.X
		uy += f.x.Slip.Y
	}
	arm := f.kin.Arm()

	// スリップ量で R を膨らませる倍率 (Yu ほか IROS 2023 の W_y = W_0 exp(||u||))。
	// 等方に効かせ、上限でクランプする。
	slipScale := 1.0
	if n.EnableSlip && n.SlipRScaleRef > 0 {
		mag := math.Hypot(f.x.Slip.X, f.x.Slip.Y)
		slipScale = math.Exp(mag / n.SlipRScaleRef)
		if slipScale > n.SlipRScaleMax {
			slipScale = n.SlipRScaleMax
		}
	}

	for i := 0; i < m; i++ {
		g := f.kin.Gain(i)
		sa, ca := math.Sincos(f.kin.Angle(i) + f.x.Ka)
		// 倍率を掛ける前の観測行 (単位は 1/m)。
		rowX := g * sa
		rowY := -g * ca

		for j := 0; j < stateDim; j++ {
			f.h[i][j] = 0
		}
		// d(v_body)/d(dPhi) = -J * v_body = (v_by, -v_bx)
		f.h[i][idxPhi] = f.x.Kv * (rowX*vb.Y - rowY*vb.X)
		// d(v_body)/d(dV) = R(theta)^T
		f.h[i][idxVx] = f.x.Kv * (rowX*cos - rowY*sin)
		f.h[i][idxVy] = f.x.Kv * (rowX*sin + rowY*cos)
		f.h[i][idxOmega] = -g * arm * f.x.Kw
		if n.EnableSlip {
			f.h[i][idxSx] = f.x.Kv * rowX
			f.h[i][idxSy] = f.x.Kv * rowY
		}
		if n.EnableParamEstimation {
			f.h[i][idxKv] = rowX*ux + rowY*uy
			f.h[i][idxKw] = -g * arm * f.x.Omega
			// d/dKa [ sin(a+Ka)*ux - cos(a+Ka)*uy ] = cos(a+Ka)*ux + sin(a+Ka)*uy
			f.h[i][idxKa] = f.x.Kv * g * (ca*ux + sa*uy)
		}

		expected := f.x.Kv*(rowX*ux+rowY*uy) - g*arm*f.x.Kw*f.x.Omega
		f.nu[i] = logical[i] - expected

		// 車輪雑音は速度に比例する分を持つ。オムニ車輪の有効転がり半径が
		// ローラの入れ替わりで変動する (polygon 効果) ため (研究 §3.7)。
		sigma2 := n.WheelNoise * n.WheelNoise
		if n.WheelNoiseSpeedCoef > 0 {
			sp := n.WheelNoiseSpeedCoef * logical[i]
			sigma2 += sp * sp
		}
		f.rdia[i] = sigma2 * slipScale
	}
	info := f.applyUpdate(m)
	info.ParamsFrozen = f.paramsFrozen
	return info
}

// updateZupt は停止中の疑似観測 (ワールド速度 0、ヨーレート 0) を当てる。
//
// SSL のロボットは試合中に頻繁に止まる (STOP / HALT / 配置待ち)。止まっている
// あいだ位置がクリープするのを止め、スリップ状態をゼロへ引き戻す。
// **誤検出は致命的** (動いているのに位置が固まる) なので、呼ぶ側で
// 3 条件の AND を取ること (estimator.go の zuptDetect)。
func (f *eskf) updateZupt() UpdateInfo {
	const m = 3
	n := f.cfg.Noise
	for i := 0; i < m; i++ {
		for j := 0; j < stateDim; j++ {
			f.h[i][j] = 0
		}
	}
	f.h[0][idxVx] = 1
	f.h[1][idxVy] = 1
	f.h[2][idxOmega] = 1

	f.nu[0] = -f.x.V.X
	f.nu[1] = -f.x.V.Y
	f.nu[2] = -f.x.Omega

	vv := n.ZuptVelNoise * n.ZuptVelNoise
	f.rdia[0] = vv
	f.rdia[1] = vv
	f.rdia[2] = n.ZuptOmegaNoise * n.ZuptOmegaNoise
	return f.applyUpdate(m)
}

// updateGyro はジャイロのヨーレートで更新する。
//
//	z = omega + b_g + noise
//
// ジャイロを「予測の入力」ではなく観測にしている理由は NoiseConfig.EnableGyro
// のコメントに書いた。バイアス b_g は vision の向きと ZARU (停止中の
// omega = 0 の疑似観測) 経由で可観測になる。**停止のたびに較正し直される。**
func (f *eskf) updateGyro(z float64) UpdateInfo {
	const m = 1
	for j := 0; j < stateDim; j++ {
		f.h[0][j] = 0
	}
	f.h[0][idxOmega] = 1
	f.h[0][idxBg] = 1
	f.nu[0] = z - (f.x.Omega + f.x.GyroBias)
	f.rdia[0] = f.cfg.Noise.GyroNoise * f.cfg.Noise.GyroNoise
	return f.applyUpdate(m)
}

// noteCollision は加速度が閾値を超えたことを記録し、しばらくプロセス雑音を膨らませる。
func (f *eskf) noteCollision(cycles int) {
	if cycles > f.collisionCycles {
		f.collisionCycles = cycles
	}
}

// updateVision は vision の絶対姿勢で更新する。
//
// **観測モデルは誤差状態に対して厳密に線形である** (h = [p, theta] そのもの)。
// したがって反復更新 (IEKF) を回しても再線形化する対象が無く、EKF と一致する。
// 計画 §4.4 は IEKF を挙げているが、この観測に関しては入れる意味がない。
func (f *eskf) updateVision(pose Pose2) UpdateInfo {
	const m = 3
	n := f.cfg.Noise
	for i := 0; i < m; i++ {
		for j := 0; j < stateDim; j++ {
			f.h[i][j] = 0
		}
	}
	f.h[0][idxPx] = 1
	f.h[1][idxPy] = 1
	f.h[2][idxPhi] = 1

	f.nu[0] = pose.X - f.x.P.X
	f.nu[1] = pose.Y - f.x.P.Y
	// 角度の残差は必ず折り返してから使う。素直に引くと +-pi 付近で 2pi 飛ぶ。
	f.nu[2] = AngleDiff(pose.Theta, f.x.Theta)

	pv := n.VisionPosNoise * n.VisionPosNoise
	av := n.VisionAngNoise * n.VisionAngNoise
	// **時刻の不確かさを速度で位置の不確かさへ直して足す** (計画 §5.3)。
	// 定数の R では表せない速度依存の項で、入れないと速い走りで過信になる。
	if st := n.VisionTimeSigma; st > 0 {
		vb := f.x.VelBody()
		speed := math.Hypot(vb.X, vb.Y)
		dp := speed * st
		pv += dp * dp
		dth := f.x.Omega * st
		av += dth * dth
	}
	nominalR := [3]float64{pv, pv, av}

	if n.AdaptiveVisionR {
		// VB-AKF (Sarkka & Nummenmaa) の軸ごと逆ガンマ版。
		// 予測: 忘却係数 rho を十分統計量に掛ける (平均は変えず分散を広げる)。
		for i := 0; i < m; i++ {
			f.vbA[i] *= n.AdaptiveForgetting
			f.vbB[i] *= n.AdaptiveForgetting
			r := f.vbB[i] / f.vbA[i]
			// クランプは発散防止のため必須 (計画 §4.4)。
			if lo := nominalR[i] * n.AdaptiveMinScale; r < lo {
				r = lo
			}
			if hi := nominalR[i] * n.AdaptiveMaxScale; r > hi {
				r = hi
			}
			f.rdia[i] = r
		}
	} else {
		for i := 0; i < m; i++ {
			f.rdia[i] = nominalR[i]
		}
	}

	// イノベーションは更新で消えるので、統計量の更新用に控えておく。
	var innov [3]float64
	copy(innov[:], f.nu[:m])

	info := f.applyUpdate(m)
	if nominalR[0] > 0 {
		info.RScale = f.rdia[0] / nominalR[0]
	}
	info.ParamsFrozen = f.paramsFrozen

	if n.AdaptiveVisionR && info.Applied {
		// 更新: b <- b + 0.5*(nu^2 + (H P_post H^T)_ii), a <- a + 0.5。
		// P_post を使うのが VB の固定点 (事前の P を使うのは古典的な
		// イノベーション整合で、R を過大に見積もる)。
		for i := 0; i < m; i++ {
			hph := f.quadraticFormP(i)
			f.vbA[i] += 0.5
			f.vbB[i] += 0.5 * (innov[i]*innov[i] + hph)
		}
	}
	return info
}

// quadraticFormP は (H P H^T)_ii を現在の P で計算する。
// 適応 R の十分統計量の更新に使う。アロケートしない。
func (f *eskf) quadraticFormP(i int) float64 {
	var sum float64
	for a := 0; a < stateDim; a++ {
		if f.h[i][a] == 0 {
			continue
		}
		var inner float64
		for b := 0; b < stateDim; b++ {
			if f.h[i][b] != 0 {
				inner += f.P[a][b] * f.h[i][b]
			}
		}
		sum += f.h[i][a] * inner
	}
	return sum
}

// applyUpdate は H / nu / rdia が埋まった状態で更新を実行する。
func (f *eskf) applyUpdate(m int) UpdateInfo {
	info := UpdateInfo{Dim: m, HuberWeight: 1}
	copy(info.Innovation[:], f.nu[:m])

	f.computePH(m)
	f.computeS(m, 1)

	// 正規化イノベーション sqrt(nu^T S^-1 nu) を測る。
	f.s2 = f.s
	if !cholFactor(&f.s2, m) {
		return info // 共分散が壊れている。更新を捨てる。
	}
	copy(f.sol[:m], f.nu[:m])
	cholSolve(&f.s2, m, &f.sol)
	var d2 float64
	for i := 0; i < m; i++ {
		d2 += f.nu[i] * f.sol[i]
	}
	if d2 < 0 || math.IsNaN(d2) {
		return info
	}
	info.Normalized = math.Sqrt(d2)

	// Huber。しきい値を超えたら重みを c/||nu|| へ落とす。
	//
	// カイ二乗のハードゲートは使わない。一度弾き始めると共分散が縮んだまま
	// 復帰できない失敗モードがあるため (計画 §4.4)。重みで落とす分には、
	// 残差が収まれば自然に戻る。
	scale := 1.0
	if c := f.cfg.Noise.HuberC; info.Normalized > c && c > 0 {
		info.HuberWeight = c / info.Normalized
		scale = info.Normalized / c
		f.computeS(m, scale)
	}

	f.s2 = f.s
	if !cholFactor(&f.s2, m) {
		return info
	}

	// K = P H^T S^-1。行ごとに S k_i^T = (P H^T)_i を解く。
	for i := 0; i < stateDim; i++ {
		copy(f.sol[:m], f.ph[i][:m])
		cholSolve(&f.s2, m, &f.sol)
		copy(f.k[i][:m], f.sol[:m])
	}

	// 機体パラメータの補正を凍結する。
	//
	// Joseph 形式 P = (I-KH)P(I-KH)^T + K R K^T は**任意の K に対して**正しいので、
	// ゲインの行をゼロにしても共分散は整合したままになる (最適ではなくなるだけ)。
	// これは Schmidt-Kalman (consider) フィルタと同じ扱い。
	if f.paramsFrozen || !f.cfg.Noise.EnableParamEstimation {
		for _, i := range paramIndices {
			for j := 0; j < m; j++ {
				f.k[i][j] = 0
			}
		}
	}

	// dx = K nu
	for i := 0; i < stateDim; i++ {
		var sum float64
		for j := 0; j < m; j++ {
			sum += f.k[i][j] * f.nu[j]
		}
		f.dx[i] = sum
	}

	f.josephUpdate(m, scale)
	f.inject()
	info.Applied = true
	return info
}

// computePH は ph = P H^T を計算する。
func (f *eskf) computePH(m int) {
	for i := 0; i < stateDim; i++ {
		for j := 0; j < m; j++ {
			var sum float64
			for k := 0; k < stateDim; k++ {
				if f.h[j][k] != 0 {
					sum += f.P[i][k] * f.h[j][k]
				}
			}
			f.ph[i][j] = sum
		}
	}
}

// computeS は s = H P H^T + scale*R を計算する。
func (f *eskf) computeS(m int, scale float64) {
	for i := 0; i < m; i++ {
		for j := 0; j < m; j++ {
			var sum float64
			for k := 0; k < stateDim; k++ {
				if f.h[i][k] != 0 {
					sum += f.h[i][k] * f.ph[k][j]
				}
			}
			if i == j {
				sum += scale * f.rdia[i]
			}
			f.s[i][j] = sum
		}
	}
}

// josephUpdate は P <- (I-KH) P (I-KH)^T + K R K^T を計算する。
//
// 素朴な P <- (I-KH) P より演算は多いが、対称性と正定値性が保たれる。
// 共分散が壊れると NEES が意味を失い、位置制御が信じてはいけない値を信じる。
func (f *eskf) josephUpdate(m int, scale float64) {
	for i := 0; i < stateDim; i++ {
		for j := 0; j < stateDim; j++ {
			var sum float64
			for k := 0; k < m; k++ {
				sum += f.k[i][k] * f.h[k][j]
			}
			f.ikh[i][j] = -sum
			if i == j {
				f.ikh[i][j] += 1
			}
		}
	}
	// tmp = (I-KH) * P
	for i := 0; i < stateDim; i++ {
		for j := 0; j < stateDim; j++ {
			var sum float64
			for k := 0; k < stateDim; k++ {
				sum += f.ikh[i][k] * f.P[k][j]
			}
			f.tmp[i][j] = sum
		}
	}
	// P = tmp * (I-KH)^T + K R K^T
	for i := 0; i < stateDim; i++ {
		for j := 0; j < stateDim; j++ {
			var sum float64
			for k := 0; k < stateDim; k++ {
				sum += f.tmp[i][k] * f.ikh[j][k]
			}
			for k := 0; k < m; k++ {
				sum += f.k[i][k] * scale * f.rdia[k] * f.k[j][k]
			}
			f.P[i][j] = sum
		}
	}
	f.symmetrize()
}

// inject は誤差状態を公称状態へ合成し、誤差をゼロへ戻す。
//
// SO(2) では姿勢誤差の注入が単なる加算 + 折り返しで済み、
// **共分散のリセット変換が恒等になる** (2 次元の回転が可換なため)。
// 3 次元だとここで J = I - 0.5*[dPhi]x を掛ける必要がある。
func (f *eskf) inject() {
	f.x.Theta = WrapAngle(f.x.Theta + f.dx[idxPhi])
	f.x.P.X += f.dx[idxPx]
	f.x.P.Y += f.dx[idxPy]
	f.x.V.X += f.dx[idxVx]
	f.x.V.Y += f.dx[idxVy]
	f.x.Omega += f.dx[idxOmega]
	if f.cfg.Noise.EnableSlip {
		f.x.Slip.X += f.dx[idxSx]
		f.x.Slip.Y += f.dx[idxSy]
	}
	if f.cfg.Noise.EnableGyro {
		f.x.GyroBias += f.dx[idxBg]
	}
	if f.cfg.Noise.EnableParamEstimation {
		n := f.cfg.Noise
		f.x.Kv = clamp(f.x.Kv+f.dx[idxKv], 1-n.ParamScaleLimit, 1+n.ParamScaleLimit)
		f.x.Kw = clamp(f.x.Kw+f.dx[idxKw], 1-n.ParamScaleLimit, 1+n.ParamScaleLimit)
		lim := n.ParamAngleLimitDeg * math.Pi / 180
		f.x.Ka = clamp(f.x.Ka+f.dx[idxKa], -lim, lim)
	}
	for i := range f.dx {
		f.dx[i] = 0
	}
}

// paramIndices は機体パラメータの誤差状態の位置。
var paramIndices = [3]int{idxKv, idxKw, idxKa}

func clamp(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

// covPose は [x, y, theta] の共分散を取り出す。
func (f *eskf) covPose() Mat3 {
	idx := [3]int{idxPx, idxPy, idxPhi}
	var out Mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			out[i][j] = f.P[idx[i]][idx[j]]
		}
	}
	return out
}

// finite は状態と共分散が数値的に壊れていないかを返す。
func (f *eskf) finite() bool {
	vals := [...]float64{
		f.x.Theta, f.x.P.X, f.x.P.Y, f.x.V.X, f.x.V.Y, f.x.Omega,
		f.x.Slip.X, f.x.Slip.Y, f.x.Kv, f.x.Kw, f.x.Ka, f.x.GyroBias,
	}
	for _, v := range vals {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return false
		}
	}
	for i := 0; i < stateDim; i++ {
		if f.P[i][i] < 0 || math.IsNaN(f.P[i][i]) || math.IsInf(f.P[i][i], 0) {
			return false
		}
	}
	return true
}

// --- 固定長の Cholesky (アロケートしない) --------------------------------

// cholFactor は対称正定値行列を下三角 L に分解する (in-place)。
// 正定値でなければ false。上三角は触らないので、読むのは下三角だけにすること。
func cholFactor(s *[maxObs][maxObs]float64, m int) bool {
	for i := 0; i < m; i++ {
		for j := 0; j <= i; j++ {
			sum := s[i][j]
			for k := 0; k < j; k++ {
				sum -= s[i][k] * s[j][k]
			}
			if i == j {
				if sum <= 0 || math.IsNaN(sum) {
					return false
				}
				s[i][i] = math.Sqrt(sum)
			} else {
				s[i][j] = sum / s[j][j]
			}
		}
	}
	return true
}

// cholSolve は L L^T x = b を解き、b を x で上書きする。
func cholSolve(l *[maxObs][maxObs]float64, m int, b *[maxObs]float64) {
	for i := 0; i < m; i++ {
		sum := b[i]
		for k := 0; k < i; k++ {
			sum -= l[i][k] * b[k]
		}
		b[i] = sum / l[i][i]
	}
	for i := m - 1; i >= 0; i-- {
		sum := b[i]
		for k := i + 1; k < m; k++ {
			sum -= l[k][i] * b[k]
		}
		b[i] = sum / l[i][i]
	}
}
