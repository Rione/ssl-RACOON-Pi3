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
}

// updateWheels は 4 輪の角速度 (論理輪番号の順) で更新する。
//
// 観測モデル: omega_i = M_i . [v_body + slip, omega]
//
// **4 次元のまま扱う。** 擬似逆で 3 次元に潰すと 4 輪それぞれの雑音が混ざり、
// 個別の車輪異常が見えなくなる。冗長性の残差はスリップ推定に効く情報でもある
// (計画 §4.5)。
func (f *eskf) updateWheels(logical [NumWheels]float64) UpdateInfo {
	const m = NumWheels
	vb := f.x.VelBody()
	sin, cos := math.Sincos(f.x.Theta)

	for i := 0; i < m; i++ {
		row := f.kin.Row(i)
		m0, m1, m2 := row[0], row[1], row[2]

		for j := 0; j < stateDim; j++ {
			f.h[i][j] = 0
		}
		// d(v_body)/d(dPhi) = -J * v_body = (v_by, -v_bx)
		f.h[i][idxPhi] = m0*vb.Y - m1*vb.X
		// d(v_body)/d(dV) = R(theta)^T
		f.h[i][idxVx] = m0*cos - m1*sin
		f.h[i][idxVy] = m0*sin + m1*cos
		f.h[i][idxOmega] = m2
		if f.cfg.Noise.EnableSlip {
			f.h[i][idxSx] = m0
			f.h[i][idxSy] = m1
		}

		expected := m0*(vb.X+f.x.Slip.X) + m1*(vb.Y+f.x.Slip.Y) + m2*f.x.Omega
		f.nu[i] = logical[i] - expected
		f.rdia[i] = f.cfg.Noise.WheelNoise * f.cfg.Noise.WheelNoise
	}
	return f.applyUpdate(m)
}

// updateVision は vision の絶対姿勢で更新する。
//
// **観測モデルは誤差状態に対して厳密に線形である** (h = [p, theta] そのもの)。
// したがって反復更新 (IEKF) を回しても再線形化する対象が無く、EKF と一致する。
// 計画 §4.4 は IEKF を挙げているが、この観測に関しては入れる意味がない。
func (f *eskf) updateVision(pose Pose2) UpdateInfo {
	const m = 3
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

	pv := f.cfg.Noise.VisionPosNoise * f.cfg.Noise.VisionPosNoise
	f.rdia[0] = pv
	f.rdia[1] = pv
	f.rdia[2] = f.cfg.Noise.VisionAngNoise * f.cfg.Noise.VisionAngNoise

	return f.applyUpdate(m)
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
	for i := range f.dx {
		f.dx[i] = 0
	}
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
	vals := []float64{f.x.Theta, f.x.P.X, f.x.P.Y, f.x.V.X, f.x.V.Y, f.x.Omega, f.x.Slip.X, f.x.Slip.Y}
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
