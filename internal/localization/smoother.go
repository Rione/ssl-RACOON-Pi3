package localization

import (
	"fmt"
	"math"
	"sort"

	"gonum.org/v1/gonum/mat"
)

// RTS スムーザ (Rauch-Tung-Striebel)。**オフライン専用。**
//
// 実機には真値が無い。平滑化した結果を疑似真値として使うのが計画 §8 の方針で、
// 「オフラインは未来の観測も使えるので、原理的にオンライン推定より必ず良い」。
// **これが無いと実機で精度を測れない** (handoff §6.2 の 2 番目)。
//
// オンラインの Estimator とは別に、専用の前向き経路を持つ。理由は 2 つ:
//
//   - オフラインでは観測を**真の時刻順に並べ直せる**ので、OOSM の巻き戻しが要らない。
//     再フィルタが無いぶん、各ステップの (事前, 事後, 遷移行列) をそのまま記録できる。
//   - オンラインの経路を平滑化のために改造すると、ホットパスのアロケーションゼロ
//     という約束を壊す。
//
// 使い方は cmd/loc_replay を見ること。

// SmootherInput は 1 つの観測。時刻順に並べて渡す。
type SmootherInput struct {
	Stamp Stamp
	// Wheels は論理輪番号の順の角速度 [rad/s]。HasWheels が true のときだけ使う。
	Wheels    [NumWheels]float64
	HasWheels bool
	// GyroZ はヨーレート [rad/s]。HasGyro が true のときだけ使う。
	GyroZ   float64
	HasGyro bool
	// Vision は絶対姿勢。HasVision が true のときだけ使う。
	Vision    Pose2
	HasVision bool
}

// SmoothedState は平滑化した 1 時刻の状態。
type SmoothedState struct {
	Stamp Stamp
	Pose  Pose2
	// VelBody はロボット系の並進速度 [m/s]。
	VelBody Vec2
	YawRate float64
	Slip    Vec2
	Params  KinematicParams
	// CovPose は [x, y, theta] の平滑化後の共分散。
	CovPose Mat3
}

// smootherStep は 1 ステップぶんの記録。
type smootherStep struct {
	stamp Stamp
	// f は 1 つ前から今へ進める遷移行列。
	f [stateDim][stateDim]float64
	// prior / priorP は予測直後 (観測を当てる前) の状態と共分散。
	prior  nominal
	priorP [stateDim][stateDim]float64
	// post / postP は観測を当てた後。
	post  nominal
	postP [stateDim][stateDim]float64
}

// Smooth は観測列を前向きに通したあと、後ろ向きに平滑化した状態列を返す。
//
// inputs は時刻順でなくてもよい (内部で並べ替える)。**vision の時刻は
// 呼び出し側で補正済みのものを渡すこと** (VisionDelayComp 相当)。
func Smooth(inputs []SmootherInput, cfg Config) ([]SmoothedState, error) {
	if len(inputs) < 2 {
		return nil, fmt.Errorf("smooth: need at least 2 inputs, got %d", len(inputs))
	}
	sorted := append([]SmootherInput(nil), inputs...)
	sort.SliceStable(sorted, func(i, j int) bool { return sorted[i].Stamp < sorted[j].Stamp })

	f, err := newESKF(cfg)
	if err != nil {
		return nil, err
	}
	// 機体パラメータの凍結はオフラインでは使わない。全区間を一度に見るので、
	// 不可観測な区間があっても前後の情報で決まる。
	f.paramsFrozen = false

	// 最初の vision で姿勢を立ち上げる。
	for _, in := range sorted {
		if in.HasVision {
			f.setPose(in.Vision)
			n := cfg.Noise
			f.P[idxPx][idxPx] = n.VisionPosNoise * n.VisionPosNoise
			f.P[idxPy][idxPy] = n.VisionPosNoise * n.VisionPosNoise
			f.P[idxPhi][idxPhi] = n.VisionAngNoise * n.VisionAngNoise
			break
		}
	}

	steps := make([]smootherStep, 0, len(sorted))
	prev := sorted[0].Stamp
	for _, in := range sorted {
		dt := in.Stamp.Sub(prev).Seconds()
		var st smootherStep
		st.stamp = in.Stamp
		st.f = f.predictRecording(dt)
		st.prior = f.x
		st.priorP = f.P

		if in.HasWheels {
			f.updateWheels(in.Wheels)
		}
		if in.HasGyro {
			f.updateGyro(in.GyroZ)
		}
		if in.HasVision {
			f.updateVision(in.Vision)
		}
		st.post = f.x
		st.postP = f.P
		steps = append(steps, st)
		prev = in.Stamp
	}

	// --- 後ろ向きの再帰 ---
	//
	//	C_k    = P_k F_{k+1}^T (P_{k+1}^-)^{-1}
	//	x_k^s  = x_k (+) C_k ( x_{k+1}^s (-) x_{k+1}^- )
	//	P_k^s  = P_k + C_k ( P_{k+1}^s - P_{k+1}^- ) C_k^T
	//
	// (+) / (-) は多様体上の合成と差 (姿勢だけ折り返しが要る)。
	out := make([]SmoothedState, len(steps))
	smoothX := make([]nominal, len(steps))
	smoothP := make([][stateDim][stateDim]float64, len(steps))

	last := len(steps) - 1
	smoothX[last] = steps[last].post
	smoothP[last] = steps[last].postP

	pm := mat.NewDense(stateDim, stateDim, nil)
	fm := mat.NewDense(stateDim, stateDim, nil)
	cm := mat.NewDense(stateDim, stateDim, nil)
	inv := mat.NewDense(stateDim, stateDim, nil)

	for k := last - 1; k >= 0; k-- {
		next := &steps[k+1]
		for i := 0; i < stateDim; i++ {
			for j := 0; j < stateDim; j++ {
				pm.Set(i, j, next.priorP[i][j])
				fm.Set(i, j, next.f[i][j])
			}
		}
		if err := inv.Inverse(pm); err != nil {
			// 予測共分散が特異。ここから先は平滑化できないので、
			// フィルタの結果をそのまま使う。
			for j := k; j >= 0; j-- {
				smoothX[j] = steps[j].post
				smoothP[j] = steps[j].postP
			}
			break
		}
		// C = P_k F^T (P^-)^{-1}
		pk := mat.NewDense(stateDim, stateDim, nil)
		for i := 0; i < stateDim; i++ {
			for j := 0; j < stateDim; j++ {
				pk.Set(i, j, steps[k].postP[i][j])
			}
		}
		tmp := mat.NewDense(stateDim, stateDim, nil)
		tmp.Mul(pk, fm.T())
		cm.Mul(tmp, inv)

		// dx = x_{k+1}^s (-) x_{k+1}^-
		dx := boxMinus(smoothX[k+1], next.prior)
		var corr [stateDim]float64
		for i := 0; i < stateDim; i++ {
			var sum float64
			for j := 0; j < stateDim; j++ {
				sum += cm.At(i, j) * dx[j]
			}
			corr[i] = sum
		}
		smoothX[k] = boxPlus(steps[k].post, corr, cfg)

		// P^s = P + C (P^s_{k+1} - P^-_{k+1}) C^T
		var diff [stateDim][stateDim]float64
		for i := 0; i < stateDim; i++ {
			for j := 0; j < stateDim; j++ {
				diff[i][j] = smoothP[k+1][i][j] - next.priorP[i][j]
			}
		}
		var ct [stateDim][stateDim]float64
		for i := 0; i < stateDim; i++ {
			for j := 0; j < stateDim; j++ {
				var sum float64
				for a := 0; a < stateDim; a++ {
					sum += cm.At(i, a) * diff[a][j]
				}
				ct[i][j] = sum
			}
		}
		res := steps[k].postP
		for i := 0; i < stateDim; i++ {
			for j := 0; j < stateDim; j++ {
				var sum float64
				for a := 0; a < stateDim; a++ {
					sum += ct[i][a] * cm.At(j, a)
				}
				res[i][j] += sum
			}
		}
		// 対称化。数値誤差で非対称になると共分散として使えない。
		for i := 0; i < stateDim; i++ {
			for j := i + 1; j < stateDim; j++ {
				m := 0.5 * (res[i][j] + res[j][i])
				res[i][j] = m
				res[j][i] = m
			}
		}
		smoothP[k] = res
	}

	idx := [3]int{idxPx, idxPy, idxPhi}
	for k := range steps {
		x := smoothX[k]
		var cov Mat3
		for i := 0; i < 3; i++ {
			for j := 0; j < 3; j++ {
				cov[i][j] = smoothP[k][idx[i]][idx[j]]
			}
		}
		out[k] = SmoothedState{
			Stamp:   steps[k].stamp,
			Pose:    x.Pose(),
			VelBody: x.VelBody(),
			YawRate: x.Omega,
			Slip:    x.Slip,
			CovPose: cov,
			Params: KinematicParams{
				TransScale: x.Kv, RotScale: x.Kw, AngleBias: x.Ka,
			},
		}
	}
	return out, nil
}

// boxMinus は 2 つの公称状態の差を誤差状態として返す。姿勢だけ折り返す。
func boxMinus(a, b nominal) [stateDim]float64 {
	var d [stateDim]float64
	d[idxPhi] = AngleDiff(a.Theta, b.Theta)
	d[idxPx] = a.P.X - b.P.X
	d[idxPy] = a.P.Y - b.P.Y
	d[idxVx] = a.V.X - b.V.X
	d[idxVy] = a.V.Y - b.V.Y
	d[idxOmega] = a.Omega - b.Omega
	d[idxSx] = a.Slip.X - b.Slip.X
	d[idxSy] = a.Slip.Y - b.Slip.Y
	d[idxKv] = a.Kv - b.Kv
	d[idxKw] = a.Kw - b.Kw
	d[idxKa] = a.Ka - b.Ka
	d[idxBg] = a.GyroBias - b.GyroBias
	return d
}

// boxPlus は公称状態に誤差状態を合成する。
func boxPlus(x nominal, d [stateDim]float64, cfg Config) nominal {
	out := x
	out.Theta = WrapAngle(x.Theta + d[idxPhi])
	out.P.X += d[idxPx]
	out.P.Y += d[idxPy]
	out.V.X += d[idxVx]
	out.V.Y += d[idxVy]
	out.Omega += d[idxOmega]
	out.Slip.X += d[idxSx]
	out.Slip.Y += d[idxSy]
	n := cfg.Noise
	out.Kv = clamp(x.Kv+d[idxKv], 1-n.ParamScaleLimit, 1+n.ParamScaleLimit)
	out.Kw = clamp(x.Kw+d[idxKw], 1-n.ParamScaleLimit, 1+n.ParamScaleLimit)
	lim := n.ParamAngleLimitDeg * math.Pi / 180
	out.Ka = clamp(x.Ka+d[idxKa], -lim, lim)
	out.GyroBias = x.GyroBias + d[idxBg]
	return out
}
