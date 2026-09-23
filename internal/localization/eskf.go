package localization

import "math"

// 誤差状態カルマンフィルタ (部分不変 / PIEKF)。
//
// 状態の持ち方 (計画 §4.2):
//
//	公称状態   theta (ワールド姿勢), p (ワールド位置), V (**ワールド**速度),
//	           omega (ヨーレート), s (ロボット系スリップ速度)
//	誤差状態   [dPhi, dP(2), dV(2), dOmega, dS(2)]  = 8 次元
//
// **速度をワールド系で持つのが部分不変の肝である。** 位置の誤差伝播が
//
//	d(dP)/dt = dV
//
// となり、**推定値 (theta や V) に一切依存しない**。ロボット系で持つと
// d(dP)/dt = R(theta)*dV + J*R(theta)*V*dPhi となって推定軌道に依存し、
// 共分散が推定の出来不出来に引きずられる。位置は誤差が一番育つ経路なので、
// ここを推定非依存にすることが一貫性に効く。
//
// 姿勢は SO(2) なので左不変と右不変が一致する (2 次元の回転は可換)。
// 誤差の注入も単なる加算 + 折り返しで済み、共分散のリセット変換も恒等になる。
//
// omega は入力ではなく状態として持つ (計画 §4.2 の 10 次元の形とは違う)。
// ジャイロが載った機体 (MainBoard_V26_2 以降) では、ジャイロを「omega の観測」として当て、
// そのバイアス b_omega を状態に持つ。こうすると車輪・vision と同じ枠に収まり、
// ジャイロの有無を設定だけで切り替えて効果を比べられる (docs/imu-requirements.md)。

// 誤差状態のインデックス。
const (
	idxPhi   = 0
	idxPx    = 1
	idxPy    = 2
	idxVx    = 3
	idxVy    = 4
	idxOmega = 5
	idxSx    = 6
	idxSy    = 7
	idxBw    = 8 // ジャイロのバイアス [rad/s]

	stateDim = 9
	// maxObs は観測の最大次元。車輪が 4、vision が 3。
	maxObs = 4
)

// nominal は公称状態。誤差状態とは別に、物理量としてそのまま保持する。
type nominal struct {
	Theta float64
	P     Vec2
	// V はワールド系の並進速度 [m/s]。ロボット系ではない。
	V     Vec2
	Omega float64
	// Slip はロボット系のスリップ速度 [m/s]。
	Slip Vec2
	// BiasOmega はジャイロのゼロ点のずれ [rad/s]。ジャイロを使わない間は 0 のまま。
	BiasOmega float64
}

// VelBody はロボット系の並進速度を返す。
func (n nominal) VelBody() Vec2 { return RotateInv(n.Theta, n.V) }

// Pose はワールド系の姿勢を返す。
func (n nominal) Pose() Pose2 { return Pose2{X: n.P.X, Y: n.P.Y, Theta: n.Theta} }

// eskf は誤差状態フィルタ本体。
//
// goroutine 安全ではない。単一の goroutine からのみ触ること (計画 §7.2)。
// ホットパス (predict / update) ではアロケートしない。
type eskf struct {
	cfg Config
	kin *Kinematics

	x nominal
	P [stateDim][stateDim]float64

	// 以下はホットパスの作業領域。起動時に確保して使い回す。
	h    [maxObs][stateDim]float64
	nu   [maxObs]float64
	rdia [maxObs]float64
	ph   [stateDim][maxObs]float64 // P * H^T
	s    [maxObs][maxObs]float64
	s2   [maxObs][maxObs]float64
	k    [stateDim][maxObs]float64
	ikh  [stateDim][stateDim]float64
	tmp  [stateDim][stateDim]float64
	dx   [stateDim]float64
	sol  [maxObs]float64
}

func newESKF(cfg Config) (*eskf, error) {
	if err := cfg.Validate(); err != nil {
		return nil, err
	}
	kin, err := NewKinematics(cfg.Geometry)
	if err != nil {
		return nil, err
	}
	f := &eskf{cfg: cfg, kin: kin}
	f.resetCovariance()
	return f, nil
}

func (f *eskf) resetCovariance() {
	n := f.cfg.Noise
	f.P = [stateDim][stateDim]float64{}
	f.P[idxPhi][idxPhi] = n.InitAngVar
	f.P[idxPx][idxPx] = n.InitPosVar
	f.P[idxPy][idxPy] = n.InitPosVar
	f.P[idxVx][idxVx] = n.InitVelVar
	f.P[idxVy][idxVy] = n.InitVelVar
	f.P[idxOmega][idxOmega] = n.InitOmegaVar
	if n.EnableSlip {
		f.P[idxSx][idxSx] = n.InitSlipVar
		f.P[idxSy][idxSy] = n.InitSlipVar
	}
	f.P[idxBw][idxBw] = n.InitGyroBiasVar
}

// setPose は公称状態を与えられた姿勢で初期化する。
func (f *eskf) setPose(pose Pose2) {
	f.x.Theta = WrapAngle(pose.Theta)
	f.x.P = Vec2{X: pose.X, Y: pose.Y}
}

// predict は dt 秒ぶん状態と共分散を進める。
//
// dt は SPI 転送時刻の差分の実測値を使う。公称 8 ms は使わない (計画 §5.2)。
func (f *eskf) predict(dt float64) {
	if dt <= 0 {
		return
	}
	n := f.cfg.Noise
	alpha := f.x.Omega * dt
	// sin(a)/a と (1-cos(a))/a。ExpSE2 と同じ V 行列。
	a, b := seriesAB(alpha)

	// --- 公称状態 ---
	//
	// ロボット系速度と omega が一定なら、位置は閉じた式で厳密に積める。
	// V はワールド系なので、機体が回るぶんだけ回転する。
	sa, ca := math.Sincos(alpha)
	prevV := f.x.V
	f.x.P.X += (a*prevV.X - b*prevV.Y) * dt
	f.x.P.Y += (b*prevV.X + a*prevV.Y) * dt
	f.x.V = Vec2{X: ca*prevV.X - sa*prevV.Y, Y: sa*prevV.X + ca*prevV.Y}
	f.x.Theta = WrapAngle(f.x.Theta + alpha)

	slipDecay := 1.0
	if n.EnableSlip {
		slipDecay = math.Exp(-dt / n.SlipTau)
		f.x.Slip.X *= slipDecay
		f.x.Slip.Y *= slipDecay
	} else {
		f.x.Slip = Vec2{}
	}

	// --- 遷移行列 F ---
	var F [stateDim][stateDim]float64
	for i := range F {
		F[i][i] = 1
	}
	// dPhi' = dPhi + dOmega*dt
	F[idxPhi][idxOmega] = dt

	// dP' = dP + Vmat(alpha)*dV*dt
	//
	// dV は omega で回りながら積分されるので、素朴な I*dt ではなく
	// ExpSE2 と同じ V 行列を使う。
	F[idxPx][idxVx] = a * dt
	F[idxPx][idxVy] = -b * dt
	F[idxPy][idxVx] = b * dt
	F[idxPy][idxVy] = a * dt

	// dV' = R(alpha)*dV + (J*V)*dOmega*dt
	F[idxVx][idxVx] = ca
	F[idxVx][idxVy] = -sa
	F[idxVy][idxVx] = sa
	F[idxVy][idxVy] = ca
	// d/domega (R(omega*dt)*V) = dt * J * R(alpha) * V。J = [[0,-1],[1,0]]。
	F[idxVx][idxOmega] = -dt * f.x.V.Y
	F[idxVy][idxOmega] = dt * f.x.V.X
	// dOmega が dV 経由で dP へ効く二次の項。
	F[idxPx][idxOmega] = -0.5 * dt * dt * prevV.Y
	F[idxPy][idxOmega] = 0.5 * dt * dt * prevV.X

	if n.EnableSlip {
		F[idxSx][idxSx] = slipDecay
		F[idxSy][idxSy] = slipDecay
	} else {
		F[idxSx][idxSx] = 0
		F[idxSy][idxSy] = 0
	}

	// --- P = F P F^T + Q ---
	f.propagateCovariance(&F)
	f.addProcessNoise(dt)
	f.symmetrize()
}

// propagateCovariance は P <- F P F^T を計算する。
func (f *eskf) propagateCovariance(F *[stateDim][stateDim]float64) {
	// tmp = F * P
	for i := 0; i < stateDim; i++ {
		for j := 0; j < stateDim; j++ {
			var sum float64
			for k := 0; k < stateDim; k++ {
				if F[i][k] != 0 {
					sum += F[i][k] * f.P[k][j]
				}
			}
			f.tmp[i][j] = sum
		}
	}
	// P = tmp * F^T
	for i := 0; i < stateDim; i++ {
		for j := 0; j < stateDim; j++ {
			var sum float64
			for k := 0; k < stateDim; k++ {
				if F[j][k] != 0 {
					sum += f.tmp[i][k] * F[j][k]
				}
			}
			f.P[i][j] = sum
		}
	}
}

// addProcessNoise は Q を足す。
//
// (位置, 速度) と (姿勢, ヨーレート) はそれぞれ二重積分の鎖なので、
// 白色の加速度雑音を連続時間から離散化した形 (dt^3/3, dt^2/2, dt) を使う。
// 対角だけに dt を足す素朴な近似では、共分散が小さく出て NEES が悪化する。
func (f *eskf) addProcessNoise(dt float64) {
	n := f.cfg.Noise
	dt2 := dt * dt
	dt3 := dt2 * dt

	qa := n.AccelNoise * n.AccelNoise
	f.P[idxPx][idxPx] += qa * dt3 / 3
	f.P[idxPy][idxPy] += qa * dt3 / 3
	f.P[idxVx][idxVx] += qa * dt
	f.P[idxVy][idxVy] += qa * dt
	cross := qa * dt2 / 2
	f.P[idxPx][idxVx] += cross
	f.P[idxVx][idxPx] += cross
	f.P[idxPy][idxVy] += cross
	f.P[idxVy][idxPy] += cross

	qw := n.AngAccelNoise * n.AngAccelNoise
	f.P[idxPhi][idxPhi] += qw * dt3 / 3
	f.P[idxOmega][idxOmega] += qw * dt
	crossW := qw * dt2 / 2
	f.P[idxPhi][idxOmega] += crossW
	f.P[idxOmega][idxPhi] += crossW

	if n.EnableSlip {
		qs := n.SlipNoise * n.SlipNoise * dt
		f.P[idxSx][idxSx] += qs
		f.P[idxSy][idxSy] += qs
	}

	// ジャイロのバイアスはゆっくり歩く (温度で動く)。
	f.P[idxBw][idxBw] += n.GyroBiasNoise * n.GyroBiasNoise * dt
}

func (f *eskf) symmetrize() {
	for i := 0; i < stateDim; i++ {
		for j := i + 1; j < stateDim; j++ {
			m := 0.5 * (f.P[i][j] + f.P[j][i])
			f.P[i][j] = m
			f.P[j][i] = m
		}
	}
}

// seriesAB は sin(x)/x と (1-cos(x))/x を返す。微小域はテイラー展開に落とす。
func seriesAB(x float64) (a, b float64) {
	if math.Abs(x) < se2Eps {
		return 1 - x*x/6, x/2 - x*x*x/24
	}
	s, c := math.Sincos(x)
	return s / x, (1 - c) / x
}
