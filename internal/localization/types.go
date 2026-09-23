// Package localization はロボット上の自己位置推定コアを提供する。
//
// このパッケージは通信もハードウェアも知らない。ビルドタグを付けないので
// 開発 PC (macOS) でそのまま `go test` が回る。
//
// 設計上の約束 (docs/self-localization-plan.md §7.4):
//   - このパッケージは time.Now() を呼ばない。時刻はすべて Stamp として引数で受ける。
//   - 乱数を使わない。同じ入力からは必ず同じ出力になる。
//
// 単位は内部では SI (m, m/s, rad, rad/s) のみを使う。mm への変換は境界でのみ行う。
package localization

import "time"

// Stamp はプロセス起動時刻を原点とする単調時刻 (CLOCK_MONOTONIC 由来)。
// 壁時計とは無関係で、NTP の step でも飛ばない。
//
// time.Time を使わないのは、単調時計の読み値が JSON/gob へのマーシャルや
// Round(0) で落ちるため (計画 §5.2)。API 境界では常に Stamp を使う。
type Stamp time.Duration

// Sub は 2 つの Stamp の差を返す。
func (s Stamp) Sub(other Stamp) time.Duration { return time.Duration(s - other) }

// Add は Stamp に経過時間を加える。
func (s Stamp) Add(d time.Duration) Stamp { return s + Stamp(d) }

// Seconds は原点からの経過秒数を返す。
func (s Stamp) Seconds() float64 { return time.Duration(s).Seconds() }

// Nanoseconds は原点からの経過ナノ秒を返す。MCAP のログ時刻に使う。
func (s Stamp) Nanoseconds() int64 { return int64(s) }

// Vec2 は 2 次元ベクトル。文脈によって位置 [m] か速度 [m/s] を表す。
type Vec2 struct {
	X float64
	Y float64
}

// Pose2 は平面上の姿勢。位置 [m] と向き [rad]。
type Pose2 struct {
	X     float64
	Y     float64
	Theta float64
}

// Mat3 は 3x3 の共分散行列。行優先。
// gonum を型の表面に出さないことで、呼び出し側がアロケーションを強いられないようにする。
type Mat3 [3][3]float64

// Health は推定の健全性。RAVEN 側のフォールバック判断に直結する
// (docs/robot-command-protocol-requirements.md §4)。
type Health uint8

const (
	// HealthOK は vision が取り込めていて共分散も正常な状態。
	HealthOK Health = iota
	// HealthDegraded は vision の欠落が続くなど、精度が落ちているが推定は継続している状態。
	HealthDegraded
	// HealthInvalid はフィルタが発散した、NaN が出たなど推定を信用してはいけない状態。
	// RAVEN はこのとき vision 生値へフォールバックする。
	HealthInvalid
)

func (h Health) String() string {
	switch h {
	case HealthOK:
		return "OK"
	case HealthDegraded:
		return "DEGRADED"
	case HealthInvalid:
		return "INVALID"
	default:
		return "UNKNOWN"
	}
}

// NumWheels は 4 輪オムニの車輪数。
const NumWheels = 4

// 車輪のインデックス。SPI フレーム上の並び (FL, BL, BR, FR) と一致させる。
// 出典: ssl-Circuit/2026/Firmware/MainBoard/MainBoard_V26_1_1/src/unit/robot.c:158
//
// 注意: この並びが実機と合っている保証はまだない。計画 §12 A-4 の通り、
// 世代間で FL / FR が入れ替わっている疑いがあり、P1 の同定で確定させる。
const (
	WheelFL = 0
	WheelBL = 1
	WheelBR = 2
	WheelFR = 3
)

// WheelSample は 1 周期分の 4 輪角速度 [rad/s] (車輪軸)。
//
// Stamp は「STM がセンサを読んだ推定時刻」であり、SPI の転送時刻から
// プロファイルの TimeOffset を引いたもの (計画 §5.2)。
type WheelSample struct {
	Stamp Stamp
	Omega [NumWheels]float64
}

// ImuSample はジャイロ・加速度の 1 サンプル。
//
// 計画 §3.5 の通り STM 側に IMU の実装が存在しないため、当面 Valid は false になる。
// 定義されていないフィールドはセンサ非搭載として扱い、コアはその観測を単に使わない。
type ImuSample struct {
	Stamp Stamp
	// GyroZ はヨーレート [rad/s]。反時計回りが正。
	GyroZ float64
	// Accel はロボット系の加速度 [m/s^2]。前方 +x、左 +y。
	Accel Vec2
	// HasGyro / HasAccel はそのフィールドが実際に届いたかを示す。
	HasGyro  bool
	HasAccel bool
}

// VisionPose は SSL-Vision の 1 観測。
type VisionPose struct {
	// Stamp は Rock5A の時間軸へ写像済みの露光時刻。timesync が埋める。
	Stamp Stamp
	// Arrival は UDP を受信した Rock5A 時刻。timesync の入力かつフォールバック。
	Arrival Stamp
	// TCapture / TSent は vision PC のクロックによる秒 (パケットの生値)。
	TCapture float64
	TSent    float64

	Pose       Pose2
	Confidence float64

	CameraID    uint32
	FrameNumber uint32

	// Mapped は timesync による写像が成功したかを示す。false なら Arrival で代用している。
	Mapped bool
}

// Estimate は推定器の出力。この構造体がそのまま RAVEN への返信内容になる
// (計画 §7.6 / §3.8 の決定事項)。
type Estimate struct {
	// Stamp はこの推定が指す時刻。PredictAhead を使った場合は未来時刻になる。
	Stamp Stamp
	// Pose はワールド系 [m, m, rad]。SSL-Vision 準拠 (フィールド中央原点、右手系)。
	Pose Pose2
	// VelBody はロボット系の並進速度 [m/s]。前方 +x、左 +y。
	VelBody Vec2
	// YawRate はヨーレート [rad/s]。
	YawRate float64
	// CovPose は [x, y, theta] の共分散。位置制御がゲインを落とす判断に使う。
	CovPose Mat3
	// Slip は推定したスリップ速度 [m/s]。診断にも制御にも使える。
	Slip Vec2
	// GyroBias は推定したジャイロのゼロ点のずれ [rad/s] (診断用)。ジャイロが無ければ 0。
	GyroBias float64
	// SinceVision は最後に有効な vision 観測を取り込んでからの経過時間。
	SinceVision time.Duration
	// Health は推定の健全性。
	Health Health
}
