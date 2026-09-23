// Package control は時刻付き経路からロボット系の速度指令を計算する。
// 通信・周期実行・時計同期は呼び出し側が担当する。内部単位は SI。
package control

import (
	"fmt"
	"math"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// TODO(integration): 将来の送受信・周期実行の接続メモ（以下は未実装）。
// 実装済みの計算仕様は docs/local-control.md、通信側の要件案は
// docs/robot-command-protocol-requirements.md を参照。wire形式は未確定であり、
// このGo構造体のフィールド順をパケットのバイト配置として使わないこと。
//
// RAVEN -> internal/receive -> []Node -> New -> 周期ごとのCalculateWithFeedback
//   -> Command -> internal/link -> Pi4 UART / Rock5A SPI -> STM
//
// 受信側で追加すること:
//   - 未来のノード列(vector)をデコードする。各点は vx, vy, theta, x, y, t、
//     任意の目標角速度omegaを持つ。omega欠省時は0（現在は時刻差から導出しない）。
//   - 現在の速度指令DATA(0x06)との互換性を保ち、経路用command_idや
//     プロトコルバージョン、robot_id、seq、計画ID、有効期限を通信側で定義する。
//     ノード数・パケット長の上限も定義し、受信バッファとUDPサイズに整合させる。
//   - 送信/計画生成時刻と各ノードの到達予定時刻を区別する。RAVENの時計を
//     ロボットの単調時計へ同期・変換し、Estimate.Stampと同じ基準でNode.Stampを埋める。
//     到達時刻が相対値なら計画開始時刻との和にする。受信時刻への単純な置換はしない。
//   - 単位をwire仕様に明記してSIへ変換する。mm/mradなら1000で割る。
//     x,yはworld系、vx,vyはそのノードの目標姿勢を基準とするbody系。
//     将来world速度へ変更した場合は受信境界でRotateInv(nodeTheta, velocity)する。
//   - 欠落・重複・順序逆転・送信側再起動を識別し、古い計画で新しい計画を上書きしない。
//     Newの検証に通った経路を制御ループへ一括公開する（atomic.Pointer等）。
//     不正な受信を通信継続の根拠にしない。有効期限切れの旧経路も走らせ続けない。
//
// 実行/送信側で追加すること:
//   - internal/appが制御ループ（想定125 Hz）を所有し、同一時点のEstimateを取得する。
//     nowと推定時刻を整合させ、CalculateWithFeedbackを1周期に一度適用する。
//   - 最終Commandをmm/s・mrad/sへ変換する場合は1000倍し、丸め方を定義して
//     int16の範囲内へ飽和させてからエンコードする。Pi4/Rock5Aの既存配置を使う。
//     速度以外のキック・ドリブル等のフィールドは既存処理と合成する。
//   - 指令モード（RAVEN速度/ローカル経路/停止）の排他と切替を管理する。
//     通信タイムアウト、緊急停止、dry-run、shutdownを補正より優先し、
//     既存の同定用VelocityOverrideが停止を再上書きしないよう接続順を検討する。
//     エラー時のゼロ出力も送信へ反映する（単にreturnして前回指令を残さない）。
//   - 通信欠落時の減速、加速度制限は接続側で定義する。最終ノード以降の
//     現仕様はゼロ指令。位置保持/終端速度維持への変更時はテストと文書も更新する。
// このパッケージにはソケット・ハードウェア依存・time.Now()を持ち込まない。

// Config のゲインは [1/s]、速度上限は [m/s, rad/s]。
// ゲインは非負、上限は正の有限値を指定する。実機に合わせて調整すること。
type Config struct {
	PositionGain float64
	HeadingGain  float64
	MaxSpeed     float64
	MaxYawRate   float64
}

// Command は現在のロボット姿勢を基準とする速度指令。
type Command struct {
	VelBody localization.Vec2 // 前方 +x、左 +y [m/s]
	YawRate float64           // 反時計回りが正 [rad/s]
}

type Phase uint8

const (
	Waiting Phase = iota // 先頭ノードの時刻より前: ゼロ指令
	Tracking
	Finished // 最終ノードの時刻以降: ゼロ指令
)

// Controller は不変の経路と設定を保持する。経路更新時は New で置き換える。
// Calculate は状態を変更せず、複数 goroutine から呼び出せる。
type Controller struct {
	nodes  []Node
	config Config
}

// New は vector に相当するノードのスライスを検証してコピーする。
// 並べ替えは行わず、重複・逆順の到達時刻をエラーにする。
func New(nodes []Node, config Config) (*Controller, error) {
	if len(nodes) < 2 {
		return nil, fmt.Errorf("trajectory requires at least two nodes")
	}
	if !finite(config.PositionGain, config.HeadingGain, config.MaxSpeed, config.MaxYawRate) ||
		config.PositionGain < 0 || config.HeadingGain < 0 || config.MaxSpeed <= 0 || config.MaxYawRate <= 0 {
		return nil, fmt.Errorf("invalid controller gains or velocity limits")
	}
	copyNodes, err := validateNodes(nodes)
	if err != nil {
		return nil, err
	}
	return &Controller{nodes: copyNodes, config: config}, nil
}

// Calculate は推定状態の時刻で経路を評価する。呼び出し側は制御対象時刻の
// Estimate を渡すこと（古い観測の鮮度判定・予測は本関数の範囲外）。
// 位置・速度はワールド系で線形補間し、角度は最短回転で補間する。
// v_world = v_target_world + Kp * (p_target - p_estimate)
// omega = omega_target + Ktheta * wrap(theta_target - theta_estimate)
// 並進指令を現在のロボット系へ変換し、速度の大きさと角速度を制限する。
// 不正な入力ではゼロ指令とエラーを返す。HealthDegraded は使用可能とする。
func (c *Controller) Calculate(estimate localization.Estimate) (Command, Phase, error) {
	if c == nil || len(c.nodes) < 2 {
		return Command{}, Waiting, fmt.Errorf("controller is not initialized")
	}
	if estimate.Stamp < 0 || (estimate.Health != localization.HealthOK && estimate.Health != localization.HealthDegraded) ||
		!finite(estimate.Pose.X, estimate.Pose.Y, estimate.Pose.Theta) {
		return Command{}, Waiting, fmt.Errorf("invalid pose estimate")
	}
	ref, phase := sampleTrajectory(c.nodes, estimate.Stamp)
	if phase != Tracking {
		return Command{}, phase, nil
	}
	x, y, theta := ref.Pose.X, ref.Pose.Y, ref.Pose.Theta

	v := localization.Vec2{
		X: ref.VelWorld.X + c.config.PositionGain*(x-estimate.Pose.X),
		Y: ref.VelWorld.Y + c.config.PositionGain*(y-estimate.Pose.Y),
	}
	w := ref.YawRate + c.config.HeadingGain*localization.AngleDiff(theta, localization.WrapAngle(estimate.Pose.Theta))
	// 有限入力でも演算がオーバーフローした場合は指令を出さない。
	speed := math.Hypot(v.X, v.Y)
	if !finite(v.X, v.Y, w, speed) {
		return Command{}, Tracking, fmt.Errorf("velocity calculation overflow")
	}
	if speed > c.config.MaxSpeed {
		scale := c.config.MaxSpeed / speed
		v.X *= scale
		v.Y *= scale
	}
	w = math.Max(-c.config.MaxYawRate, math.Min(c.config.MaxYawRate, w))
	return Command{VelBody: localization.RotateInv(estimate.Pose.Theta, v), YawRate: w}, Tracking, nil
}

func finite(values ...float64) bool {
	for _, v := range values {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return false
		}
	}
	return true
}
