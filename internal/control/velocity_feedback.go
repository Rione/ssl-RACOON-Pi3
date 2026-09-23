package control

import (
	"fmt"
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// TODO(integration): 将来のセンサー受信・フィードバック接続メモ（以下は未接続）。
//
// STMのエンコーダ/IMU -> 受信・デコード ─┐
// SSL-Vision -> locadapterのVision受信 ─┴-> カルマンフィルタ
//   -> localization.Estimate -> Correct / CalculateWithFeedback -> STMへの速度指令
//
// センサー受信とフィルタ側で追加/確認すること:
//   - STM受信は既存のstmframe/locadapterを利用し、車輪順序・符号・半径・
//     スケール・観測時刻を実機仕様に合わせる。車輪角速度をbody並進速度と混同しない。
//   - 現行rock5a-v1にはIMUがない。rock5a-v2-imuの定義だけで実測値が来るとは
//     判断せず、STMファームの配置/単位/有効フラグが確定してから有効化する。
//     欠測IMUを0の観測としてフィルタへ渡さない。
//   - エンコーダ・IMU・ビジョンを観測時刻付きでフィルタへ渡す。センサー単位で
//     時刻同期、遅延、欠測を扱い、フィルタからPose/VelBody/YawRate/Stamp/Healthを公開する。
//     制御側でIMU加速度を別途積分したり、生の車輪速度を二重に加算したりしない。
//   - フィルタはEstimate全体を一括公開し、制御側は1周期に1スナップショットを使う。
//     予測のみでStampが更新される場合も観測欠落をHealthに反映する。
//     Correctの時刻チェックだけでは各センサーの欠測は分からない。
//
// 制御ループと診断送信側で追加すること:
//   - RAVEN直接速度はCorrect、経路追従はCalculateWithFeedbackのどちらか一方。
//     目標は必ず補正前の値。前周期の補正出力や両モードの補正を重ねない。
//   - 停止/通信期限切れを先に判定する。古いRAVEN指令の有効期限はCorrectの
//     MaxEstimateAgeとは別に管理する。最終STM送信の扱いはcontroller.goのメモを参照。
//   - RAVENへの診断候補: 指令元/計画ID、補正前目標、推定速度、速度誤差、
//     補正後指令、最終STM送信値、制限の適用有無、Health、推定時刻、送信seq。
//     PiToMw拡張や別メッセージの選択・送信周期・単位はプロトコル側で定義する。
//     推定速度と指令速度を別フィールドにし、記録/送信の待ちで制御を止めない。
//   - ゲイン変更や将来PI/PID化する場合は、モータ応答遅延と既存STM制御との
//     相互作用を確認する。積分を追加するなら飽和時の積分抑制、停止/経路・モード切替/
//     欠測時のリセット、座標系の回転を仕様化してから追加する（現在は状態なしのP補正）。
// 現在のHealthDegradedは通常補正。共分散に応じたゲイン低減や観測源ごとの
// フォールバックは未実装であり、将来のフィルタ出力仕様に合わせて決める。

// VelocityFeedbackConfig は速度誤差のP補正設定。
// ゲインは無次元で非負。補正上限は非負、最終速度上限・許容遅延は正。
// ゼロゲインまたはゼロ補正上限で、並進・回転それぞれの補正を無効化できる。
type VelocityFeedbackConfig struct {
	LinearGain           float64
	AngularGain          float64
	MaxLinearCorrection  float64 // 並進補正ベクトルの大きさ [m/s]
	MaxAngularCorrection float64 // 角速度補正の絶対値 [rad/s]
	MaxSpeed             float64 // 最終並進速度の大きさ [m/s]
	MaxYawRate           float64 // 最終角速度の絶対値 [rad/s]
	MaxEstimateAge       time.Duration
}

// VelocityFeedback はエンコーダ・IMU・ビジョンを統合したカルマンフィルタの
// Estimate を使用する。センサーの統合・欠測判定自体はフィルタ側が担当する。
// 状態を保持しないので、古い誤差の蓄積や補正の積み上がりは発生しない。
type VelocityFeedback struct{ config VelocityFeedbackConfig }

func NewVelocityFeedback(cfg VelocityFeedbackConfig) (*VelocityFeedback, error) {
	if !finite(cfg.LinearGain, cfg.AngularGain, cfg.MaxLinearCorrection, cfg.MaxAngularCorrection, cfg.MaxSpeed, cfg.MaxYawRate) ||
		cfg.LinearGain < 0 || cfg.AngularGain < 0 || cfg.MaxLinearCorrection < 0 || cfg.MaxAngularCorrection < 0 ||
		cfg.MaxSpeed <= 0 || cfg.MaxYawRate <= 0 || cfg.MaxEstimateAge <= 0 {
		return nil, fmt.Errorf("invalid velocity feedback configuration")
	}
	return &VelocityFeedback{config: cfg}, nil
}

// Correct は RAVEN またはローカル制御の「補正前」の目標速度を受け取る。
// target と estimate.VelBody は同じロボット座標基準、単位は m/s・rad/s。
// now は推定の鮮度を調べる制御時刻（ロボット単調時計）。送信元の時刻ではない。
// 推定は now 時点が理想。MaxEstimateAge 以内の過去値は近似として許容する。
// 未来予測を使う場合、now 自体をその実行予定時刻とすること。
//
// corrected = target + gain * (target - estimatedVelocity)
// 目標、補正、最終出力を制限する。前周期の補正済み出力を target に戻してはならない。
// 全成分ゼロの目標は停止要求として必ずゼロを返す（逆方向の制動指令は出さない）。
// 無効・古い・未来の推定、演算異常ではゼロ指令とエラーを返す。
// HealthDegraded はフィルタが継続使用可能と判断した値として許容する。
func (f *VelocityFeedback) Correct(target Command, estimate localization.Estimate, now localization.Stamp) (Command, error) {
	if f == nil || f.config.MaxSpeed <= 0 {
		return Command{}, fmt.Errorf("velocity feedback is not initialized")
	}
	if !finite(target.VelBody.X, target.VelBody.Y, target.YawRate) {
		return Command{}, fmt.Errorf("invalid target velocity")
	}
	if target == (Command{}) {
		return Command{}, nil
	}
	if now < 0 || estimate.Stamp < 0 || estimate.Stamp > now || now.Sub(estimate.Stamp) > f.config.MaxEstimateAge ||
		(estimate.Health != localization.HealthOK && estimate.Health != localization.HealthDegraded) ||
		!finite(estimate.VelBody.X, estimate.VelBody.Y, estimate.YawRate) {
		return Command{}, fmt.Errorf("velocity estimate is invalid, stale, or in the future")
	}
	var ok bool
	target.VelBody, ok = limitVector(target.VelBody, f.config.MaxSpeed)
	if !ok {
		return Command{}, fmt.Errorf("target velocity overflow")
	}
	target.YawRate = limitScalar(target.YawRate, f.config.MaxYawRate)
	var delta localization.Vec2
	var dw float64
	if f.config.LinearGain > 0 && f.config.MaxLinearCorrection > 0 {
		delta = localization.Vec2{
			X: f.config.LinearGain * (target.VelBody.X - estimate.VelBody.X),
			Y: f.config.LinearGain * (target.VelBody.Y - estimate.VelBody.Y),
		}
	}
	if f.config.AngularGain > 0 && f.config.MaxAngularCorrection > 0 {
		dw = f.config.AngularGain * (target.YawRate - estimate.YawRate)
	}
	delta, ok = limitVector(delta, f.config.MaxLinearCorrection)
	if !ok || !finite(dw) {
		return Command{}, fmt.Errorf("velocity correction overflow")
	}
	dw = limitScalar(dw, f.config.MaxAngularCorrection)
	out := Command{
		VelBody: localization.Vec2{X: target.VelBody.X + delta.X, Y: target.VelBody.Y + delta.Y},
		YawRate: target.YawRate + dw,
	}
	out.VelBody, ok = limitVector(out.VelBody, f.config.MaxSpeed)
	if !ok || !finite(out.YawRate) {
		return Command{}, fmt.Errorf("corrected velocity overflow")
	}
	out.YawRate = limitScalar(out.YawRate, f.config.MaxYawRate)
	return out, nil
}

// CalculateWithFeedback は経路追従の速度に速度フィードバックを一度適用する。
// 既存の Calculate は位置・姿勢補正のみのAPIとして残す。
// Waiting/Finished はフィードバックを通さずゼロを返す。
// 経路側と速度補正側の上限が異なる場合は小さい方を採用する。
func (c *Controller) CalculateWithFeedback(estimate localization.Estimate, now localization.Stamp, feedback *VelocityFeedback) (Command, Phase, error) {
	target, phase, err := c.Calculate(estimate)
	if err != nil || phase != Tracking {
		return target, phase, err
	}
	command, err := feedback.Correct(target, estimate, now)
	if err != nil {
		return Command{}, phase, err
	}
	command.VelBody, _ = limitVector(command.VelBody, c.config.MaxSpeed)
	command.YawRate = limitScalar(command.YawRate, c.config.MaxYawRate)
	return command, phase, nil
}
