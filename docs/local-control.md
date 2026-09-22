# ノード列からの速度計算

`internal/control` は、未来の経路点 `[]control.Node` と現在の
`localization.Estimate` からロボット系速度を計算する独立パッケージです。
受信処理、制御ループ、基板出力にはまだ接続していません。

## 入力の約束

- `Node.Pose`: ワールド座標の m、rad。
- `Node.VelBody`: **そのノードの目標姿勢**に対する前方・左方向の目標速度 m/s。
- `Node.YawRate`: 目標角速度 rad/s。
- `Node.Stamp`: 各点への到達予定時刻。計画の生成時刻・送信時刻とは別です。
  呼び出し側でロボットの単調時間軸へ変換してください。RAVEN の時計値をそのまま渡せません。
- ノードは2点以上で、到達時刻は厳密な昇順。経路は `New` でコピーされます。
- `Estimate.Stamp`: 計算対象時刻。`Estimate.Pose` も同じ時刻の推定姿勢を渡します。
  内部で現在時刻を取得したり、観測を未来へ予測したりはしません。
- 通信要件書の mm・mrad・mm/s・mrad/s は、受信境界で1000で割って SI に変換します。

## 計算

時刻を挟む2ノードを選び、位置・ワールド系へ変換した目標速度・角速度を線形補間します。
姿勢は ±π を跨ぐ場合も最短回転で補間します（1区間で半回転を超える回転は表せません）。

```text
v_world = interpolated_target_velocity_world + PositionGain * position_error_world
omega   = interpolated_target_yaw_rate + HeadingGain * wrapped_heading_error
v_body  = rotate(-current_theta, v_world)
```

並進速度の大きさを `MaxSpeed`、角速度を `MaxYawRate` で制限します。
目標速度は入力値を使い、位置の差分から再計算しません。ゼロも有効な入力なので、
ノードの位置・時刻・速度が計画として整合することは送信側の責任です。

先頭時刻より前は `Waiting`、最終時刻以降は `Finished` とゼロ指令を返します。
終端で位置保持や非ゼロ速度の継続はしません。滑らかに停止するには経路内に減速区間を含めます。
不正入力や `HealthInvalid` はゼロ指令とエラー、`HealthDegraded` は通常計算です。

## 使用例

```go
// nodes の時刻と単位は変換済み。数値は説明用であり実機調整値ではない。
controller, err := control.New(nodes, control.Config{
    PositionGain: 2, HeadingGain: 3,
    MaxSpeed: 1, MaxYawRate: 2,
})
if err != nil {
    return err
}
command, phase, err := controller.Calculate(estimate)
// command.VelBody [m/s], command.YawRate [rad/s] を呼び出し側へ返す。
// 経路が更新されたら新しい Controller を作成して置き換える。
```

`Calculate` 単体の推定値の鮮度判定、時計同期、通信欠落・緊急停止、加速度制限、
125 Hzの周期実行は接続側の担当です。本パッケージ単体では通信断を判定できません。
今回のテストは計算と理想速度応答での追従を確認するもので、実機の安定性を保証するものではありません。

```sh
go test ./internal/control
```

## 速度フィードバック

`velocity_feedback.go` は、指令どおりに動いているかをカルマンフィルタの推定速度と
比較して補正します。エンコーダ、IMU、ビジョンの生データはフィルタへ入力し、
制御側はその出力 `Estimate.VelBody` と `Estimate.YawRate` を使用します。
センサー融合やSTM受信の実装を本パッケージに重複して追加するものではありません。

```text
エンコーダ ─┐
IMU ──────┼→ カルマンフィルタ → Estimate ─────────────┐
ビジョン ──┘                                        ↓
RAVENの速度指令 または ローカルのCalculate → 速度フィードバック → 補正指令
```

```text
delta_v = LinearGain * (target.VelBody - estimate.VelBody)
delta_w = AngularGain * (target.YawRate - estimate.YawRate)
output  = target + bounded_correction
```

- ゲインは無次元。位置補正用の `PositionGain` / `HeadingGain` とは別設定です。
- 並進補正はベクトルの大きさ、回転補正は絶対値に上限を設けます。
  最終出力も `MaxSpeed` / `MaxYawRate` で制限します。
- 積分項はありません。動かなくても補正が周期ごとに増え続けることはありません。
  P補正のため定常偏差が完全になくなるとは限りません。
- 全成分ゼロの指令は停止要求としてゼロを維持します。
  一部の成分だけゼロの場合、その軸の横流れや不要な回転は補正します。
- `HealthInvalid`、不明なHealth、NaN/Inf、古すぎる推定、未来時刻の推定は
  ゼロ指令とエラーを返します。`HealthDegraded` は使用可能とします。
- `now` と `Estimate.Stamp` は同じロボットの単調時間軸です。
  推定は制御時刻 `now` に一致するのが理想で、`MaxEstimateAge` 以内の過去値は近似として許容します。
  この処理自体は観測遅延・モータ応答遅延を補償しません。
  将来の実行時刻へ予測した推定を使うときは `now` もその実行時刻にそろえます。
- `Estimate.Stamp` は状態の時刻であり、各センサーが新鮮かどうかはフィルタ側で判定して
  Health に反映してください。予測だけ更新し続けた状態の観測欠落は、時刻チェックだけでは検出できません。

```go
// 例示値。ゲインと上限は機体の応答に合わせて調整する。
feedback, err := control.NewVelocityFeedback(control.VelocityFeedbackConfig{
    LinearGain: 0.5, AngularGain: 0.5,
    MaxLinearCorrection: 0.3, MaxAngularCorrection: 0.4,
    MaxSpeed: 1, MaxYawRate: 2,
    MaxEstimateAge: 40 * time.Millisecond,
})
if err != nil {
    return err
}

// ローカル経路追従の場合（位置補正→速度補正）。
command, phase, err := controller.CalculateWithFeedback(estimate, now, feedback)

// RAVENから直接速度を受ける場合は、代わりにこちらを呼ぶ。
// ravenTarget はSI単位・現在のロボット座標に変換した補正前の指令。
command, err = feedback.Correct(ravenTarget, estimate, now)
```

この2つは指令元に応じて選択し、同じ指令へ二重に適用しないでください。
前周期の補正済み指令を次周期の目標に戻すこともしません。
`CalculateWithFeedback` は経路側・補正側のうち小さい速度上限を守り、
`Waiting` / `Finished` では補正を行いません。

カルマンフィルタとの実行時接続、通信指令の有効期限、緊急停止、STMへの出力は未接続です。
現行STMの既定プロファイル `rock5a-v1` は車輪速度のみで、IMU融合には
ファームウェアからの実測IMUデータとフィルタ側の対応が必要です。
