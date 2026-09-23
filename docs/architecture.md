# ローカル制御のフォルダ構成

既存の Raspberry Pi 4B / Rock5A の起動・通信機能を残し、計算と通信を分離する。
本変更は構造整理と計算周期の接続点の追加。実機の Run は従来経路のまま。

```text
internal/
  app/
    run.go                 既存起動・サービスの組立て
    platform_*.go          ボード別の組立て
    localization.go        既存の観測記録・Vision受信の起動
    control_loop.go        観測→推定→期限判定→速度計算の1周期（未接続）
    snapshot.go            計算結果を送信側へ渡す値コピー
  receive/
    receive.go             既存の速度指令受信
    camera.go              オンボードカメラのボール検出受信
    trajectory.go          時刻変換済みノード列の検証・所有（wire形式ではない）
  locadapter/
    vision.go              SSL-Visionの受信・時刻変換
    spi_recorder.go         既存SPI観測記録
    wheel.go               車輪観測の変換
    imu.go                 プロファイルからIMU観測への変換
  stmframe/                STM通信フレームの検証・デコード
  localization/            カルマンフィルタ・車輪運動学・遅延観測処理
  control/
    trajectory.go          ノード検証・時刻に対応する参照値の補間
    controller.go          参照速度＋位置・姿勢誤差による追従
    velocity_feedback.go   推定速度と指令速度の差による補正
    limits.go              速度・補正量の制限
    diagnostics.go         参照値と推定値の比較
  supervisor/
    mode.go                モード・停止理由の型
    watchdog.go            非常停止・指令期限・推定鮮度の判定
  mw/
    mw.go                  既存MW接続管理
    status.go              既存の車輪などの状態送信
    estimate.go            推定結果・追従誤差・計算指令の送信向け型
  link/                    ボードに依存しない既存通信処理
  pi4/                     UART・GPIO
  rock5a/                  SPI・GPIO
  timesync/                送信側時刻とロボット単調時計の対応
  loclog/                  記録と読み出し
  locsim/                  シミュレーション検証
  state/                   既存経路の共有状態
  api/ upgrade/ util/ wheelgraph/   既存の周辺機能
cmd/ proto/ camera/ scripts/        既存の起動・通信定義・カメラ・運用
```

## データ経路

将来の接続先は次のとおり。矢印全体が現在実機で動くという意味ではない。

```text
STM → pi4(UART) / rock5a(SPI) → stmframe → locadapter ─┐
SSL-Vision → locadapter/vision → timesync ─────────────┤
                                                     ↓
RAVEN → receive/trajectory → app/ControlCycle → localization
                                   ↑               ↓ Estimate
                                   └── control ←───┘
                                         ↓
                         supervisorによる許可判定済みの計算指令
                                         ↓
                       将来: link → pi4 / rock5a → STM

app/ControlCycle → SnapshotStore → 将来: mw → MW → RAVEN
既存の STM → RACOON → MW/RAVEN の車輪状態送信は維持する。
```

自己位置推定は観測から計算し、目標経路で推定位置を上書きしない。
追従誤差と推定状態は別フィールド。EstimateReport.Command は計算結果であり、送信済み指令ではない。

## 計算速度と所有権

- 推定器は計算用の1本の goroutine が所有する。ControlCycle.Step はその1周期で、スケジューラではない。
- 経路のコピー・検証は PreparePlan で受信更新時に行う。周期中は不変の経路を二分探索する。
- Step 内にネットワーク送受信・ログ書込み・待機を入れない。観測配列は到着済みの新しい標本だけを渡す。
- SnapshotStore は短いロック中に値をコピーする。送信・記録は読み出したコピーで行う。
- 周期中の経路評価・スナップショット受渡しは確保を避ける。推定器全体の確保数や実機周期性能は別途測定が必要。
- 容量を制限した受信キュー、観測欠落の監視、実機での処理時間計測は周期駆動を接続するときに実装する。

## 今回の実装範囲と未接続部分

既存の経路補間、速度制御、カメラ受信、状態送信、SPI観測変換を責務別ファイルへ分離した。
ControlCycle は既存の推定器と制御器を接続し、未初期化・古い推定、期限切れ計画、非常停止でゼロ指令を返す。
初期観測は車輪更新とVisionによる位置初期化の両方を必要とする。
これは判定関数であり、実機の停止ラッチ・復帰・減速・送信監視の実装ではない。

今後接続するもの:

1. RAVENのノード列受信と推定結果送信の通信定義。既存protobufやcommand IDは変更していない。
2. IMU形式の確定と融合。locadapter/imu.goは既存プロファイル変換の分離のみ。Estimator.AddImu は従来どおり未実装。
3. UART観測を推定用入力に変換する配線。既存SPI記録の変換をUARTにも接続済みとは扱わない。
4. Runからの周期起動、受信キュー、モード切替、最終STM出力、RAVENへの実送信。

ノードのStampは到達予定時刻で、推定・周期時刻と同じロボット単調時計へ変換する。
Node.VelBodyは各ノード姿勢を基準とするm/s、YawRateはrad/s。受信vx/vyがworld系ならデコーダ側で変換する。
車輪入力Omegaはハードウェアスロット順のrad/sで、既存状態送信の車輪周速度と混同しない。
通信の将来変更に関する既存コメントはcontrol内に残している。
