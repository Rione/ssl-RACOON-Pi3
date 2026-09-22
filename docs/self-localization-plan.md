# RACOON 自己位置推定 実装計画（最終版 v1.0）

> **この文書の読み方**
>
> 会話の文脈を知らない人（人間・AI を問わず）が、これ 1 本で実装に入れることを目標に書いてある。
> 調査で確認した事実には**出典のファイルパスと行番号**を付けた。
> 未確認の推測には「**未検証**」と明記した。この区別を崩さないこと。
>
> - 関連文書: [`robot-command-protocol-requirements.md`](./robot-command-protocol-requirements.md)（通信担当への要件）
> - 最終更新: 2026-09-12

---

## 0. 一枚まとめ

**やること**: Rock5A 上に Go で自己位置推定（カルマンフィルタ系）を実装する。
STM から来る 4 輪の車輪速度と IMU、および SSL-Vision から直接届く絶対座標を融合し、
カメラ単体より高精度・低遅延・欠落耐性のある推定を作る。位置制御ループもロボット上で閉じ、結果を RAVEN へ返す。

**最初にやること**: フィルタを書く前に **P1（計測基盤）** を実機に入れる。
理由は §3.3〜§3.5 の通り、**機体パラメータが 4 ソースで食い違い、符号規約も世代で反転しており、IMU に至っては存在しない**ため。
この状態でフィルタを書いても、誤差が出たときに原因を切り分けられない。

**過去の失敗**: RAVEN 側に同種の EKF（`EncoderEkfRobotFilter`）が既にあるが、
「**効果が測れなかった**」という理由で不採用になっている。§3.4 の符号反転が原因である可能性が高い（未検証）。
本計画は**「効果が測れること」を機能要件として設計に入れる**（§9）。

---

## 1. ゴールと非ゴール

### ゴール

1. ロボット上で 125 Hz の自己位置推定を動かす。出力は位置・速度・共分散・健全性。
2. SSL-Vision 生値より高精度にする（定量目標は §10）。
3. vision が欠落・遅延しても推定を継続する。
4. 位置制御ループがそのまま使える出力にする（連続性・遅延補償・共分散の信頼性）。
5. **効果を数値で示せるようにする。**

### 非ゴール（この計画の範囲外）

- **通信プロトコルの実装** — 別担当。要件は [`robot-command-protocol-requirements.md`](./robot-command-protocol-requirements.md) にまとめた。
- **STM ファームウェアの改修** — 回路担当。ただし §12 の依頼事項がある。
- **位置制御器そのもの** — 別途。本計画は推定の出力仕様までを担当する。
- **ボール・他ロボットの推定** — 自機の自己位置のみ。

---

## 2. システム構成

```
 ┌─────────────┐   マルチキャスト 224.5.23.2:10694        ┌──────────────────────┐
 │ SSL-Vision  │ ───────────────────────────────────────► │                      │
 │   (PC)      │   SSL_WrapperPacket (protobuf)           │      Rock5A          │
 └─────────────┘   t_capture / t_sent / frame_number      │   (Go, 本計画の範囲)  │
                                                          │                      │
 ┌─────────────┐   UDP 20011（下り）                       │  ┌────────────────┐  │
 │   RAVEN     │ ───────────────────────────────────────► │  │ timesync       │  │
 │   (PC/Java) │   目標位置 ★新設が必要                     │  ├────────────────┤  │
 │             │ ◄─────────────────────────────────────── │  │ localization   │  │
 └─────────────┘   マルチキャスト 224.5.69.4:16941（上り）   │  │  (EKF)         │  │
                   自己位置推定 ★新設が必要                  │  ├────────────────┤  │
                                                          │  │ 位置制御       │  │
 ┌─────────────┐   SPI /dev/spidev4.0 @1MHz Mode0         │  └────────────────┘  │
 │  STM32F446  │ ◄──────────────────────────────────────► │                      │
 │ (MainBoard) │   20 バイトフレーム, 8 ms 周期            └──────────────────────┘
 └─────────────┘
       │ UART 250kbps
       ├──► WheelUnit × 4（オムニホイール、エンコーダ）
       ├──► CAN: PowerBoard / Dribbler
       └──► UART: UI 基板
```

### リポジトリの場所（ローカル）

| 名前 | パス | 言語 | 役割 |
| --- | --- | --- | --- |
| **ssl-RACOON-Pi3** | `~/ssl_ws/ssl-RACOON-Pi3` | Go | **本計画の実装先。** Rock5A / Pi4B 上で動く |
| ssl-RAVEN | `~/ssl_ws/ssl-RAVEN` | Java (Gradle) | AI・戦略・世界モデル。ブランチ `for-toyota-game` |
| ssl-Circuit | `~/ssl_ws/ssl-RAVEN/ssl-Circuit` | C / C++ | STM ファームウェア。`2026/` が現行世代 |

> ssl-Circuit は ssl-RAVEN の**中**にある（`ssl_ws/ssl-Circuit` ではない）。探すときに注意。

---

## 3. 調査で判明した事実

**ここに書いてあることは実際にコードを読んで確認した。再調査は不要。**

### 3.1 SPI フレームの現状（Rock5A ↔ STM）

Rock5A が**マスタ**、STM が**スレーブ**。8 ms 周期（125 Hz）。20 バイト固定。

```
[0xFF][ ペイロード 18 バイト ][0xAA]
```

**上り（STM → Rock5A）** — 出典 `ssl-Circuit/2026/Firmware/MainBoard/MainBoard_V26_1_1/src/unit/robot.c:158`

| バイト | 内容 |
| --- | --- |
| 0 | ヘッダ `0xFF` |
| 1 | バッテリ電圧 × 10 |
| 2 | ドリブラ状態 |
| 3 | キャパシタ電圧 |
| 4–11 | **車輪角速度 × 4**（`int16` LE、`rad/s × 100`）順は FL, BL, BR, FR |
| **12–18** | **0 埋め（7 バイトの空き）** ← IMU 用に使える |
| 19 | フッタ `0xAA` |

**下り（Rock5A → STM）** — 出典 `robot.c:87` (`Robot_RockApplyRecvPacket`) / `ssl-RACOON-Pi3/internal/state/state.go` (`SendPayload`)

| バイト | 内容 | STM 側で使われているか |
| --- | --- | --- |
| 0–5 | VelX / VelY / VelAng（`int16` LE、mm/s・mrad/s） | ✅ 使用 |
| 6 | ドリブルパワー | ✅ |
| 7–8 | キック / チップ | ✅ |
| **9–14** | **RelativeX / RelativeY / RelativeTheta** | ❌ **受信して構造体に入るだけで、どこからも読まれていない** |
| **15–16** | **CameraBallX / CameraBallY** | ❌ **同上** |
| 17 | Informations（ビットフィールド） | ✅ |

> **8 バイトの死にフィールドが下りにある。** プロトコルを変えずに使える余地がある。
> ただし STM 側が無視しているだけなので、将来使われ始めたら衝突する。使うなら回路担当に一言要る。

**Rock5A 側の関連コード**

| ファイル | 内容 |
| --- | --- |
| `internal/rock5a/spi.go` | `RunSPI()` = 8 ms ticker のループ。`processSPICommunication()` が `conn.Tx()` を呼ぶ |
| `internal/rock5a/frame.go:44` | `validateSPIFrameAt()` — **パディング（12–18）が 0 であることを必須にしている。IMU を載せるなら要変更** |
| `internal/rock5a/config.go` | `WheelDiameterMm = 60.0`、`SPIPeriodMs = 8` など |
| `internal/state/state.go` | `RecvData` / `SendPayload` の構造体定義 |
| `internal/link/common.go` | ボード非依存の送信フレーム組み立て |

### 3.2 車輪速度の単位 — 確定

Pi が受け取る `raw / 100` は **車輪軸の角速度 [rad/s]**。減速比は STM 側で既に割られている。

根拠 — `ssl-Circuit/2026/STMDev/main_F446RE/src/unit/MotorDriver.cpp:73`:

```cpp
int16_t MotorDriver::motorToWheelScaled(int16_t motor_omega) {
      float wheel_omega = (float)motor_omega * INV_GEAR_RATIO;   // = 15/56
      return Constrain((int16_t)(wheel_omega * 100.0f), ...);
}
```

エンコーダはモータ軸にあるが、`GEAR_RATIO = 56/15 = 3.7333` を割ってから送っている。
新世代 (`omni_drive.c`) も `vel_wheel_angular * ROBOT_WHEEL_RADIUS` で周速に変換しており、同じ解釈で整合する。

> **注意**: 値を作っているのは WheelUnit の `BLDC_GetAngularSpeed()` だが、
> `2026/Firmware/WheelUnit/WheelDriver_V26_3/` は `usart.c` と `app.c` の 2 ファイルしかコミットされておらず、
> **実装で裏が取れていない**（§12-5）。

### 3.3 ⚠ 機体パラメータが 4 ソースで食い違っている

| パラメータ | 旧世代 STM<br>`STMDev/main_F446RE/src/unit/MotorDriver.hpp` | 新世代 STM<br>`Firmware/.../src/config/parammeter.h` | ssl-RAVEN<br>`app/config/system_model_sim.yaml` | ssl-RACOON-Pi3<br>`internal/rock5a/config.go` |
| --- | --- | --- | --- | --- |
| 車輪半径 | **27 mm**<br>`WHEEL_DIAMETER 54` | **30 mm**<br>`ROBOT_WHEEL_RADIUS 0.03f` | **26 mm**<br>`wheel_radius_mm: 26.0` | **30 mm**<br>`WheelDiameterMm = 60.0` |
| 回転のモーメントアーム | **85 mm**<br>`WHEEL_BASE_DIAMETER 170` | **75 mm**<br>`ROBOT_WHEEL_BASE_RADIUS 0.075f` | **90 mm**<br>`robot_radius_mm: 90.0` | — |
| 車輪取付角 | 55 / 135 / −135 / −55 | 55 / 135 / −135 / −55 | **60** / 135 / −135 / **−60** | — |
| 減速比 | **56 : 15 = 3.7333** | 記述なし（適用済みの値を受け取る） | 3.7333 | — |

**そのほか気になる点**

- 新世代の `parammeter.h` には `ROBOT_RADIUS 0.089f`（89 mm）もあるが、**運動学では使われていない**。
  使われているのは `ROBOT_WHEEL_BASE_RADIUS`（75 mm）。「車輪基底**径**」という名前なのに半径として使われている。
- 旧世代の `±55°` の車輪だけ `cos` に **`* 1.05`** という経験補正が入っている
  （`MotorDriver.cpp:36,39`）。**実際の取付角が 55° ではない可能性**を示唆する。
- 旧世代の `ROBOT_SPIN_TO_MOTOR_ROTATE_RATIO ((WHEEL_BASE_DIAMETER / WHEEL_DIAMETER) * GEAR_RATIO)` は
  **整数除算**なので `170 / 54 = 3`（真値 3.148）。回転スケールが約 4.7% 小さい。

**方針**: 数値は今後確認する。**それまでは暫定値で進めてよい**が、
**この不一致が残る限り §10 の精度目標は評価できない。**

### 3.4 ⚠ 符号規約が新世代だけ反転している

```
旧世代 (MotorDriver.cpp:36-39)     v_wheel ∝  +vx·sin(α) − vy·cos(α) − R·ω
新世代 (omni_drive.c:22)           v_w     =  −vx·sin(θ) + vy·cos(θ) + R·ω
RAVEN  (OmniWheelKinematics.java)  v_i     =   sin(α)·vx − cos(α)·vy − R·ω
```

**旧世代と RAVEN は一致し、新世代だけが完全に符号反転している。**

> **これが「効果が測れなかった」の原因候補である（未検証）。**
>
> RAVEN の運動学は旧世代に合わせて書かれている。新世代が符号を反転させたなら、
> RAVEN がエンコーダから復元する速度は真値の符号が逆になる。
> vision と融合すれば補正が毎回逆向きに効くので、「エンコーダを足しても良くならない」結果になる。
> さらに `system_model_sim.yaml` の `innovation_gate: 0.0` はゲートを無効化しているので棄却もされない。
>
> **反証の可能性**: WheelUnit 側の符号や BLDC ID の設定次第で辻褄が合っている可能性は残る。
> **P1 で「指令 vs 実測」を記録すれば 1 分で確定する。最優先で確認すること。**

加えて、**ホイール番号と取付角の対応も世代で食い違う。**

```cpp
// 旧世代 Robot.cpp:112-117 のコメント
motorToWheelScaled(motor[3]),  // FL (M3, -55°)
motorToWheelScaled(motor[0]),  // FR (M0,  55°)
```

旧世代は **FL = −55° / FR = +55°**。新世代は `ROBOT_MOTOR_DEGREE[4] = {55, 135, -135, -55}` を
index 0..3 に割り当て、index 0 を FL として SPI に載せるので **FL = +55° / FR = −55°**。
**FL と FR が入れ替わっている可能性がある。**

### 3.5 ⚠ IMU は存在しない（両世代とも）

**新世代 `Firmware/MainBoard/MainBoard_V26_1_1`**
IMU への言及は `DigitalIn sw_imu`（`IMU_RESET` GPIO）の宣言 2 行のみ。センサを読むコードも送るコードもない。

**旧世代 `STMDev/main_F446RE`**
`Lib/MPU6500/`, `Lib/BNO055/`, `Lib/Madgiwick/`, `Lib/RobotPoseEstimator/` は**存在するが、すべて無効化されている**。

```cpp
// Robot::hardwareInit()  — Robot.cpp:6
//   bno.init();              ← コメントアウト
// mpu.init();                ← コメントアウト
//   bnoCalibrate();          ← コメントアウト

// app.cpp:22  TimInterrupt4khz()
// robot.mpuget();            ← コメントアウト
```

`mainMode.cpp:12` / `cameraMode.cpp:14` の `bnoGet()` 呼び出しも両方コメントアウト。
`RobotPoseEstimator` は**どこからも参照されていない**。

**送信経路にも IMU はない。**

| 送信先 | 関数 | 中身 |
| --- | --- | --- |
| Pi（UART） | `rasSendSerial` (`Robot.cpp:101`) | 13 B: `[0xFF][batt][dribble][cap][wheel×4][0xAA]` |
| UI 基板 | `uiSendSerial` (`Robot.cpp:217`) | 21 B: batt / capa / buzzer / ボール検知 / モータ 4 |
| CAN | `MotorDriver` / `KickerBoard` | モータ・キック指令のみ |

**結論: 「STM から IMU が来る」ことにコード上の裏付けはまったくない。**
IMU を使うには、(1) 旧世代の `MPU6500.cpp` を有効化して動作確認、(2) C++ → C で新世代へ移植、
(3) SPI の 0 埋め部分に載せる、(4) Pi 側の `validateSPIFrameAt()` を緩める、の 4 段階が要る。

### 3.6 IMU の想定仕様（移植の準拠先）

実際に動いた実績は不明だが、コードから読める仕様は次の通り。**これに準拠して設計する。**

| 項目 | 内容 | 出典 |
| --- | --- | --- |
| センサ | **MPU6500**（`hspi2`、CS = `SPI2_CS0`） | `Robot.hpp:265` |
| ジャイロ | **±250 dps、131 LSB/(deg/s)** | `MPU6500.cpp:80` の `/131` |
| 加速度 | **±2 g、16384 LSB/g** | `MPU6500.cpp:77` の `/16384` |
| オフセット | STM 側で 34464 サンプル平均、Flash 保存、起動時適用。`IMU_SW` 押下起動で再較正 | `MPU6500.cpp:40`, `Robot.cpp:327` |
| BNO055 | I2C1 に実装あり。ただし `init` はコメントアウト | `Robot.hpp:270` |

### 3.7 SSL-Vision（ロボットへ直送）

| 項目 | 値 | 出典 |
| --- | --- | --- |
| マルチキャスト | `224.5.23.2:10694` | `ssl-RAVEN/app/config/network.yaml` |
| パケット | `SSL_WrapperPacket` → `SSL_DetectionFrame` | `.../proto/pb_src/ssl_vision_detection.proto` |
| 使えるフィールド | `frame_number` / **`t_capture`** / **`t_sent`** / `camera_id` / `robots_blue[]` / `robots_yellow[]` | 同上 |
| ロボット座標 | `x`, `y`（mm）, `orientation`（rad）, `confidence` | 同上 |

> **マルチキャストは ACK も再送もない。**多くの AP では最低基本レートで送出されるため、
> ユニキャストより損失率が明確に高い。**欠落率の実測は P1 の必須項目**（`frame_number` の欠番で測る）。

### 3.8 RAVEN 側の状況

| 項目 | 内容 |
| --- | --- |
| 既存 EKF | `app/src/main/java/org/rione/ssl/raven/common/filter/robot/EncoderEkfRobotFilter.java`。状態 `[x, y, θ, vx, vy, ω]`。**「効果が測れなかった」ため不採用。本計画では参照しない** |
| 時刻正規化 | `common/vision/VisionTimestampNormalizer.java`。`t_sent` ベースの EMA（α=0.05）。**本計画では採用しない**（§5.3 に理由） |
| 単位変換 | `communication/robot/RobotClient.java:311` が `× 1000` で m/s → mm/s。RAVEN 内部は **mm** |
| 指令プロトコル | `grSim_Robot_Command`（`proto/pb_src/grSim_Commands.proto`）。**速度のみ。目標位置は送れない** |
| 上り | `PiToMw`（`proto/pb_src/pi_to_mw.proto`）。マルチキャスト `224.5.69.4:16941` |
| MCAP ログ | `foxglove/McapWriter.java`（484 行、外部ライブラリ不要の自前実装）、`foxglove/RecordingClock.java`。**設計を踏襲する**（§6） |
| 遅延補償の枠 | `app/config/control.yaml` の `input_delay_compensation_enabled: false` 等。枠はあるが無効 |

**決定事項**: **RAVEN は自機について独自推定をやめ、ロボットが返す推定をそのまま採用する。**
ロボットとRAVENの両方でフィルタを掛けると、観測が白色でなくなり、
同じ vision フレームが二重計上されて共分散が過小評価され、遅れも二重に積み上がるため。

### 3.9 既存コードの注意点

- **`state.FlWheelSpeedRadS` は実際には m/s。** 名前と単位が一致していない
  （`motorRawToWheelMS()` が `rad/s × 車輪半径` を返す）。新パッケージでは単位明示型のみ使い、直接参照しない。
- **`internal/link`, `internal/pi4`, `internal/rock5a` には `//go:build pi4 || rock5a` が付いている。**
  開発 PC（macOS）ではビルドもテストもできない。新パッケージには**ビルドタグを付けない**（§7.1）。

---

## 4. 推定器の設計

### 4.1 座標系と単位

- **ワールド系**: SSL-Vision 準拠。フィールド中央原点、右手系、`θ` は x 軸から反時計回り。
- **ロボット系**: 前方 `+x`、左 `+y`、`+ω` 反時計回り。既存の `veltangent → VelX`, `velnormal → VelY` と整合。
- **内部は SI 単位（m, m/s, rad, rad/s）のみ。** mm への変換は境界（アダプタ層）でのみ行う。
  RAVEN 内部が mm なので、ここを曖昧にすると必ず事故る。

### 4.2 状態

```
公称状態（物理量として保持）
  T = (R(θ), p) ∈ SE(2)   姿勢と位置
  v ∈ R²                   ロボット系の並進速度
  ω ∈ R                    ヨーレート
  b_ω ∈ R                  ジャイロバイアス
  s ∈ R²                   車輪オドメトリのスリップ速度（外乱）

誤差状態（フィルタが扱う 10 次元）
  δx = [ δφ, δp(2), δv(2), δω, δb_ω, δs(2) ]
```

**設計上の要点**

- 姿勢誤差は **SO(2) の接空間**で扱い、公称状態への注入時に多様体上で合成する。
  角度の巻き戻し処理が不要になり、±π 付近の不連続が消える。
- 姿勢と速度は**右不変**な形で扱い、位置はユークリッドのまま持つ（部分不変 EKF / PIEKF）。
  誤差伝播が推定軌道に依存しなくなり、共分散の過信を避けられる。
- **スリップ `s` を独立した状態として持つ。** スリップは数十 ms の速い外乱、
  スケール誤差は数分〜恒久のモデル誤差で時定数が違う。混ぜると両方とも正しく推定できない。
  スケール誤差はオフライン較正（§8）で追い出し、オンラインではスリップだけを推定する。

### 4.3 予測（125 Hz、実測 dt で駆動）

```
θ ← θ + (ω_gyro − b_ω)·dt
p ← p + R(θ)·v·dt
v ← v + a_body·dt      加速度計がある場合。無ければ定速度モデル（Q で表現）
b_ω ← b_ω              ランダムウォーク（Q は Allan 分散から、§8）
s   ← α·s              一次減衰（外乱オブザーバ、時定数 τ_s 既定 100 ms）
```

- **ジャイロは「入力」、車輪速度は「観測」。** 逆にするとスリップが予測に混入して補正できなくなる。
- **`dt` は SPI 転送時刻の差分の実測値を使う。公称 8 ms を使わない**（理由は §5.2）。
- スリップ状態の一次減衰が外乱オブザーバの役割。放置すると恒久バイアスとして誤学習する。

### 4.4 観測

| 観測 | 内容 | レート | 扱い |
| --- | --- | --- | --- |
| **SSL-Vision** | `[p, θ]` の絶対値 | 60 Hz 前後・欠落あり | `timesync` で `t_capture` を写像 → OOSM retrodiction → IEKF 反復更新 → Huber → 適応 R |
| 車輪オドメトリ | 4 輪速度 | 125 Hz | **4 次元のまま**観測。観測モデルにスリップ `s` が入る |
| ジャイロ | `ω` | 125 Hz | 予測の入力。`b_ω` は vision と ZARU で可観測になる |
| 加速度計 | `[a_x, a_y]` | 125 Hz | 速度予測に使用。低速域は S/N が悪いので重みを下げる |
| **ZUPT / ZARU** | 疑似観測 `v = 0` / `ω = 0` | 停止検出時 | 下記 |

**ZUPT / ZARU** — SSL のロボットは試合中に頻繁に停止する（STOP・HALT・配置待ち）。
停止検出時に `v = 0`, `ω = 0` を非常に小さい R の疑似観測として与える。
判定は **3 条件の AND**（指令速度が閾値以下、4 輪速度がすべて閾値以下、ジャイロ・加速度の短窓分散が閾値以下）。
効果は位置クリープの停止と、**停止のたびにジャイロバイアスが較正し直されること**。
誤検出は致命的（動いているのに位置が固まる）なので条件は厳しめにし、継続時間に上限を設ける。

**Huber 型ロバスト更新** — 正規化イノベーション `‖ν‖_S` が閾値 `c`（既定 1.5〜3）を超えたら
重みを `c/‖ν‖_S` に落とす。**χ² ハードゲートは使わない**（一度弾き始めると共分散が縮んだまま復帰できない失敗モードがある）。
別途、大きな残差が一定時間続いたら**強制リセット**する経路を残す（誘拐ロボット対策）。

**適応 R** — イノベーション共分散のマッチングによる逐次推定を**上下限クランプ付き**で入れる。
マルチキャスト欠落や遮蔽で実効精度が落ちる区間を自動で扱える。クランプは発散防止のため必須。

### 4.5 4 輪オムニの逆運動学

配置角 `α_i`、ロボット半径 `R`、車輪半径 `r`（**いずれも §3.3 の通り未確定。設定ファイルから読む**）。

```
v_i = ±( sin(α_i)·vx − cos(α_i)·vy − R·ω )     ← 符号は §3.4 の確定待ち。設定で反転できるようにする
```

- `M`（4×3）の擬似逆は**起動時に一度だけ**計算する。
- 観測モデルは **4 次元のまま**扱う（擬似逆で 3 次元に潰さない）。
  潰すと 4 輪それぞれの雑音が混ざり、個別の車輪異常が見えなくなる。
- **冗長性（4 式 3 未知数）の残差は情報。** スリップ状態 `s` の推定に効くほか、
  特定の 1 輪だけ残差が続けばモータ・エンコーダの故障検出になる。

### 4.6 遅延観測の扱い（OOSM retrodiction）

1. 状態・共分散・入力を**リングバッファに 200 ms 分**保持（8 ms × 25 エントリ）。
2. vision 観測を `timesync` で Rock5A の時間軸 `t_v` に写像する。
3. 現在状態を `t_v` へ retrodict し、**IEKF 更新（2〜3 回反復）**して、その補正を現在時刻へ伝播する。
4. `t_v` がバッファより古ければ**その観測を捨てる**（無理に取り込むと共分散が壊れる）。破棄率をメトリクスに出す。

> **全区間の再伝播はしない。** 文献上、retrodiction の共分散悪化は全区間再処理比で 2〜3% にとどまり、
> 履歴を 3〜4 ステップより長く持っても利得は急減する。費用対効果が悪い。

---

## 5. 時刻同期

**この系の精度を決めるのは、フィルタの洗練ではなく「どの値がいつの瞬間のものか」を正しく扱えるかである。**
3 m/s のロボットにとって 10 ms のズレは 3 cm の位置誤差になる。

### 5.1 クロックドメイン

| ドメイン | 時計 | 観測できるもの | 同期手段 |
| --- | --- | --- | --- |
| SSL-Vision PC | PC のシステムクロック | `t_capture` / `t_sent`（秒, double） | **下側凸包でオフセット＋スキューを推定**（§5.3） |
| **Rock5A** | `CLOCK_MONOTONIC` | UDP 受信時刻、SPI 転送時刻 | **基準時刻** |
| STM32 | MCU のタイマ | **なし**（処理追加不可） | 固定オフセット＋ジッタを Q に載せる（§5.2） |
| RAVEN | PC のシステムクロック | 目標位置パケット | 制御目標なので厳密な同期は不要 |

水晶の確度は ±20〜100 ppm。50 ppm なら 1 分で 3 ms、10 分の試合で 30 ms ずれる。
**オフセットだけでなくスキュー（周波数差）まで扱う必要がある。**

`t_sent − t_capture` は**同一クロック内の差**なので、クロック同期なしで正確に測れる vision 処理遅延である。

### 5.2 Rock5A の時刻基準（Go 固有の罠）

**`CLOCK_MONOTONIC` を唯一の基準とする。** すべてのサンプルをこの時間軸へ写してからフィルタに入れる。

```go
// Stamp はプロセス起動時刻を原点とする単調時刻。壁時計とは無関係。
type Stamp time.Duration
```

- `time.Now()` の `time.Time` は単調時計の読み値を内部に持ち、`Sub`/`Since` はそれを使う。
  **しかし JSON や gob にマーシャルすると落ちる。`Round(0)` も落とす。**
  → ログ・IPC で `time.Time` をそのまま渡さず、**`Stamp` だけを API 境界で使う**。
- 壁時計（`CLOCK_REALTIME`）は**ログの人間可読なメタデータ以外に一切使わない**。
- **NTP デーモンは止めない。** `CLOCK_MONOTONIC` は NTP の step で飛ばないし、
  スルーイング（最大 ±500 ppm）は §5.3 のスキュー推定が 30 秒ウィンドウで追従する。
  むしろ RAVEN のログと壁時計で突き合わせられる利点の方が大きい。

**SPI の時刻付け**

- 現行は `time.NewTicker(8ms)`（`internal/rock5a/spi.go`）。
  **`time.Ticker` は受信が遅れると間隔を調整したりティックを落としたりする。公称 8 ms を信用してはいけない。**
- `conn.Tx()` の**直前と直後で `time.Now()` を取り、中点を転送時刻**とする。
- `dt` のジッタ分布を常時ヒストグラムで監視する（GC ポーズ、スケジューリング遅延の可視化）。

**STM 側の時刻（処理追加が不可なので推定するしかない）**

Rock5A がマスタなので**転送時刻は完全に分かる**。未知なのは「STM がいつセンサを読んだか」だけ。

- **バイアス**: STM のサンプリング周期 `T_stm` の半分＋ IMU の LPF 群遅延＋エンコーダ窓平均の半分。
  → **センサごとの固定 `timeOffsetMs`** としてフレームプロファイルに持たせる。
- **ジッタ**: 幅 `T_stm` の一様分布 → **プロセスノイズ Q に載せる**。
  速度観測に `σ² += (a_max · T_stm/√12)²` 相当を上乗せする。
- **計測**: 既知の加減速を与え、指令と車輪速度応答の相互相関から遅延を推定する。

### 5.3 SSL-Vision ↔ Rock5A

```
t_arrive_i = (1 + α)·t_capture_i + θ + δ_i        δ_i ≥ 0

  α … スキュー    θ … オフセット    δ_i … 片方向遅延（vision 処理 + ネットワーク）
```

`δ_i` は必ず非負で、無線の再送によりバースト的に伸びる。
**分布は下側に硬い壁があり上側に長い裾を持つので、平均ではなく最小値が真値に近い。**

→ **下側凸包（lower convex hull）による推定**を採る。
点群 `(t_capture, t_arrive − t_capture)` の下側凸包の
**傾きがスキュー、切片がオフセット＋最小片方向遅延**になる。
線形回帰や EMA と違い、遅い配送の影響を原理的に受けない。O(n log n) でスライディングウィンドウに載る。

- ウィンドウは直近 30 秒程度。**`camera_id` ごとに独立した状態**を持つ。
- **`t_sent` ではなく `t_capture` を使う**（`t_sent` だと vision の処理負荷変動が混入する）。
  → RAVEN の `VisionTimestampNormalizer` は `t_sent` ＋ EMA なので、**そのまま真似しない**。
- `frame_number` の欠番で**マルチキャストの欠落率を計測**する。
- **原理的な限界**: 片方向観測だけでは「クロックオフセット」と「最小片方向遅延」は分離できない。
  分離できるのは**スキューと遅延の変動分**。残る定数分はオフラインで実測して定数として持つ。

```go
type Sync interface {
    Observe(remote, arrival Stamp)
    ToLocal(remote, arrival Stamp) (Stamp, Quality)
}
```

**クロック推定は姿勢推定フィルタとは別のフィルタにする。**
クロックのドリフトは分オーダー、姿勢は ms オーダーで時定数が 4〜5 桁違う。
同じ状態ベクトルに混ぜると数値的に条件が悪くなり、姿勢側の共分散が意味を失う。

> `t_capture` が信用できないと判明した場合に備え、遅延 `τ` を状態としてオンライン推定する実装
> （観測ヤコビアン `∂h/∂τ = −v`）を設定で有効化できるよう残す。
> ただし `v = 0` で不可観測、`v` 一定で位置バイアスと縮退するため、**加減速・旋回中のみ更新**する。

### 5.4 レイテンシ予算

**数値は未測定。推測値を設計に埋め込まないこと。**

| 区間 | 記号 | 測り方 | 用途 |
| --- | --- | --- | --- |
| 露光中点 → vision 送信 | `τ_det` | **`t_sent − t_capture` として直接測れる** | vision 観測の時刻補正 |
| vision 送信 → Rock5A 受信 | `τ_net` | 凸包法の切片。無線なので分布の裾も見る | 同上 |
| 受信 → フィルタ反映 | `τ_proc` | Go 内で計測（1 ms 未満のはず） | 同上 |
| STM サンプリング → SPI 転送 | `τ_stm` | 指令と車輪速度応答の相互相関 | センサの `timeOffsetMs` |
| 推定 → SPI 送出 | `τ_tx` | 0〜8 ms（次のティック待ち）、平均 4 ms | `PredictAhead` |
| SPI 受領 → モータ応答 | `τ_act` | ステップ応答を実測 | `PredictAhead` |

上 3 つの和が vision 観測の遅延、下 2 つの和が制御の遅延。**用途が違うので別々に管理する。**

### 5.5 監視すべき失敗モード

時刻同期は静かに壊れる。**壊れたことを検出できる仕組みを最初から入れる。**

| 失敗モード | 検出方法 | 対応 |
| --- | --- | --- |
| スキュー推定の暴走 | 推定値が ±200 ppm を超える | 推定を凍結し、直前の値を保持して警報 |
| AP ローミング・経路変更 | 最小片方向遅延が急にジャンプ | 凸包ウィンドウをリセット |
| **マルチキャスト欠落** | **`frame_number` の欠番率** | 欠落が続けば `Health = DEGRADED` |
| 順序逆転・重複 | `frame_number` の非単調性 | 重複は破棄、逆転は OOSM バッファで処理 |
| バッファより古い観測 | retrodiction 可能範囲外 | 破棄し、破棄率をメトリクスに出す |
| `t_capture` が不正（0 や負） | 値の検証 | 到着時刻フォールバック。発生率をログ |

---

## 6. ログ設計（MCAP）

RAVEN の `foxglove/McapWriter.java` と `RecordingClock.java` の設計を踏襲する。
**Foxglove で RAVEN 側のログと並べて見られる**ことに大きな価値がある。

### 6.1 踏襲する設計

| 設計 | 内容 | 本件での意味 |
| --- | --- | --- |
| **epoch アンカー + 単調差分** | 起動時に「壁時計 epoch」と「単調時計」を 1 度だけ組にして留め、以降は単調差分だけ足す。アンカーを `clock_epoch_ns_at_start` / `clock_monotonic_ns_at_start` として MCAP metadata に書く | §5.2 の `Stamp` 設計と完全に一致。単調性を保ちつつ読む側は壁時計へ引き直せる |
| **生入力をそのまま記録** | `/in/vision` は protobuf を FileDescriptorSet ごと埋める。`/in/feedback` は datagram を base64 のまま | **STM のフレーム仕様が未確定でも、生の 20 バイトを記録しておけば後で再デコードできる。本件では決定的** |
| **チャンク単位で flush** | Chunk を閉じるたびに MessageIndex を書いて flush | 実機は電源を落として止めるので必須 |
| **チャンネルごとの連番** | MCAP の `sequence` を発行者ごとに採番 | 欠落検出 |
| **単位をフィールド名に埋める** | `ekfVx_mm_s`, `batteryVoltage_V`, `length_bytes` | §3.9 の `FlWheelSpeedRadS` 問題の再発防止。**全面採用する** |
| **記録自体の健康状態を記録** | `/recorder/status` | 落ちたことがログの外にしかないと、後日ログだけ見る人に穴が見えない |

### 6.2 チャンネル設計

| トピック | encoding | 内容 |
| --- | --- | --- |
| `/in/spi` | json | **SPI の生フレーム 20 バイト**（tx / rx）。`tx_ns` / `rx_ns`、`frame_base64`。**仕様未確定でも今から記録できる** |
| `/in/vision` | protobuf | `SSL_WrapperPacket` を FileDescriptorSet ごと |
| `/in/vision_meta` | json | `camera_id`, `frame_number`, `t_capture_s`, `t_sent_s`, `recv_ns`, `mapped_ns`, `skew_ppm`, `offset_ns` |
| `/sensors/wheel` | json | `wheelFL_rad_s` ほか 4 輪、`sample_ns`（`timeOffsetMs` 適用後） |
| `/sensors/imu` | json | `gyroZ_rad_s`, `accelX_m_s2`, `accelY_m_s2`, `sample_ns` |
| `/est/state` | json | `x_mm`, `y_mm`, `theta_rad`, `vx_mm_s`, `vy_mm_s`, `omega_rad_s`, `gyroBias_rad_s`, `slipX_mm_s`, `slipY_mm_s`, `cov_*`, `health` |
| `/est/innovation` | json | vision / オドメトリの残差、正規化イノベーション、Huber 重み、棄却フラグ |
| `/est/timing` | json | `dt_ns` 実測、`retrodict_ns`、破棄観測数、ループ処理時間 |
| `/control/target` | json | RAVEN から来た目標位置 |
| `/out/command` | json | STM へ送った指令 |
| `/recorder/status` | json | 記録自体の健康状態、ドロップ数 |

### 6.3 実装

- **`github.com/foxglove/mcap/go/mcap`（公式 Go 実装）を使う。** RAVEN は Java で自前実装しているが、
  Go には公式ライブラリがあるので車輪の再発明はしない。
- 記録は**別 goroutine**。推定ループからはバッファ付きチャネルでノンブロッキングに渡す。
  **記録が詰まっても推定と制御は絶対に止めない。** 落とした件数は `/recorder/status` に出す。
- 同じ MCAP を `cmd/loc_replay` が読み、**ビット単位で同じ推定結果を再現できる**ようにする。

---

## 7. Go 実装設計

### 7.1 パッケージ構成

```
internal/localization/        # 推定コア。通信もハードも知らない。ビルドタグなし
  types.go                    #   Stamp, Pose2, ImuSample, WheelSample, VisionPose, Estimate
  estimator.go                #   公開API: Predict / AddWheel / AddImu / AddVision / At / PredictAhead
  eskf.go                     #   誤差状態フィルタ（predict / update / inject / reset）
  manifold.go                 #   SE(2), SO(2) の exp / log / adjoint
  kinematics.go               #   4輪オムニ ⇔ body velocity、冗長残差
  slip.go                     #   外乱オブザーバ、ZUPT / ZARU 判定
  oosm.go                     #   リングバッファと retrodiction
  robust.go                   #   Huber 重み、適応 R
  health.go                   #   発散検知・リセット判定
  config.go                   #   幾何・共分散パラメータ（JSON）
  *_test.go

internal/timesync/            # クロック推定。通信を知らない。ビルドタグなし
  sync.go convexhull.go monitor.go *_test.go

internal/stmframe/            # STM フレームのファイル駆動デコーダ。ビルドタグなし
  profile.go decode.go profiles/*.json

internal/loclog/              # MCAP の読み書き。ビルドタグなし
  channels.go writer.go reader.go clock.go

internal/locadapter/          # ハードウェア・通信に触る層。ここだけビルドタグあり
  vision_multicast.go sensor_spi.go sink_raven.go replay.go

cmd/loc_replay/               # ログ再生・パラメータ掃引ツール
```

**ビルドタグの方針が重要。**
既存の `internal/link`・`internal/pi4`・`internal/rock5a` は `//go:build pi4 || rock5a` が付いており、
**開発 PC（macOS）ではビルドもテストもできない**。
`localization` / `timesync` / `stmframe` / `loclog` には**ビルドタグを付けない**。
これで `go test ./internal/localization/...` が開発 PC でそのまま回り、実機なしで開発できる。
**ハードウェアに触るのは `locadapter` だけに閉じ込める。**

### 7.2 goroutine 構成

- **リンク goroutine（既存の `RunSPI`）が推定の predict を回す。**
  SPI トランザクション直後にサンプルが揃うので余分な同期が要らない。
- **vision 受信は別 goroutine。** 受信したら `timesync.Observe` を呼び、
  **バッファ長 1 のチャネルへノンブロッキング送信**する（`select` の `default` で捨てる）。
  フィルタが詰まっても受信側が止まらない。捨てた件数はメトリクスに出す。
- **記録は別 goroutine。** ノンブロッキング。詰まったら捨てて件数を数える。
- **推定器は単一 goroutine からのみ触る。** ロック不要になる。
- 結果の公開は `atomic.Pointer[Estimate]`。制御側・HTTP 側はロックなしで読める。
- **推定が panic しても走行機能を巻き込まない**よう `recover` を入れる。
  リンクループが推定結果を待つ構造にはしない。

### 7.3 GC・アロケーション・行列

- **ホットパス（predict / update）でアロケートしない。**
  リングバッファ、行列ワークスペース、SPI バッファは起動時に確保して再利用する。
- `go test -bench . -benchmem` で**1 周期あたり 0 アロケーションをテストで固定**する。
  GC の発生頻度そのものが下がり、§5.2 のジッタが減る。
- リンク goroutine は `runtime.LockOSThread()` で OS スレッドに固定する。
- 行列は **`gonum.org/v1/gonum/mat` をレシーバ事前確保で使う。** 純 Go で実行時依存が増えない。
  0 アロケーションが破れた箇所だけ手書きに落とす。
- **記録経路は別 goroutine なのでアロケートしてよい**（流量は監視する）。

### 7.4 決定論

`localization` と `timesync` は **`time.Now()` を呼ばない。** すべて `Stamp` を引数で受ける。
乱数を使わない（使う場合はシードを外から注入）。
**同じ MCAP を入れれば必ず同じ出力になる。** これがオフラインチューニングと回帰テストの前提。

### 7.5 STM フレームのプロファイル

バイト配置が未定なので、**フレーム定義をファイル駆動**にする。

```jsonc
{
  "profile": "rock5a-v2-imu",
  "frameSize": 20, "header": "0xFF", "footer": "0xAA",
  "fields": [
    { "name": "wheelFL", "offset": 4,  "type": "i16le", "scale": 0.01, "unit": "rad/s", "timeOffsetMs": -2.0 },
    // ... BL, BR, FR

    // MPU6500 の生 LSB を送る前提（§3.6）。±250 dps = 131 LSB/dps, ±2 g = 16384 LSB/g
    { "name": "gyroZ",   "offset": 12, "type": "i16le", "scale": 0.000133231, "unit": "rad/s", "timeOffsetMs": -3.5 },
    { "name": "accelX",  "offset": 14, "type": "i16le", "scale": 0.000598550, "unit": "m/s^2", "timeOffsetMs": -3.5 },
    { "name": "accelY",  "offset": 16, "type": "i16le", "scale": 0.000598550, "unit": "m/s^2", "timeOffsetMs": -3.5 }
  ],
  "reserved": [{ "offset": 18, "length": 1, "mustBeZero": true }]
}
```

スケールの根拠: `1/131 [deg/s/LSB] × π/180 = 1.33231e-4`、`1/16384 [g/LSB] × 9.80665 = 5.98550e-4`。

サポートする型: `u8 / i8 / u16le / u16be / i16le / i16be / i32le / i32be / f32le / f32be`。

**設計原則: 定義されていないフィールドはセンサ非搭載として扱い、コアはその観測を単に使わない。**
これにより **IMU の仕様が来る前に、車輪＋vision だけでフィルタを完成させて実機投入できる。**

**既存の `validateSPIFrameAt()`（`internal/rock5a/frame.go:44`）はパディング＝0 を必須にしているので、
IMU を載せる際はプロファイル駆動の検証に置き換える必要がある。**

### 7.6 出力（位置制御向け）

```go
type Estimate struct {
    Stamp       Stamp         // この推定が指す時刻
    Pose        Pose2         // ワールド系 [m, m, rad]
    VelBody     Vec2          // ロボット系 [m/s]
    YawRate     float64       // [rad/s]
    CovPose     Mat3          // 位置・姿勢の共分散
    Slip        Vec2          // 推定スリップ速度（診断にも制御にも使える）
    SinceVision time.Duration // 最後に有効な vision 観測を取り込んでからの経過
    Health      Health        // OK / DEGRADED / INVALID
}
```

| 要件 | 理由 | 対応 |
| --- | --- | --- |
| **出力が飛ばない** | 位置制御の P 項・D 項に段差が入ると指令が跳ねる | vision 更新の補正を 1 ステップで全量当てず、**大きな補正は数ステップかけて注入**する。注入速度の上限は共分散に応じて可変 |
| **アクチュエータが効く時刻の状態を出す** | 推定 → 指令 → SPI → STM → モータ応答の遅れ。現在時刻の状態で制御すると位相が遅れて振動する | `PredictAhead(τ_tx + τ_act)`。ホライズンに上限を課す |
| **速度推定の品質** | D 項・フィードフォワードに直接効く | vision 座標の数値微分で作らない。フィルタの状態としての `v` を返す |
| **共分散が信頼できる** | 推定が怪しい時にゲインを落とす判断に使う | 不変フィルタ系を採る理由がここ |
| **異常時に制御を止めない** | 推定が壊れても走行機能は落としてはならない | `Health = INVALID` で制御側がフォールバックできるようにする |

**この構造体がそのまま RAVEN への返信内容になる**（§3.8 の決定事項）。

---

## 8. パラメータ同定

**チューニングを試行錯誤にしない。各パラメータに、それを決める手続きを紐づける。**

| 対象 | 方法 | 優先度 |
| --- | --- | --- |
| **符号とホイール順序**（§3.4） | 1 輪ずつ既知方向に動かし、指令と実測の符号を照合する | **最優先。P1 で実施** |
| **速度スケール**（§3.3） | 既知の直線距離を一定速度で走らせ、オドメトリ積分距離と実距離の比を取る。**この比がそのまま補正係数になる** | **最優先。P1 で実施** |
| **IMU の Q** | **Allan 分散。** 静止で数時間ログを取り、N（ARW）/ K（RRW）/ B（バイアス不安定性）を抽出して連続時間 Q を構成する | IMU 実装後 |
| **運動学パラメータ**（`α_i`, `R`, `r`） | 実測値を初期値とし、既知経路の走行ログに対して**オフライン最適化**（PSO 等）。先行研究では SSL のオドメトリ精度が 76% 改善している | 高 |
| **センサの `timeOffsetMs`** | IMU はデータシートの群遅延から初期値 → 実機で微調整。エンコーダは窓長の半分 | 中 |
| **vision の R** | 静止ロボットの vision 座標の分散を実測 → 初期値。以降オンライン適応（クランプ付き）。フィールド中央と端で分けて測る | 中 |
| **真値がない場合の評価基準** | **RTS スムーザ**をログにオフライン適用し、平滑化結果を疑似真値として使う。オフラインは未来の観測も使えるため、原理的にオンライン推定より必ず良い | 高 |

---

## 9. 「効果が測れること」を機能要件にする

RAVEN の EKF が不採用になった理由は「**効果が測れなかった**」である。
§3.3 / §3.4 を踏まえると、原因は次のどちらか（または両方）の可能性が高い。

1. **符号が逆だった** — エンコーダ由来の速度補正が毎回逆向きに効いていた。
2. **幾何パラメータが 10〜20% ずれていた** — 符号が正しくてもスケールが合っていなければ改善は出ない。

**どちらもフィルタの数式の問題ではない。共通しているのは、それを検出できる仕組みが無かったことである。**

| 対策 | 内容 |
| --- | --- |
| **符号とスケールを最初に確定する** | P1 で「指令 vs 実測」を記録し、4 輪それぞれの符号と比を出す |
| **疑似真値を作る** | RTS スムーザの平滑化結果を真値として使う（§8） |
| **比較対象を固定する** | 「vision 生値の RMSE」対「融合値の RMSE」を**単体テストの合格条件**にする。改善していなければ CI が落ちる |
| **共分散の正しさも測る** | **NEES / NIS** をモンテカルロで検定する。位置制御が共分散を見て判断する以上、ここが合っていないと使えない |
| **パラメータを先に潰す** | フィルタのチューニングより先に幾何パラメータを確定・較正する |
| **全部記録する** | 入力から内部状態・イノベーション・時刻まで MCAP に残し、**後から「なぜ効かなかったか」を追える**ようにする |
| **リプレイで再現する** | 同じ MCAP を流せばビット単位で同じ出力になるようにし、パラメータを振って比較できるようにする |

### 検証項目

1. **単体テスト（合成データ）** — 円軌道・8 の字・急加減速・衝突の真値を生成し、
   雑音・遅延・欠落・スリップを注入。**vision 生値より改善していることをテストで固定**する。
2. **時刻同期の単体テスト** — 既知のスキュー（例 +50 ppm）とバースト遅延を合成し、
   凸包法が ppm オーダーで推定できることを検証。AP ローミング相当のステップも入れる。
3. **一貫性テスト（NEES / NIS）** — 共分散が誤差を正しく表現しているかをモンテカルロで検定。
4. **アロケーションのベンチマーク** — 1 周期 0 アロケーションをテストで固定。
5. **リプレイの決定性テスト** — 同じ MCAP を 2 回流してビット単位で同じ出力になること。
6. **オフライン再生** — 実機ログを `cmd/loc_replay` で再生してパラメータを掃引。
7. **実機評価** — vision 生値と融合値を重ねてライブ表示。
   直線 / 円 / 静止（ZUPT）/ 急停止 / 誘拐 / ロボット同士の衝突 の 6 シナリオ。
8. **フェイルセーフ** — vision 喪失 5 秒で共分散が発散しないか、復帰時に飛びが出ないか、
   NaN 発生時に自動リセットするか、**推定が壊れても走行機能が落ちないか**。

---

## 10. 精度目標（合意済み）

**P1 の実測で前提が変わった場合のみ見直す。すべて測定可能な形にしてある。**

| # | 項目 | 目標 | 測り方 |
| --- | --- | --- | --- |
| 1 | **定常時の位置精度** | 位置 RMSE ≤ 10 mm、角度 RMSE ≤ 1.0° | 2 m/s 直線走行・vision 正常。RTS スムーザを疑似真値に |
| 2 | **vision 生値に対する改善** | 位置 RMSE を **50% 以上削減** | 同一ログでの vision 生値 vs 融合値 |
| 3 | 遅延補償の効き | 実効遅延 ≤ 5 ms 相当（2 m/s で系統誤差 10 mm 以下） | 一定速度走行時の進行方向バイアス |
| 4 | **vision 欠落 0.5 s 後** | 位置誤差 ≤ 30 mm | vision を人為的に落として計測 |
| 5 | **vision 欠落 2.0 s 後** | 位置誤差 ≤ 150 mm | 同上 |
| 6 | 停止時のドリフト | 10 秒静止で位置 ≤ 5 mm、角度 ≤ 0.5° | ZUPT の効果検証 |
| 7 | 出力の連続性 | 1 ステップの位置補正 ≤ 20 mm | 位置制御が跳ねないことの担保 |
| 8 | **一貫性** | NEES が 95% 区間に入る割合 ≥ 90% | モンテカルロ |
| 9 | 計算コスト | 1 周期 ≤ 1 ms、アロケーション 0 | `go test -bench -benchmem` |
| 10 | 時刻同期 | スキュー残差 ≤ 10 ppm、オフセット変動 ≤ 2 ms | 合成データと実機ログ |

**1・4・5 が本命。** 2 は「作った意味があったか」、8 は「共分散を制御が信用してよいか」の指標。

> **注意: §3.3 の機体パラメータが確定するまで、1〜6 は評価できない。**
> 誤差が出たときに「フィルタのせいか、パラメータのせいか」を切り分けられないため。

---

## 11. 実装フェーズ

| Phase | 内容 | 前提 | 成果物 |
| --- | --- | --- | --- |
| **P0** | `localization` / `timesync` / `stmframe` / `loclog` の骨格、`Stamp` 型、MCAP 書き出し、ビルドタグなしのテスト基盤 | なし | 実機なしで開発できる土台 |
| **P1** | **計測基盤。** SPI 生フレーム・vision パケット・全時刻を MCAP に記録。**(a) 指令 vs 実測による符号・ホイール順序・速度スケールの確定、(b) `frame_number` によるマルチキャスト欠落率の計測、(c) ジッタとレイテンシ内訳** | なし | **符号・スケール・時刻・欠落の事実。以降の判断の根拠** |
| **P2** | `internal/timesync`。凸包法によるスキュー・オフセット推定、異常検出、メトリクス | P1 のログ | vision 時刻を正しく写像できる |
| **P3** | 4 輪オムニ運動学、誤差状態フィルタの predict、車輪オドメトリ観測 | **P1 の符号確定。寸法は暫定値で可** | オドメトリ単体で動く |
| **P4** | vision 観測の取り込み、OOSM retrodiction、IEKF、Huber、適応 R | P2 | **vision 単体より高精度**（IMU なしでも成立） |
| **P5** | ジャイロを予測入力に、`b_ω` 推定、ZUPT / ZARU、スリップ外乱、加速度計 | **STM 側の IMU 実装（§3.5、現状ゼロ）** | IMU の効果が入る |
| **P6** | 位置制御向け出力（`PredictAhead`、スルーレート制限、`Health`）、RAVEN への返信、制御との結合 | P5、指令プロトコル | 位置制御ループに載る |
| **P7** | Allan 分散による Q 同定、運動学較正、RTS スムーザによる評価、実機チューニング | P5 | §10 の目標を数値で達成 |

**P1 を最優先で実機に入れること。**
§6.1 の通り**生フレームをそのまま MCAP に記録しておけば、STM のバイト配置が確定した後で過去ログを再デコードできる**ので、
**仕様確定を待たずにログ収集を始められる。**
**P0〜P4 は IMU なしで完了でき、車輪＋vision だけでも vision 単体より高精度な推定は成立する。**

---

## 12. 実装上の不明点

### A. 数値（フィルタの精度に直結。確定するまで §10 は評価できない）

| # | 項目 | 候補 | 影響 | 確定方法 |
| --- | --- | --- | --- | --- |
| A-1 | **車輪半径** | 26 / 27 / 30 mm | 速度スケールが最大 15% ずれる | 実測、または既知距離の走行で比を取る |
| A-2 | **回転のモーメントアーム** | 75 / 85 / 89 / 90 mm | 角速度スケールが最大 20% ずれる | 実測、または定速旋回で比を取る |
| A-3 | **車輪取付角** | 55° / 60° | 並進の方向が最大 5° ずれる。旧世代の `* 1.05` 補正も気になる | 実測 |
| A-4 | **ホイール番号と取付角の対応** | 世代で FL / FR が入れ替わっている | 左右が入れ替わる | **P1 で 1 輪ずつ回して確認** |
| A-5 | **符号規約** | 新世代だけ反転 | **符号が逆だと融合が逆効果になる。「効果が測れなかった」の原因候補** | **P1 で指令 vs 実測を照合。最優先** |

### B. STM 側（回路担当への依頼事項）

| # | 項目 | 現状 | 必要なこと |
| --- | --- | --- | --- |
| B-1 | **IMU の実装** | **両世代とも存在しない**（旧世代はドライバがあるが全部コメントアウト） | ① 旧世代 `MPU6500.cpp` を有効化して読めるか確認 → ② C++ から C へ移植 → ③ SPI の 0 埋め部（バイト 12–18）に載せる |
| B-2 | IMU の出力形式 | 未定 | **生のジャイロ・加速度**（オフセット除去済み）が欲しい。積分済みの姿勢角だけだとバイアスが観測できず、ジャイロを予測入力にできない |
| B-3 | IMU のバイト配置とスケール | 未定 | `gyroZ` / `accelX` / `accelY` の 3 つ（`int16` × 3 = 6 B）。想定は §7.5。**決まればプロファイルの書き換えだけで追従できる** |
| B-4 | IMU のデジタルフィルタ設定 | 未定 | カットオフ周波数。**群遅延の見積もり（`timeOffsetMs`）に要る** |
| B-5 | `BLDC_GetAngularSpeed()` の実装 | **リポジトリに無い**（`WheelDriver_V26_3` は `usart.c` と `app.c` の 2 ファイルのみ） | 全ファイルをコミットしてほしい。減速比が既に掛かっているかの裏取りに要る |
| B-6 | エンコーダ速度の性質 | 未定 | 瞬時値か窓平均か。窓平均なら窓長（`timeOffsetMs` に要る） |
| B-7 | 下りの死にフィールド | バイト 9–16 が未使用 | 推定結果を STM へ返すのに使ってよいか（将来の衝突を避けるため要確認） |

### C. 通信（別担当。要件は [`robot-command-protocol-requirements.md`](./robot-command-protocol-requirements.md)）

| # | 項目 | 必要なこと |
| --- | --- | --- |
| C-1 | 目標位置の下り | `grSim_Robot_Command` は速度のみ。**位置を送る経路が無い** |
| C-2 | 推定結果の上り | `PiToMw` は状態通知用で 60 Hz 前提ではない。拡張するか別経路にするか |
| C-3 | **タイムスタンプ** | **載せられるか。載せられない場合 §10-1 の目標を下げる必要がある** |
| C-4 | 共分散と `health` | 載せられるか。RAVEN のフォールバック設計に直結する |

### D. 計測待ち（P1 で埋まる）

| # | 項目 |
| --- | --- |
| D-1 | マルチキャストの欠落率（`frame_number` の欠番） |
| D-2 | `t_sent − t_capture`（vision の処理遅延） |
| D-3 | vision の片方向遅延とそのジッタ分布 |
| D-4 | SPI 周期の実測ジッタ（GC・スケジューリングの影響） |
| D-5 | STM のサンプリング位相と遅延（`τ_stm`） |
| D-6 | モータのステップ応答（`τ_act`） |
| D-7 | vision 座標の静止時分散（`R` の初期値） |

---

## 13. 設計判断の根拠（なぜそうしたか）

他の実装者が「なぜこうなっているのか」を再検討するときのために、主要な判断とその根拠を残す。

| 判断 | 根拠 |
| --- | --- |
| **EKF 系を使う（UKF / 粒子フィルタを使わない）** | 平面 3 自由度で非線形性は回転行列のみ。組込みの計算コストに見合わない。SSL の他チーム（TIGERs, RoboTeam Twente）も KF 系 |
| **誤差状態 + 部分不変（PIEKF）** | 誤差伝播が推定軌道に依存せず一貫性が高い。姿勢誤差を接空間で扱えば角度の巻き戻しが不要。位置まで群に載せる完全 InEKF は車輪オドメトリと相性が悪い |
| **因子グラフを使わない** | スライディングウィンドウ FGO はマルコフ性＋単一状態ウィンドウの下で反復 EKF と数学的に等価に退化する。優位性の実体は再線形化なので、**IEKF で安く取る** |
| **ジャイロは入力、車輪速度は観測** | 逆にするとスリップが予測に混入して補正できない |
| **スリップを状態として推定** | 「残差が大きい周期は R を膨らませる」は情報を捨てるだけ。状態にすればスリップ中も推定を継続できる |
| **retrodiction（完全再伝播をしない）** | 共分散悪化は全区間再処理比で 2〜3%。履歴 3〜4 ステップ以上は利得が急減する |
| **Huber（χ² ハードゲートを使わない）** | ハードゲートは一度弾き始めると共分散が縮んだまま復帰できない失敗モードがある |
| **凸包法（EMA を使わない）** | 片方向遅延は非負でバースト的に伸びるため分布が下側に硬い壁を持つ。EMA は平均に引かれて輻輳時にずれる |
| **クロック推定を姿勢推定と分離** | 時定数が 4〜5 桁違う。混ぜると数値的に条件が悪くなり姿勢側の共分散が意味を失う |
| **NTP を止めない** | `CLOCK_MONOTONIC` は step で飛ばず、スルーイングはスキュー推定が追従する。RAVEN ログとの突き合わせの利点が上回る |
| **RAVEN 側の推定を廃止** | 二重フィルタは観測の白色性を壊し、同じ vision を二重計上して共分散を過小評価し、遅れも二重に積む |
| **gonum を使う** | 状態 10 次元＋ Cholesky ＋反復更新は手書きだと誤りが入りやすい。純 Go なので実行時依存も増えない |
| **コアにビルドタグを付けない** | 既存パッケージは開発 PC でテストできない。推定コアだけでも macOS で `go test` が回るようにする |

---

## 参考文献

**リー群・不変フィルタ**
- R. Hartley et al., *Contact-aided Invariant Extended Kalman Filtering for Robot State Estimation*, IJRR 2020 — https://journals.sagepub.com/doi/10.1177/0278364919894385
- *PIEKF-VIWO: Visual-Inertial-Wheel Odometry using Partial Invariant Extended Kalman Filter* — https://arxiv.org/abs/2303.07668
- *Degeneration of Sliding-Window Factor Graph Optimization into Iterated Extended Kalman Filtering* — https://arxiv.org/abs/2511.00306

**スリップ・モデル誤差・車輪慣性航法**
- *Fully Proprioceptive Slip-Velocity-Aware State Estimation via Invariant Kalman Filtering and Disturbance Observer*, IROS 2023 — https://arxiv.org/abs/2209.15140
- L. Mozzarelli et al., *Mobile Robot Localization: a Modular, Odometry-Improving Approach* — https://arxiv.org/abs/2403.13452
- X. Niu et al., *Wheel-INS* — https://arxiv.org/abs/1912.07805

**時刻同期・遅延**
- T. Qin, S. Shen, *Online Temporal Calibration for Monocular Visual-Inertial Systems*, IROS 2018 — https://arxiv.org/abs/1808.00692
- S. B. Moon, P. Skelly, D. Towsley, *Estimation and Removal of Clock Skew from Network Delay Measurements*, INFOCOM 1999
- Out-of-Sequence Measurement / retrodiction — https://www.mathworks.com/help/fusion/ug/handle-out-of-sequence-measurements-with-filter-retrodiction.html
- Allan 分散による慣性センサ雑音解析 — https://www.mathworks.com/help/fusion/ug/inertial-sensor-noise-analysis-using-allan-variance.html

**RoboCup SSL**
- L. Cavalcanti et al., *Improving Inertial Odometry Through PSO in the RoboCup SSL*, RoboCup 2023 — https://link.springer.com/chapter/10.1007/978-3-031-55015-7_8
- RobôCIn, *Onboard Perception and Localization for Resource-Constrained Dynamic Environments*, JINT 2025 — https://link.springer.com/article/10.1007/s10846-025-02259-8
- TIGERs Mannheim, *ETDP 2024* — https://ssl.robocup.org/wp-content/uploads/2024/04/2024_ETDP_TIGERsMannheim.pdf
- S. Zickler et al., *SSL-Vision*, RoboCup 2009 — https://link.springer.com/chapter/10.1007/978-3-642-11876-0_37

**MCAP**
- MCAP 仕様 — https://mcap.dev/spec
- Foxglove MCAP Go 実装 — https://github.com/foxglove/mcap/tree/main/go/mcap
