# MainBoard 側から見る SPI 受信の確認手順

> **2026-09-23 実施済み。結果は「下りは完全に健全」。**
> 送った値がそのまま届き、復号も正しく、エラー 0 (`valid:2442 invalid:18`)。
> 以下は手順の記録。同じ確認が要るときに再利用する。


ファームウェア担当者向け。STM32 (MainBoard_V26_2) にデバッガか UART を繋いで、
**Rock5A からの MOSI が本当に届いているか**を確認したい。

所要時間は 10 分ほど。ロボットは走らない (最後の任意の手順を除く)。

---

## 何が起きているか

Rock5A 側から見ると、こうなっている。

| 経路 | ピン (Rock5A → STM32) | 状態 |
|---|---|---|
| CLK | PIN_23 → PB10 | 生きている |
| CS | PIN_24 → PB12 | 生きている |
| MISO | PIN_21 ← PC2 | **完璧**。電圧・車輪・IMU すべて正しく読める |
| MOSI | PIN_19 → PC1 | **ここが疑わしい** |

MISO で受け取るフレームはビットずれも取りこぼしも無い (250 フレーム連続で
ヘッダ `0xFF` とフッタ `0xAA` が揃う)。電圧 26.0 V、車輪ほぼ 0 rad/s、
yawRate 0.002 rad/s と、静止したロボットとして値も正しい。

`Robot_RockArm` は `HAL_SPI_TransmitReceive_IT` で送受信を 1 つの転送として行う。
こちらに正しい送信データが途切れず届いている以上、**その転送は毎回完了していて、
STM も毎回 21 バイトを受信バッファに取り込んでいる**はず。

それでも `is_signal_received` が立たない (LED2 が点かない、車輪が回らない)。
`info->status` を書いているのは `Robot_RockApplyRecvPacket` の
`info->status.data = data[17];` **1 箇所だけ**なので、
`Robot_RockFindFrame` がヘッダとフッタを一度も見つけられていないことになる。

**つまり、STM が受け取ったバイトがこちらの送ったものと違う。**
確認してほしいのはこの一点。

---

## 手順 1: Rock5A から決まった値を流す

Rock5A 側でこれを実行する (こちらでやるので、始めるタイミングだけ合わせてほしい)。

```
systemctl stop ssl-racoon.service      # 本番サービスを止める (必須)
/root/trajpoc/spi_diag -pattern 120    # 2 分間、125 Hz で流し続ける
```

流れるフレームは 21 バイト固定で、中身は偶然には出ない値にしてある。

```
FF D2 04 D2 E9 84 03 5A 00 00 nn nn 34 12 78 56 C1 C2 01 00 AA
```

| フィールド | 値 |
|---|---|
| `vel_x` | **1234** |
| `vel_y` | **-5678** |
| `vel_angular` | **900** |
| `dribble_power` | **0x5A** (90) |
| `kicker.straight` / `.chip` | 0 / 0 (撃たない) |
| `relative_position_x` | **毎フレーム 1 ずつ増える** |
| `relative_position_y` | **0x1234** |
| `relative_theta` | **0x5678** |
| `camera.x` / `camera.y` | **0xC1** / **0xC2** |
| `status` | **0x01** (emergency_stop。車輪は回らない) |

`relative_position_x` が増え続けることで、「止まった値を見ている」のか
「本当に流れている」のかも同時に分かる。

## 手順 2: STM 側で受信バッファを見る

### デバッガがあるなら

`src/unit/robot.c` の `Robot_RockUpdateSPI` に停止点を置き、
`rock_spi_rx_xfer` (21 バイト) と `info` をライブウォッチで見る。

### UART の printf で見るなら

`Robot_RockUpdateSPI` の `Robot_RockRxWindowPush(rock_spi_rx_xfer);` の直後に、
一時的にこれを入れる。制御ループは 1 ms なので 200 回に 1 回 = 約 0.2 秒ごと。

```c
{
  static uint32_t dbg_n = 0;
  if ((dbg_n++ % 200) == 0) {
    printf("RX:");
    for (int i = 0; i < ROCK_SPI_FRAME_SIZE; i++) printf(" %02X", rock_spi_rx_xfer[i]);
    printf("  find=%d\n",
           (int)Robot_RockFindFrame(rock_spi_rx_window, ROCK_SPI_RX_WINDOW_SIZE));
  }
}
```

`find` が -1 ならフレームが見つかっていない (= いまの状態)。0 以上なら見つかっている。

## 手順 3: 見えたもので判断する

| `rock_spi_rx_xfer` の中身 | 意味 | 次にやること |
|---|---|---|
| **全部 `00`** | MOSI に何も来ていない。線が Low に落ちているか未接続 | PIN_19 ↔ PC1 の導通を当たる |
| **全部 `FF`** | MOSI が開放。線が切れている | 同上 |
| **手順 1 の値どおり** | **MOSI は生きている。原因は別** | 手順 4 へ |
| 値は来るが並びが回っている | 線は生きている。バイト位置がずれているだけ | `Robot_RockFindFrame` が拾うはずなので `find` の値を見る |
| 値は来るが全体が化けている | ビットずれ。CPHA か NSS を疑う | `hspi2.Init` を確認 |

`0x00` と `0xFF` のどちらになるかは PC1 のプルの向き次第。**どちらでも「届いていない」で同じ。**

## 手順 4: 値が正しく届いていた場合

MOSI は無実なので、`Robot_RockFindFrame` から先を見る。

- `find` が -1 のままなら、ヘッダ `0xFF` とフッタ `0xAA` の位置関係が
  `ROCK_SPI_FRAME_SIZE` と噛み合っていない (フレーム長 21 の想定と実際のずれ)。
- `find` が 0 以上なのに `info->status` が更新されないなら、
  `Robot_RockApplyRecvPacket` の呼び出し条件を確認。

## 手順 5 (任意): 車輪まで通すか見る

**ここから車輪が回る。周りを空けてから。**
手順 1 のコマンドを `-pattern-signal` 付きで実行すると、`status` が
`0x20` (is_signal_received) になる。届いていれば `vel_x = 1234 mm/s` で走り出す。

```
/root/trajpoc/spi_diag -pattern 10 -pattern-signal
```

---

## 手順 6: ゲートから先を見る (2026-09-23 時点でここが残っている)

### 確認済み: ゲートは開いていた

`drive_test -vel 200 -sec 3` を流したときのファーム側ログ。

```
raw: FF C8 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 20 00 AA
vx:200 vy:0 w:0 drib:0 kick_s:0 kick_c:0 status:0x20
   (同じ行が 3 秒ぶん連続、err:0、valid:15709)
```

`0xC8` = 200、`status = 0x20` = `emergency_stop:0` / `is_signal_received:1`。
**3 秒間ずっとゲートの条件を満たしていた。それでも車輪は回らなかった。**

⇒ **SPI は上下とも完全に無実。原因は MainBoard → モータードライバの側。**

> ログの最後で `0x01` に戻るのは `drive_test` の終了処理 (速度 0 を 30 回 → 非常停止を 10 回)。
> `Rock SPI stall detected` は Rock5A 側で試験を流していない間の正常な表示。
> `invalid` が少し増えるのは、窓の中でフレームがバイト単位でずれた回
> (`raw: 00 ... 20 00 AA FF C8 00 ...` のような行)。`Robot_RockFindFrame` が拾えているので実害は無い。

### 残っているのはこの先

Rock5A 側でこれを流す (速度 0 なので**車輪は回らない**。ゲートの確認だけ)。

```
systemctl stop ssl-racoon.service
/root/trajpoc/drive_test -vel 0 -sec 120
```

送られる status は `0x20` = `emergency_stop:0` / `is_signal_received:1`。
ログに `status:0x20` と出るはず。そのうえで見てほしいのは次の 3 つ。

### (1) ゲートを通っているか

`src/mode/main_mode.c` の分岐に入っているか。LED2 (PB5) が点いていれば
`is_signal_received` は立っている。

```c
if (!r->info.status.emergency_stop && r->info.status.is_signal_received) {
```

### (2) `Robot_SendOmniDrive` まで届いているか

`src/unit/omni_drive.c` の `OmniDrive_SetVel` の末尾に一時的に入れる。

```c
{
  static uint32_t dbg_n = 0;
  if ((dbg_n++ % 200) == 0) {
    printf("[DRV] in: %d %d %d -> m: %d %d %d %d\n",
           vel_x, vel_y, vel_angle, m[0], m[1], m[2], m[3]);
  }
}
```

- **何も出ない** → ゲートで止まっている。(1) を見る
- **出るが `m` が全部 0** → 逆運動学か `MAF_Update` の側
- **`m` に値が入っている** → MainBoard は仕事をしている。原因はドライバ側 → (3)

### (3) モータードライバ側

`OmniDrive_Send` が 4 本の UART すべてに出ているか。
車輪の実速度は上りに乗ってきている (= ドライバからの戻りは生きている) ので、
**戻りは来るのに指令が効いていない**状態。

ドライバが止める条件:

| 条件 | 表示 |
|---|---|
| 指令が 0.5 秒途切れた | LED1 |
| 60 °C 超え | LED3 |
| 電源が 15〜30 V の外 | LED3 |
| ゲートドライバ未準備 | LED2 |

### おまけ: 電池電圧の ADC が不安定

上りの電圧が **生値 4 と 250 を行き来**している (実測は 24 V)。
`Robot_UpdateSensor` の `adc_val[0] * ADC2VOLT + BATTERY_VOLTAGE_OFFSET` が
安定していない。`HAL_ADC_Start_DMA` が止まっていないか確認してほしい。
走行の可否には直接効かないはずだが、Rock5A 側は電池残量の判断に使っている。

また、生値 250 が 25 V に相当することから、**このビルドの倍率は ×10 (0.1 V/LSB) に
戻っている**ように見える。以前 ×5 (0.2 V/LSB) に直してもらった変更が、
デバッグ用ビルドで巻き戻っていないか確認してほしい。

---

## 補足: こちらで既に潰してあること

- **本番サービスとの奪い合い** — `ssl-racoon.service` が 6 GHz Wi-Fi 復帰後に
  自動起動して `/dev/spidev4.0` を掴む。試験中は必ず止めている。
- **ビットずれ** — `SPI_NSS_SOFT` 版を焼いていた時間帯には実在した (3〜7 ビット)。
  現在の `SPI_NSS_HARD_INPUT` では 0 ビットで完全に安定しているので、
  **CS 配線は健全**と確定してよい (CS が死んでいれば SPI2 ごと無効になる)。
- **フレームの形** — `Robot_RockApplyRecvPacket` の並びと Rock5A の送信は完全に一致。
  `Robot_RockFindFrame` はヘッダとフッタしか見ておらず、parity は検査していない。
- **モード違い** — モードは `MainMode` 1 つだけ。
- **電圧・温度** — 電池 26〜27 V でドライバの停止条件 (15〜30 V) の内側。
