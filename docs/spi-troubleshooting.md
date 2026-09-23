# ロボットが動かないときの切り分け手順

Rock5A → メインボード (STM32) の SPI が怪しいときに、上から順に潰す。
各手順は「次にどちらへ進むか」まで書いてあるので、飛ばさずに順にやること。

道具: `cmd/spi_diag` (車輪は回らない) と `cmd/drive_test` (**車輪が回る**)。

```
GOOS=linux GOARCH=arm64 go build -tags rock5a -o spi_diag ./cmd/spi_diag
ssh root@<ロボット> 'cat > /root/trajpoc/spi_diag && chmod +x /root/trajpoc/spi_diag' < spi_diag
```

## 配線 (Rock 5A / spi4m2)

`/sys/kernel/debug/pinctrl/*/pinmux-pins` と `/sys/kernel/debug/gpio` で確認した実機の割り当て。

| GPIO | 40 ピンヘッダ | 役割 |
|---|---|---|
| gpio1-0 (32) | PIN_21 | MISO (STM → Pi) |
| gpio1-1 (33) | PIN_19 | MOSI (Pi → STM) |
| gpio1-2 (34) | PIN_23 | CLK |
| gpio1-3 (35) | PIN_24 | CS (spi4m2-cs0) |

**Rock5A は CS を必ず駆動する。** これは periph の設定ではなく devicetree の `spi4m2-cs0` なので、
こちらから「CS を出さない」ことはできない。STM 側が `SPI_NSS_SOFT` なら CS は無視されるだけで害は無い。

---

## 手順 0: Rock5A 自身は正常か (疑わしいときだけ)

PIN_19 (MOSI) と PIN_21 (MISO) をジャンパ線で繋いで実行する。

```
/root/trajpoc/spi_diag -loopback
```

- **送った通り返る** → Rock5A の SPI は正常。線を外して手順 1 へ。
- **全部 ff** → 線が繋がっていないか Rock5A 側の問題。ここから先へ進んでも無駄。

## 手順 1: STM は返事をしているか (車輪は回らない)

```
/root/trajpoc/spi_diag -frame 21
```

4 つの速度 × 4 つの mode を総当たりして、受信バイトのうち `ff` 以外が何バイトあるかを数える。

MISO は誰も駆動しなければプルアップで 1 のまま、つまり `ff` が並ぶ。
STM が返す中身には電圧 0 や車輪 0 など **必ず 0 のバイトが混ざる**ので、
何千バイト読んで `ff` しか無いなら「ずれている」ではなく「駆動していない」が確定する。

- **`ff` 以外がある** → STM は生きている。噛み合っていないだけ。→ 手順 5 へ。
- **全部 `ff`** → STM は MISO を一切駆動していない。→ 手順 2 へ。

## 手順 2: 下り (Pi → STM) は届いているか — LED2 を見る

メインボードの LED2 は `is_signal_received` を映す。速度 0 のまま送り続けて目で見る。

```
/root/trajpoc/spi_diag -led -sec 20
```

- **LED2 が点く** → 下りは届いていて、STM はフレームを解釈できている。
  壊れているのは**返事 (MISO) だけ**。→ 手順 4 (C) へ。
- **LED2 が点かない** → STM は受信もできていない。SPI2 そのものが動いていない。→ 手順 3 へ。

LED2 が見えない・自信が無いときは、ドリブラで代用する (車輪は回らない)。

```
/root/trajpoc/spi_diag -dribble 80 -sec 5
```

回れば「下りは届いている」と同じ意味になる。

## 手順 3: SPI2 が動いていない — ファーム側で確認すること

疑わしい順。上から潰す。

**A. NSS が `SPI_NSS_HARD_INPUT` のままで、CS が繋がっていない**
`hspi2.Init.NSS` を確認する。`SPI_NSS_HARD_INPUT` なら PB12 が high のあいだ SPI2 は無効で、
MISO は high-Z のまま = こちらから見て全部 `ff`。これは今回の症状と完全に一致する。
CS を配線していないなら `SPI_NSS_SOFT` に戻す (加えて `SPI_CR1_SSI` を立てる)。

**B. `MX_SPI2_Init()` が走っていない / slave になっていない**
`hspi2.Init.Mode` が `SPI_MODE_SLAVE` か。`Robot_RockArm()` (最初の受信待ち) が
成功しているか。ここで止まっていると以降が動かない。

**C. 起動直後に落ちている**
HardFault ハンドラに入っていないか。ブザー・LED の初期化は通っているか。

**D. 電源**
メインボードの電源 LED。電池電圧が足りているか。

**切り分けの決定打: 直前の動いていたビルドを焼き直す。**
今朝の時点では同じ `spi_diag` で電圧 25.0 V と車輪と IMU が読めていた。
そのビルドに戻して手順 1 が通れば、**間の変更 (NSS) が原因**と確定する。通らなければ
ファームではなくハード (配線・電源・破損) を疑う。

## 手順 4: 返事だけ壊れている (LED2 は点くのに `ff`)

- `PC2` (SPI2_MISO) の GPIO が `GPIO_MODE_AF_PP` / `GPIO_AF5_SPI2` のままか。
- 送信バッファに実際に中身を積んでいるか (`HAL_SPI_TransmitReceive_DMA` の TX 側)。
- MISO の線が物理的に切れていないか (手順 0 のジャンパで Pi 側は潰せている)。

## 手順 5: 返事はあるが噛み合わない

`spi_diag` の表で、どの速度・どの mode で `ff` 以外が出たかを見る。

- **本番と違う mode でだけ出る** → ファーム側の CPOL/CPHA が変わっている。
- **本番と同じ設定で出るが中身が壊れている** → フレーム長 (20/21) かバイト順の食い違い。
  `docs/SPI_PROTOCOL.md` と `internal/rock5a/frame.go` を突き合わせる。
- **たまに壊れる** → 同期ずれ。`internal/rock5a/spi.go` の窓探索が効いているか確認する。

## 手順 6: 通信が通ったら、走らせる前に段階を踏む

**ここから車輪が回る。周りを空けてから。**

```
/root/trajpoc/drive_test -vel 0 -dribble 80 -sec 3   # ドリブラだけ
/root/trajpoc/drive_test -vel 200 -sec 3             # 3 秒前進
```

車輪が回らないときは STM → モータードライバ側を見る (`docs/imu-requirements.md` 参照)。
ドライバが止める条件: 指令が 0.5 秒途切れた (LED1)、60 °C 超え (LED3)、
電源が 15-30 V の外 (LED3)、ゲートドライバ未準備 (LED2)。

---

## 記録: 2026-09-23 の測定

| 時点 | ファーム | 手順 1 の結果 | 車輪 |
|---|---|---|---|
| 午前 | IMU 入り (電圧 ×10) | 電圧・車輪・IMU すべて読めた | 回らない |
| 昼 | IMU 入り (電圧 ×5) | 読めた (25.0 V) | 回らない |
| NSS 変更後 | ? | **16 通り 8400 バイト全部 `ff`** | 回らない |

NSS を変える前は返事があったので、手順 3 の A が最も疑わしい。
