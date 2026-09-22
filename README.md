# RACOON-Pi v3

ssl-RACOON-Controller などから送信された指令値をもとに、高速に情報を受信・制御を行うロボット側ソフトウェアです。

**Pi 4B（UART）** と **Rock5A（SPI）** の2ボードに対応しています。ビルド時に `-tags` でボードを明示指定してください。

> **Note:** 旧 `rock5a` ブランチの実装は本リポジトリに統合済みです。`rock5a` ブランチは廃止しました。

## ディレクトリ構成

```
cmd/
  racoon-pi3/          # メインエントリポイント
  dip_test/            # Rock5A DIP 診断ツール
  spi_test/            # Rock5A SPI 診断ツール
internal/
  app/                 # 起動・goroutine オーケストレーション
  state/               # 共有状態・データ構造
  link/                # UART/SPI 共通リンクロジック
  receive/             # AI / カメラ UDP 受信
  mw/                  # RACOON-MW へのマルチキャスト送信
  api/                 # HTTP API
  upgrade/             # 自動アップデート
  pi4/                 # Pi 4B 専用（UART, go-rpio）
  rock5a/              # Rock5A 専用（SPI, rock5a-gpio-go, sysfs PWM）
proto/                 # Protobuf 定義・生成コード
camera/                # カメラ処理（Python）
  capture/             # ボード別カメラ入力（pi4=Picamera2 / rock5a=V4L2）
  detect/              # color.py（HSV 検出）, calib.py（YOLO キャリブ）
  transport/           # UDP 送信・エンコード
  yolo/                # git submodule: Rione/ssl-YOLO-Detection
```

## クローン

カメラの YOLO モデルは git submodule（[Rione/ssl-YOLO-Detection](https://github.com/Rione/ssl-YOLO-Detection)）として `camera/yolo/` に含まれます。submodule ごと取得してください。

```bash
git clone --recurse-submodules https://github.com/Rione/ssl-RACOON-Pi3.git

# 既にクローン済みの場合
git submodule update --init --recursive
```

## ビルド

```bash
# Raspberry Pi 4B（UART / go-rpio）
go build -tags pi4 -o racoon-pi3 ./cmd/racoon-pi3

# Rock5A（SPI / rock5a-gpio-go）
go build -tags rock5a -o racoon-pi3 ./cmd/racoon-pi3

# Rock5A 診断ツール（開発 PC 上で Linux/arm64 向けにクロスビルドしてボードへ配置）
GOOS=linux GOARCH=arm64 go build -tags rock5a -o dip-test ./cmd/dip_test
GOOS=linux GOARCH=arm64 go build -tags rock5a -o spi_test ./cmd/spi_test
# 例: scp または ssh 経由で Rock5A へコピー
# scp spi_test root@<robot>:/root/spi_test
```

Rock5A 上で `go build` した場合は `GOOS`/`GOARCH` は不要です。Mac 等でビルドしたバイナリをそのままコピーしても **アーキテクチャ不一致で動きません**（または古いバイナリのまま）。必ず `GOOS=linux GOARCH=arm64` でビルドしてください。

タグ未指定の `go build .` は不可です。

## 実機で試す（手で載せる）

今ロボットで動いているのは Pi2 で、Pi2 は Pi2 のリリースしか取りに行きません。
機体上の配置は Rock5A の場合次のとおりです（2026-09-22 に robot 15 で確認）。

| もの | 場所 |
|---|---|
| 起動設定 | `/etc/systemd/system/ssl-racoon.service`（写しは [`scripts/ssl-racoon.service`](scripts/ssl-racoon.service)） |
| Pi2 のバイナリ | `/root/racoon-pi2-rock5a` |
| 作業ディレクトリ | `/root`（ボール色のしきい値は `/root/threshold.json`、カメラの Python は `/root/camera/`） |

そのため **Pi3 は最初の 1 回を手で載せます**。どのブランチのコードでも、ビルドして載せれば確かめられます
（master やリリースを経由する必要はありません）。開発ビルドは版番号が開発版扱いになり、**自己更新しません**。

Pi2 の置き場所を上書きせず、**別のディレクトリ**に置きます。戻すときはサービスを起動し直すだけで Pi2 に戻れます。

```bash
# 開発 PC で（確かめたいブランチのまま）
GOOS=linux GOARCH=arm64 go build -tags rock5a -o racoon-pi3 ./cmd/racoon-pi3
ssh root@<robot> 'mkdir -p /root/racoon-pi3'
scp racoon-pi3 root@<robot>:/root/racoon-pi3/
scp -r camera root@<robot>:/root/racoon-pi3/   # 搭載カメラを使う場合
```

```bash
# ロボット上で
cp /root/threshold.json /root/racoon-pi3/            # ボール色の HSV しきい値を引き継ぐ
sudo systemctl stop ssl-racoon.service                # Pi2 を止める
cd /root/racoon-pi3 && sudo ./racoon-pi3 -dryrun      # まずは車輪に指令を送らずに起動
```

- **必ず置き場所へ `cd` してから起動してください。** `threshold.json` は作業ディレクトリから読み
  （無ければ既定値で新しく作る）、カメラの Python（`camera/`）はバイナリと同じディレクトリから起動します。
- `-dryrun` は SPI へ速度・ドリブル・キックを送りません。受信・通信・ログを、ロボットを動かさずに確かめられます。
  問題なければ `-dryrun` を外して走らせます。
- **Pi2 に戻す**: Ctrl+C で止めて `sudo systemctl start ssl-racoon.service`。Pi2 の置き場所には触れていないので元どおりです。
  ロボットを再起動した場合も Pi2 が起動します。
- 自己位置推定の計測（`-loclog` など）は [`docs/localization-p1-runbook.md`](docs/localization-p1-runbook.md) を参照してください。

### 本番の載せ替え（Rock5A）

機体を Pi3 で常用にするときの手順です。**リリース版（タグを付けて作ったもの）を入れてください。**
開発ビルドは自己更新しないので、入れたまま放置すると以後の更新を受け取れません。

```bash
# ロボット上で（例: v7.0.0）
cd /root
curl -fL -o /tmp/pi3.tar.gz \
  https://github.com/Rione/ssl-RACOON-Pi3/releases/download/v7.0.0/racoon-pi3-rock5a_7.0.0_linux_arm64.tar.gz
tar -xzf /tmp/pi3.tar.gz -C /root        # /root/racoon-pi3-rock5a と /root/camera/ ができる（.pt は上書きしない）

# 起動設定を Pi3 用に差し替える（開発 PC から scripts/ssl-racoon.service を持ってくる）
cp /etc/systemd/system/ssl-racoon.service /root/ssl-racoon.service.pi2.bak
cp ssl-racoon.service /etc/systemd/system/ssl-racoon.service
systemctl daemon-reload
systemctl restart ssl-racoon.service
journalctl -u ssl-racoon.service -f      # 起動ログを確認（Ctrl+C で抜ける）
```

- `threshold.json` と `camera/` は Pi2 と同じ `/root` を使うので、しきい値はそのまま引き継がれます。
- 追加設定 `ssl-racoon.service.d/wait-6ghz.conf`（Wi-Fi の 6 GHz 切替を待ってから起動する）は
  別ファイルなので、上の差し替えでは消えません。写しは [`scripts/ssl-racoon.service.d/`](scripts/ssl-racoon.service.d/) にあります。
  待ち合わせ先の `wifi-prefer-6ghz.service` は機体上にだけあり、中身は取得していません。機体に残ったままなので**載せ替えには不要**です
  （要るのは新しい SD カードで機体を一から作るときです）。
- **Pi2 に戻す**: `cp /root/ssl-racoon.service.pi2.bak /etc/systemd/system/ssl-racoon.service && systemctl daemon-reload && systemctl restart ssl-racoon.service`。
  Pi2 のバイナリ（`/root/racoon-pi2-rock5a`）は消していないので、そのまま起動します。
- Pi 4B の機体の起動設定はまだ取得していません（バイナリ名は `racoon-pi2-pi4` のはず）。ローカル制御は Rock5A の世代から
  始めるので、旧世代を Pi3 に載せ替えると決まったときに取得します。

## カメラ

カメラ処理は Python の `camera/` パッケージが担当します。通常運転では軽量な HSV + 輪郭検出のみを行い、検出結果を UDP（ポート 31133）で Go 本体へ送信します。Go 本体は起動時に `python3 -m camera` を実行し、ビルドタグに応じて環境変数 `RACOON_BOARD`（`pi4` / `rock5a`）を渡します。

### ボード別のカメラ入力

| ボード | バックエンド | デバイス |
| ------ | ------------ | -------- |
| Pi 4B | Picamera2（MIPI CSI） | Picamera2 既定（CSI ポートはオーバーレイで指定） |
| Rock5A | OpenCV V4L2 | `/dev/video11`（既定）。`threshold.json` の `cameraDevice` で上書き可 |

### Pi 4B の CSI オーバーレイ（IMX219 / OV5647）

Pi 4B には CSI コネクタが **CAM0** と **CAM1** の 2 つあります。接続ポートに応じて `/boot/firmware/config.txt` でオーバーレイを指定する必要があります。`camera_auto_detect=1` だけでは検出に失敗することがあります（`rpicam-hello --list-cameras` が `No cameras available!` になる）。

| センサー | モジュール | オーバーレイ例 |
| -------- | ---------- | -------------- |
| IMX219 | Pi Camera v2 | `dtoverlay=imx219,cam0` または `dtoverlay=imx219,cam1` |
| OV5647 | Pi Camera v1.3 | `dtoverlay=ov5647,cam0` または `dtoverlay=ov5647,cam1` |

手動設定の例（CAM1 に IMX219 を接続した場合）:

```ini
camera_auto_detect=0
dtoverlay=imx219,cam1
```

- **起動時のオーバーレイ自動選択**: `scripts/select-pi4-camera.sh` が `imx219,cam0` → `imx219,cam1` → `ov5647,cam0` → `ov5647,cam1` の順に試します。`scripts/racoon-camera-autoselect.service` は Pi（`/boot/firmware/config.txt`）と Rock5A（`/boot/dietpiEnv.txt`）のどちらでも動作します。

  ```bash
  sudo install -m 0755 scripts/select-pi4-camera.sh /usr/local/sbin/select-pi4-camera.sh
  sudo install -m 0755 scripts/select-rock5a-camera.sh /usr/local/sbin/select-rock5a-camera.sh
  sudo install -m 0644 scripts/racoon-camera-autoselect.service /etc/systemd/system/racoon-camera-autoselect.service
  sudo systemctl daemon-reload
  sudo systemctl enable racoon-camera-autoselect.service
  ```

- **映像プレビュー**: `http://<robot>:9191/color-tuner` でライブプレビューと HSV しきい値調整ができます。

- **上下左右反転（180°回転）**: `threshold.json` の `"cameraFlip180": true/false` で制御します。IMX219 の既定は反転あり（RACOON 取り付け向きに合わせた設定）。

- **キャプチャ解像度（Pi 4B）**: IMX219 は全画角を維持する最小解像度 **1640×1232**（2×2 ビニング）を既定にしています。640×480 はセンサー中央の切り出しになるため使いません。`threshold.json` の `"frameWidth"` / `"frameHeight"` で上書きできます。

### Rock5A のセンサー自動判別（IMX219 / OV5647）

Rock5A は Raspberry Pi Camera v1.3（OV5647）と v2（IMX219）の両方に対応しますが、それぞれ別のデバイスツリーオーバーレイを使い、**同時に有効化すると CSI パイプラインが壊れます**（`rkcif ... get remote terminal sensor failed`）。

| センサー | モジュール | オーバーレイ | I2C アドレス |
| -------- | ---------- | ------------ | ------------ |
| IMX219 | Pi Camera v2 | `rpi-camera-v2` | `0x10` |
| OV5647 | Pi Camera v1.3 | `rpi-camera-v1_3` | `0x36` |

- **起動時のオーバーレイ自動選択**: `scripts/select-rock5a-camera.sh` を systemd の oneshot（`scripts/racoon-camera-autoselect.service`）として `ssl-racoon.service` より前に実行します。センサーの subdev（`imx219` / `ov5647`）が出ていなければ `/boot/dietpiEnv.txt` の `overlays=` を片方だけになるよう書き換えて 1 回だけ再起動します。状態ファイル `/var/lib/racoon-camera-autoselect.state` で IMX219↔OV5647 を最大 1 巡しか試さないため、リブートループにはなりません。正しいオーバーレイが既に設定済み（通常運転）なら何もしません。Pi 4B 向けは上記「Pi 4B の CSI オーバーレイ」を参照してください。

  ```bash
  # 初回のみ（Rock 5A 端末上で。Pi 4B は select-pi4-camera.sh も併せて install）
  sudo install -m 0755 scripts/select-rock5a-camera.sh /usr/local/sbin/select-rock5a-camera.sh
  sudo install -m 0644 scripts/racoon-camera-autoselect.service /etc/systemd/system/racoon-camera-autoselect.service
  sudo systemctl daemon-reload
  sudo systemctl enable racoon-camera-autoselect.service
  ```

- **露出・ゲインのセンサー別設定**: `camera/sensor.py` は `/sys/class/video4linux/v4l-subdev*/name` から接続中センサーを判別し、センサーごとに正しいコントロール名・範囲で露出/ゲインを適用します。IMX219 は明るさを実効ゲイン（`gain`、最大 43663。`analogue_gain` ではない）で稼ぎ、動きブレを抑えるため露出は低め（既定 `exposure=1000` / `gain=5000`）にします。OV5647 は従来どおり `auto_exposure`/`gain_automatic`/`analogue_gain` を使用します。`threshold.json` の `cameraExposure` / `cameraGain` / `cameraAutoExposure` / `cameraSensorSubdev` で上書きできます。

- **上下左右反転（180°回転）**: センサーごとに既定値があります（**IMX219: 反転あり** / **OV5647: 反転なし**）。RACOON の取り付け向きに合わせた設定です。`threshold.json` の `"cameraFlip180": true/false` または環境変数 `CAMERA_FLIP180` で**接続中センサーに対して上書き**できます。個別の `cameraHFlip` / `cameraVFlip` も引き続き利用可能です。反転はソフトウェア側で行います（センサー側 flip は Bayer デモザイクの都合で色が緑に寄るため使いません）。

- **IMX219 の色補正**: Rockchip ISP + IMX219 ではドライバ AWB がなく緑被りが出やすいため、IMX219 接続時は既定で BGR ゲイン補正（`1.15, 0.78, 1.12`）を適用します。`threshold.json` の `"cameraColorGains": "1.15,0.78,1.12"`（B,G,R 順）で上書きできます。OV5647 では既定では適用しません。

### 依存パッケージ

```bash
pip install -r camera/requirements.txt
```

`picamera2` は Pi 4B のみ必要です。`ultralytics`（YOLO）はキャリブレーション時のみ遅延 import されます。

### ボール色キャリブレーション（`/calibballcolor`）

Raspberry Pi 上で常時 YOLO を動かすのは負荷が高いため、YOLO はキャリブレーション時のみ使用します。`GET /calibballcolor` を叩くと、カメラプロセスが 1 フレームを YOLO で推論し、検出したボールのバウンディングボックス中心と上下左右 4 点（計 5 点）から HSV を算出して `threshold.json` を更新します。以降は通常の HSV 検出が新しいしきい値で動作します（プロセス再起動不要）。

```bash
# ボールをカメラに写した状態で実行
curl http://<robot>:9191/calibballcolor
```

成功時はしきい値・バウンディングボックス・サンプル点・プレビュー画像（base64 JPEG）を含む JSON を返します。ボール未検出時は HTTP 400 を返します。

## 自動アップデート

GitHub Release からボード別バイナリを取得します。

仕組み:

- **起動時に 1 回だけ**確認します（`internal/app/run.go` → `internal/upgrade`）。取得先はこのリポジトリ（`Rione/ssl-RACOON-Pi3`）のリリースです。
- 新しい版があれば**実行中のバイナリ自身を上書き**し、リリースに同梱の `camera/` もバイナリの隣へ展開して、
  **プロセスを終了**します。再起動は systemd に任せています。
- 開発ビルド（タグの付いていないコミットや、未コミットの変更入りでビルドしたもの。版が `v0.0.0-…` や `…+dirty` になる）は確認そのものをしません。
- ロボットの起動時にこのプログラムを立ち上げるのは systemd の `ssl-racoon.service` です（Rock5A 用の写しは
  [`scripts/ssl-racoon.service`](scripts/ssl-racoon.service)）。`Restart=always` で立ち上げ直し、
  ネットワーク（と Wi-Fi の 6 GHz 切替）を待ってから起動します。起動時にネットワークが無いと、その回の確認は失敗し、次の起動まで再試行しません。

Public リポジトリのため `.env` や `GITHUB_TOKEN` は必須ではありません。`.env` がある場合は自動で読み込みます（API レート制限を避けたい場合に `GITHUB_TOKEN` を設定できます）。

| ビルド | Release アセット名（例） | フィルタ |
|--------|-------------------------|----------|
| Pi 4B | `racoon-pi3-pi4_7.0.0_linux_arm64.tar.gz` | `^racoon-pi3-pi4_` |
| Rock5A | `racoon-pi3-rock5a_7.0.0_linux_arm64.tar.gz` | `^racoon-pi3-rock5a_` |

Release の tar には Go バイナリと `camera/` の Python ソース（**YOLO モデル `.pt` は含まない**）が同梱されます。キャリブレーション用の重みは Release ごとに 1 つだけ別アセット（`racoon-pi3-yolo_<version>_last.pt`、約 23 MiB）として公開します。初回セットアップ時にロボットへ配置してください。

```bash
# バイナリと同じディレクトリで（git clone 時は submodule でも可）
./scripts/install-yolo-model.sh v7.0.0
# または手動:
# curl -fL -o camera/yolo/last.pt \
#   https://github.com/Rione/ssl-RACOON-Pi3/releases/download/v7.0.0/racoon-pi3-yolo_7.0.0_last.pt
```

自動アップデートはバイナリと Python のみ同期し、既にある `camera/yolo/*.pt` は上書きしません（毎回 ~23 MiB×2 の転送を避けるため）。

### 版番号とリリース

- **タグを push するとリリースされます**（`.github/workflows/main.yml` → goreleaser）。版番号はタグ名そのものです。
- **Pi3 の版番号は `v7.0.0` から始めます。** Pi2 の最終系列（v6.x）の続きとして数えるため、
  Pi2 と Pi3 の機体が混在する移行期でも、RAVEN の Robot Status で「v7 以上なら Pi3」と一目で分かります。
- 自己更新は **このリポジトリ（Rione/ssl-RACOON-Pi3）のリリース同士でしか比べません**。
  Pi2 で動いている機体は Pi2 のリリースしか見ないので、**Pi2 から Pi3 への最初の載せ替えは手作業**になります。
  一度 Pi3 を載せれば、以後は Pi3 のリリースで自動更新されます。
- Pi2 のリリースタグ（v0.1.0〜v6.x）はこのリポジトリへ持ち込んでいません。**Pi2 のタグを push しないでください**
  （goreleaser は直近のタグから版を決めるため、Pi3 が Pi2 の版番号でリリースされてしまいます）。

## Robot IDの決定方法

ロボットIDには、ロボットに内蔵されたDIPスイッチよりIDの検出を行います。
カバーと同じ色にIDを設定するようにしてください。

---

## Pi 4B（UART）

Raspberry Pi 4B 向け。STM との通信は UART（`/dev/serial0` @ 230400 baud）です。

### PIN ASSIGN / ピン配置

| 名称 | ピン番号/ポート名 |
| ------------- | ------------- |
| Serial(UART)  | /dev/serial0  |
| LED 1         | GPIO 18       |
| LED 2         | GPIO 27       |
| Button 1      | GPIO 22       |
| Button 2      | GPIO 24       |
| Buzzer        | GPIO 13(PWM)  |
| DIP 1         | GPIO 4        |
| DIP 2         | GPIO 5        |
| DIP 3         | GPIO 6        |
| DIP 4         | GPIO 25       |

UART を使用する際には設定が必要です。

```
sudo raspi-config
```

---

## Rock5A（SPI）

Radxa Rock5A 向け。STM との通信は SPI Master（`/dev/spidev4.0` @ 1 MHz, Mode0）です。送受信フレーム長は **20 バイト**（ヘッダ `0xFF` + ペイロード 18 バイト + フッタ `0xAA`）。受信ペイロードの先頭 11 バイトが有効データで、続く 7 バイトはパディング（`0x00`）です。ヘッダ・フッタ・パディングが不正なフレームは破棄されます。

### PIN ASSIGN / ピン配置

| 名称 | Rock5A GPIO | 物理ピン |
| ------------- | ------------- | ------------- |
| SPI           | /dev/spidev4.0 | - |
| LED 1         | GPIO4_A1 (bank4,portA,pin1) | Pin 12 |
| LED 2         | GPIO4_B2 (bank4,portB,pin2) | Pin 13 |
| Button 1      | GPIO4_B4 | Pin 15 |
| Button 2      | GPIO1_B0 | Pin 18 |
| Buzzer (PWM)  | Pin11 = PWM15 | Pin 11 |
| DIP 1         | GPIO1_B3 | Pin 7 |
| DIP 2         | GPIO1_B2 | Pin 29 |
| DIP 3         | GPIO1_B1 | Pin 31 |
| DIP 4         | GPIO1_B5 | Pin 22 |

ブザー PWM にはデバイスツリーオーバーレイ `rk3588-pwm15-m1` の有効化が必要です。

### SPI 診断 (`spi_test`)

```bash
sudo /root/spi_test -interval 8ms          # 本番と同じ 125Hz（SignalReceived のみ、EmgStop=0）
sudo /root/spi_test -once                  # 1 回送信（TX 20 バイト）
sudo /root/spi_test -interval 8ms -velx 500 -charge   # 走行テスト（DoCharge も付与）
sudo /root/spi_test -emgstop               # 起動直後 idle 相当（EmgStop=1、走行不可）
sudo /root/spi_test -interval 8ms -mismatch-only   # NG のみ表示
# Ctrl+C で OK/NG パケット統計を表示
```

Informations の bit0 (`EmgStop`) は **1=非常停止中** です。MW から指令を受けている本番状態では 0 です。

初期ホスト名 `DietPi` の場合、初回起動時に `racoon-XXXXX` 形式のホスト名へ自動変更されます。

## 経緯

このリポジトリは [ssl-RACOON-Pi2](https://github.com/Rione/ssl-RACOON-Pi2) の master（`f183f35`）から派生しました。
ロボット上のローカル制御（自己位置推定・追従制御）への移行をここで進めます。
Pi2 のコミット履歴はそのまま引き継いでいるので、それ以前の経緯は `git log` / `git blame` で辿れます。
Pi2 の `feat/#1-self-localization`（`8860429`）と `feat/#2-set-velocity`（`ae433a8`）も取り込み済みです。
