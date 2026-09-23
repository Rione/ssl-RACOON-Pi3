# 軌道追従の実機データ一式 (2026-09-22)

テスト機 1 台 (Rock5A、hostname `racoon-56011`、MAC `20:bd:1d:d7:3f:f2`、DIP の ID 15) で取った走行の記録。
「軌道の追従をロボット側 (Rock 5A) に移す価値があるか」を確かめるために回したもの。

- 場所: 会場のフィールド (SSL-Vision 224.5.23.2:10006、カメラ 0 のみ、6130 × 3880 mm)
- 日付: すべて 2026-09-22 (ファイル名の時刻はロボットの時計 = JST)
- 走行 48 本 (有効 40 本)。うち車輪の回転速度つきが 17 本
- 経緯・解釈・結論は [`reports/traj-poc-log.md`](reports/traj-poc-log.md) (これが本文)。仕組みの説明は [`reports/traj-poc-design.md`](reports/traj-poc-design.md)

## まず読むもの

| ファイル | 中身 |
|---|---|
| `index.csv` | **走行 48 本の一覧**。条件 (形・手法・先読み・ゲイン・電池) と指標 (位置・輪郭・遅れ・向き・到着) が 1 行 1 本 |
| `reports/traj-poc-log.md` | 作業の記録。何を確かめ、何が分かったか (§5 に結論が 21 項目) |
| `reports/raven-*-metrics.txt` | 今の RAVEN の追従器で同じ軌道を走らせたときの指標 |
| `reports/wheel-geometry-identification.txt` | 車輪の符号・取付角・半径・腕の長さの同定結果 |
| `reports/localization-dropout.txt` | vision を 0.3 s 抜いたときに自己位置推定がどこまで持つか |

## 置き場所

```
poc/      ロボット上で追従した走り (45 本ぶんのログ + 47 個の CSV)
raven/    今の RAVEN の追従器で走らせた 3 本 (tick ごとの記録・貼り付けた軌道・vision の生記録・評価用 CSV)
geometry/ 機体パラメータ (同定結果と、比較に使った設定)
reports/  指標のテキストと作業の記録
scripts/  解析スクリプト (円の膨らみ・向きのぶれ・車輪とスリップ)
```

## 走行の記録 (`poc/trajpoc-<日時>-<手法>-<補間>.csv`)

ロボット上の制御の周期 (SPI、125 Hz) ごとに 1 行。距離は mm、角度は rad、時刻は s。

| 列 | 意味 |
|---|---|
| `t_s` | 軌道の時刻 (0 が軌道の先頭。走り出しの前は負) |
| `tv_s` | その行で見ていた vision の**撮影**時刻 (同じ撮影が複数行に出る) |
| `vision_age_ms` | 撮影から今までの古さ |
| `held` | 1 なら vision が古くて速度 0 を出した周期 |
| `pose_x_mm`, `pose_y_mm`, `pose_theta_rad` | vision で見た機体の姿勢 (生の値) |
| `ref_*` | その撮影時刻の参照 (位置・向き・速度・角速度) |
| `cmd_world_*`, `cmd_omega_rad_s` | 出した速度指令 (ワールド座標) |
| `cmd_body_*` | 同じ指令を機体座標にしたもの (STM へ送る値) |
| `near_goal`, `pred_*` | 止まり際の寄せを使った周期と、そのときのスミス予測の位置 (`e5b972f` 以降) |
| `wheel_fl_rad_s` ほか 4 輪 | STM から届いた車輪の回転速度 (`3d3b5ed` 以降の 17 本のみ) |

`poc/trajpoc-<日時>.log` は同じ走りのロボット側のログ (条件と指標の要約、止まった理由)。

## RAVEN の走り (`raven/`)

| ファイル | 中身 |
|---|---|
| `raven-<日時>-id15.csv` | RAVEN の制御周期 (60 Hz) ごとの記録。時刻は `t_ns` (Java の `System.nanoTime`)。RAVEN の追跡が推定した姿勢・速度、参照、指令 |
| `raven-<日時>-id15-ref.jsonl` | 実際に追従器へ渡した軌道 (貼り付け後・絶対時刻)。先頭の注釈に `t0_ns` |
| `vision-<形>.csv` | 同じ走りの間に開発 PC で取った **vision の生の検出** (時刻は `CLOCK_MONOTONIC` = `System.nanoTime` と同じ時計) |
| `eval-<形>.csv` | 上の 2 つを突き合わせて PoC と同じ形に直した評価用の記録 |
| `traj-<形>.jsonl` | 走らせた相対軌道 (PoC と同じ入力ファイル) |

**位置の測り方は PoC と RAVEN で同じ**にしてある (どちらも vision の生の値、同じ `ComputeMetrics`)。
RAVEN 自身の推定値ではないので、RAVEN に不利でも有利でもない。

## 機体パラメータ (`geometry/`)

| ファイル | 中身 |
|---|---|
| `geometry-racoon-56011.json` | **採用値**。符号 −1 ×4、取付角 55.4 / 136.1 / −136.3 / −57.4°、半径 28.0〜29.3 mm、腕の長さ 74 mm |
| `ident-all-6runs.json` | 6 本 (往復 2・円・その場回転・8 の字・進行方向を向く円) からの同定そのまま。腕の長さだけ雑音で小さく出る |
| `ident-line-only.json` | 往復だけで同定した初期の結果 (横と回転が足りず取付角・腕が決まらない) |
| `default-dims-signs-flipped.json` | 既定の寸法のまま符号だけ −1 にしたもの (比較用) |

## 指標の読み方 (`index.csv` と `reports/*-metrics.txt`)

- `pos_rms` / `pos_max`: 参照の位置とのずれ [mm]。時刻も含めた追従の良さ
- `contour_rms`: 経路 (時刻を無視した線) からのずれ [mm]。形の正しさ
- `along_mean_mm` / `lag_ms`: 進行方向のずれと、それを速度で割った時間の遅れ。**負なら遅れ、正なら先走り**
- `cross_rms_mm`: 進行方向に直交するずれ
- `head_rms` / `head_max`: 向きのずれ [deg]
- `final_pos_mm` / `final_head_deg`: 軌道が終わって 0.8 s 後の到着の精度
- `cmd_accel_rms_mps2`: 指令の変化の激しさ (大きいほどがたついている)
- `settle_swing_deg` / `settle_cmd_max_mms`: 止まった後の向きの振れと並進の指令

## 使うときの注意

1. **無効な 8 本** (`valid=no`): 12:54 の 4 本は vision の模様が別の物を指しており、13:01 の 4 本は STM が無応答で機体が動いていない。列 `note` に理由がある
2. **指令が効くまでの遅れは電池の電圧で変わる**。午前 (21〜23 V) に合わせた先読み 85 ms は、満充電 (24 V) では効きすぎて 18〜20 mm 先走った。`battery_v` の列を見て比べること
3. **RAVEN 側は較正できていない**。`--calibrate` はこの機体で発散したので、入力むだ時間は既定の 100 ms。個体のゲインも無い。比較を公平にするには、個体設定のある機体で測り直す必要がある
4. 各条件 1 本のことが多い。同じ条件を繰り返したときの差は約 2 mm
5. 開始位置は走りごとに違う。軌道は**開始時の姿勢に貼り付ける**ので、機体座標で見た動きはどの走りでも同じ

## 取り方 (再現したいとき)

```bash
# ロボット上の追従 (ssl-RACOON-Pi3, feat/traj-poc)
ROBOT=<ip> VISION_IFACE=wlan0 scripts/trajpoc.sh deploy
ROBOT=<ip> VISION_IFACE=wlan0 scripts/trajpoc.sh run -shape circle -size 0.6 -speed 0.4 -- \
    -trajmethod ffp_vlead -trajlead 85 -trajvisionid 15
ROBOT=<ip> scripts/trajpoc.sh fetch     # CSV とログを取り出す
ROBOT=<ip> scripts/trajpoc.sh restore   # 終わったら Pi2 のサービスを戻す

# RAVEN の追従器 (ssl-RAVEN, feat/trajpoc-baseline)
vision_rec -team blue -id 15 -iface <nic> -sec 90 -out vision.csv &
./gradlew run -Draven.config.dir=<設定の写し> -Draven.team_color=blue \
    --args="--trajpoc=<軌道.jsonl> --trajpoc-id=15 --headless"
traj_eval -ref <…-ref.jsonl> -raven <….csv> -vision vision.csv
```
