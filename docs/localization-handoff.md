# 自己位置推定 — 引き継ぎ文書

**宛先**: この作業を引き継ぐ人（人間・AI を問わず）
**日付**: 2026-09-13
**ブランチ**: `feat/self-localization`（未コミット）

---

## 0. 最初に読むべきこと

> **カルマンフィルタはまだ 1 行も実装されていない。**
>
> 今回入れたのは計画書の **P0（骨格）と P1（計測基盤）**だけである。
> 予測ステップも更新ステップも共分散伝播も存在しない。詳しくは §3。

| 文書 | 内容 |
| --- | --- |
| [`self-localization-plan.md`](./self-localization-plan.md) | 設計の全体計画（v1.0）。**P0〜P7 の定義はここ** |
| [`robot-command-protocol-requirements.md`](./robot-command-protocol-requirements.md) | 通信担当への要件 |
| [`localization-p1-runbook.md`](./localization-p1-runbook.md) | **P1 の実行手順と、実装中に見つかった計画の穴 4 件** |
| この文書 | 今回やったこと・やっていないこと・次の一手 |

---

## 1. 今回やったこと

### 1.1 新規パッケージ

| パッケージ | 行数 | 役割 | ビルドタグ |
| --- | ---: | --- | --- |
| `internal/localization` | 898 | 型定義、SE(2) 演算、4 輪オムニ運動学、**機体パラメータ同定** | **なし** |
| `internal/stmframe` | 638 | STM フレームのファイル駆動デコーダ、フレーム位相探索 | **なし** |
| `internal/loclog` | 875 | MCAP の書き出し・読み返し、単調時計 | **なし** |
| `internal/locadapter` | 740+ | vision 受信、SPI 記録、加振ドライバ | **なし** |
| `internal/timesync` | 81 | クロック同期のインタフェース（実装は P2） | **なし** |
| `cmd/loc_ident` | 260 | MCAP から機体パラメータを同定する CLI | **なし** |

**ビルドタグを付けていないのが重要。** 既存の `internal/link` `internal/pi4` `internal/rock5a` は
`//go:build pi4 || rock5a` が付いていて開発 PC（macOS）ではビルドもテストもできない。
新パッケージは全部タグ無しなので、実機なしで `go test ./...` が回る。**この性質を壊さないこと。**

テストは 1,200 行ほど。実機に繋がずに検証できる範囲は全部テストにしてある。

### 1.2 既存ファイルへの変更（最小限）

| ファイル | 変更 |
| --- | --- |
| `internal/rock5a/spi.go` | `conn.Tx()` の**前後で時刻を取り**、`link.NotifySPI` を呼ぶ（+13 行）。受信処理のロジックは一切変えていない |
| `internal/link/observer.go` | **新規**。SPI 観測フックと速度指令の差し替えフック |
| `internal/link/common.go` | `PrepareSendData` に `applyVelocityOverride` を 1 行追加（+5 行） |
| `internal/app/localization.go` | **新規**。計測基盤の起動 |
| `internal/app/run.go` | フラグ 6 個と `startLocalization` の呼び出し（+19 行） |
| `internal/state/state.go` | 設定用のグローバル変数と後始末フック（+52 行） |
| `proto/pb_src/ssl_vision_*.proto` | **新規**。RAVEN から取り込み、Go を生成 |
| `go.mod` / `go.sum` | `gonum` と `foxglove/mcap` を追加 |

**`-loclog` を指定しなければフックは登録されず、従来と完全に同じ動作になる。**

`internal/link/config.go`, `internal/rock5a/board.go`, `internal/rock5a/config.go` に
**空白のみの差分**が入っている。`gofmt -w` の巻き添えで、元が未整形だったため。

### 1.3 環境

- **Go がこのマシンに入っていなかった**ので `brew install go`（1.27.1）した
- `gonum.org/v1/gonum`（擬似逆・Cholesky）、`github.com/foxglove/mcap/go/mcap`（MCAP）を追加
- `protoc-gen-go` を `go install`。生成は既存と同じ protoc 3.21.12 で行った

---

## 2. 動くことを確認した範囲

```
go test ./...                 全パス
go test -race ./internal/...  全パス
go build -tags rock5a ./...   OK
go build -tags pi4 ./...      OK
```

**実機では一切検証していない**（ロボット未接続）。合成データでの通し確認のみ:

新世代 STM の符号反転（全輪 −1）＋ FL/FR 入れ替わり＋旧世代寸法（r=27mm, R=85mm）を仕込んだ
合成ログを MCAP に書き、`loc_ident` で読み返したところ

```
wheelSlotOrder  [3 1 2 0]         <- 仕込んだ {FR,BL,BR,FL} と一致
wheelSigns      [-1 -1 -1 -1]     <- 一致
wheelRadiusMm   [27.02 ...]       <- 仕込んだ 27 mm
momentArmMm     85.00             <- 仕込んだ 85 mm
```

を復元できた。**符号とホイール順序の検出が原理的に動くことは確認済み。**

---

## 3. カルマンフィルタは何も入っていない

計画書 §7.1 が挙げているファイルのうち、**実装したのは 4 つだけ**である。

| 計画 §7.1 のファイル | 状態 |
| --- | --- |
| `types.go` | ✅ 実装（`Stamp` / `Pose2` / `Estimate` / `Health` / 各種サンプル型） |
| `manifold.go` | ✅ 実装（SO(2)/SE(2) の exp・log、合成、角度正規化） |
| `kinematics.go` | ✅ 実装（4 輪 ⇔ body、擬似逆、冗長残差） |
| `config.go` | ✅ 実装（幾何パラメータの JSON） |
| **`estimator.go`** | ❌ **無い**。`Predict` / `AddWheel` / `AddVision` / `At` / `PredictAhead` のどれも無い |
| **`eskf.go`** | ❌ **無い**。誤差状態フィルタ本体。予測も更新も共分散も無い |
| **`oosm.go`** | ❌ **無い**。リングバッファと retrodiction |
| **`slip.go`** | ❌ **無い**。外乱オブザーバ、ZUPT / ZARU |
| **`robust.go`** | ❌ **無い**。Huber 重み、適応 R |
| **`health.go`** | ❌ **無い**。発散検知・リセット |

つまり `localization.Estimate` は**型が定義されているだけで、それを埋める者が居ない**。
`Health` も enum があるだけである。

**フィルタに繋がる下地として今回入ったもの**は次の 3 つ。

1. **`manifold.go`** — 姿勢誤差を接空間で扱うための SE(2) の `Exp` / `Log`。
   誤差状態フィルタの予測（`p ← p + R(θ)v dt` を厳密に積む）と、
   OOSM の retrodiction で状態を過去へ戻すのに要る。
   `LogSE2` は教科書の式 `a = ω·sin ω / (2(1−cos ω))` を**使っていない**。
   分母が ω² の速さで 0 に近づき ω=1e-6 で既に有効数字が 4 桁まで落ちるため、
   半角に直した `(ω/2)·cot(ω/2)` にしてある。テストで固定済み。

2. **`kinematics.go`** — 観測モデル `ω_wheel = M·[vx, vy, ω]` の行 `M` を返す `Row(i)`。
   これが**そのまま観測ヤコビアン**になる。擬似逆は起動時に 1 度だけ計算して配列に持つので、
   ホットパスは 0 アロケーション（テストで固定）。
   計画 §4.5 の通り、観測は 4 次元のまま扱う想定で `Residual` も用意してある。

3. **`config.go` / `ident.go`** — フィルタを書く前に潰すべき機体パラメータ。

**フィルタを書き始める前に P1 のログを取ること。** 計画 §9 の通り、
RAVEN の EKF が「効果が測れなかった」のは数式の問題ではなくパラメータの問題である可能性が高い。
同じ失敗を繰り返さないための順序がこうなっている。

---

## 4. 計画から意図的に変えた点

### 4.1 符号同定の方法（§8）

計画は「**1 輪ずつ既知方向に動かす**」としているが、**これは実行できない**。
SPI の下りは `VelX` / `VelY` / `VelAng` の 3 つしか持たず、車輪個別の指令が存在しない。
STM 側が逆運動学で 4 輪へ配分するので、1 輪だけ回す経路が無い。

代わりに **3 自由度を順に、3 段階の振幅で加振**し、各スロットの

```
ω_j = a_j·vx + b_j·vy + c_j·ω
```

を最小二乗で解く方式にした（`localization.Identify`）。運動学の基準形と見比べると

```
|(a, b)| = 1/r            → 車輪半径
atan2(a, −b) = A or A+π   → 取付角と符号
c = −s·R/r                → モーメントアーム
```

となり、**符号・ホイール順序・車輪半径・取付角・モーメントアームが同時に出る。**

**限界（重要）**: 基準にしているのは「STM へ送った速度指令」なので、同定されるのは
**STM 自身の逆運動学とエンコーダ経路の整合性**であって絶対寸法ではない。
**符号（A-5）とホイール順序（A-4）はこれで確定するが、寸法（A-1〜A-3）の絶対値は確定しない。**
STM 側の運動学パラメータが間違っていれば、その誤差ごと吸収する。
絶対寸法には vision を基準にした同定が要り、それは **P2 の後**でないと成立しない。
`loc_ident -ref vision` は現状そう明示してエラーを返す。

### 4.2 P1 では timesync を使わない

計画 §5.3 の凸包法は P2 である。P1 の間は `timesync.Arrival`（到着時刻をそのまま使う）を既定にし、
**生の `t_capture` / `t_sent` / 到着時刻を全部 MCAP に記録**してある。
凸包法は**ログからオフラインで開発・検証してから**実機へ入れられる。

この間、vision 観測には片道遅延ぶんの系統誤差が丸ごと残る（2 m/s で 20 ms なら 40 mm）。
**P2 までは §10 の精度目標を評価できない。**

---

## 5. 実装中に見つかった、計画に無かった問題

詳細は [`localization-p1-runbook.md`](./localization-p1-runbook.md) §4。要点だけ:

1. **フレーム位相探索が計画から抜けていた。** 既存実装は 40 バイト窓を総当たりしていた。
   `stmframe.Decoder.Find` へ移し、既存実装と同じ位置を選ぶことをテストで固定した。
   さらに**窓に 2 フレーム入り得る**ので、`/in/spi` の `frameCount` が 2 以上なら
   時刻が 1 周期（8 ms）ずれ得る。`τ_stm` を測るときに効く。

2. **パディング 0 がフレーム同期の判別力を担っている**（9 バイト → IMU を載せると 3 バイト）。
   回路担当への依頼（§12-B3）に **CRC 1 バイト**を追加する必要がある。

3. **ロボットが自分のチーム色を知らない。** SSL-Vision は青黄を分けて送り `robot_id` はチーム内採番なので、
   色が無いと自機を特定できない。暫定で `-team` フラグにしたが、
   **取り違えると相手チームのロボットを自機だと思い込む**。
   プロトコル要件書 §1 に `team_color` を足す依頼が要る。

4. **ログのローテーションが未設計。** 試合 10 分あたり数十 MB。容量監視も削除も無い。

---

## 6. 次にやること

### 6.1 実機が繋がったら（最優先）

```
racoon-pi2 -loclog /var/log/racoon -team blue -locident   # 約 40 秒、自走する
loc_ident -log /var/log/racoon/racoon-loc-*.mcap -v
loc_ident -log ... -out geometry.json
```

これで **A-4（ホイール順序）と A-5（符号）が確定する。**
**確定するまでフィルタを書き始めないこと。** 誤差が出たときに切り分けられなくなる。

同時に取れるもの:
- `/in/vision_meta` の `lostFrames` → マルチキャスト欠落率（D-1）
- `/in/vision_meta` の `processing_s` → vision の処理遅延（D-2）
- `/in/spi` の `dt_ns` → SPI 周期の実測ジッタ（D-4）

### 6.2 実機がなくても進められること

- **P2（timesync）**: 凸包法によるスキュー・オフセット推定。
  `timesync.Sync` インタフェースは切ってあるので、`Arrival` の隣に実装を足すだけ。
  合成データでのテスト（既知スキュー +50 ppm、バースト遅延、AP ローミング相当のステップ）は
  計画 §9 の検証項目 2 にある。
- **P3（誤差状態フィルタ）の骨格**: `kinematics.Row(i)` が観測ヤコビアン、
  `manifold` が予測の積分を提供済み。ただし **P1 の符号確定が前提**（計画 §11 の P3 の欄）。

### 6.3 他担当への依頼

| 宛先 | 内容 |
| --- | --- |
| 回路担当 | 計画 §12-B の 7 項目 **＋ CRC 1 バイト**（§5-2） |
| 通信担当 | 要件書の通り **＋ `team_color`**（§5-3） |

---

## 7. コードを触るときの約束

1. **`localization` / `timesync` / `stmframe` / `loclog` / `locadapter` にビルドタグを付けない。**
   開発 PC でテストが回らなくなる。
2. **`localization` と `timesync` から `time.Now()` を呼ばない。** 時刻は `Stamp` で引数として渡す。
   同じ MCAP から同じ出力が出ることが、オフラインチューニングと回帰テストの前提（計画 §7.4）。
3. **SPI のホットパスで新しくアロケートしない。** `TestSPIRecorderDoesNotAllocate` など
   3 本のテストが 0 アロケーションを固定している。`any` へのボックス化でも落ちる。
4. **記録が詰まっても推定と制御を止めない。** 捨てて件数を数える。
5. **ログのフィールド名に単位を書く**（`wheelFL_rad_s`）。
   このリポジトリは既に `FlWheelSpeedRadS` の中身が m/s という事故を起こしている。
6. **`state.FlWheelSpeedRadS` を参照しない。** 名前と単位が一致していない。
