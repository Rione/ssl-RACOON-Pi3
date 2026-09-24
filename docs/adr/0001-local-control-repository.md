# ADR 0001: ローカル制御を別リポジトリに切り出し、周期の本体を値の関数にする

- 状態: 採用 (2026-09-24)
- 決めた人: trowde
- 関係する合意: [ssl-RAVEN-Wing `docs/local-link.md`](https://github.com/Rione/ssl-RAVEN-Wing/blob/main/docs/local-link.md)、2026-09-24 の境界の取り決め 6 件

## 状況

`ssl-RAVEN-Wing` (以下 Wing) が Rock5A 側の入出力を担うことになり、責任分界が決まった。
Wing が RAVEN との UDP・MainBoard との SPI・キック/ドリブラ・非常停止と減速停止を持ち、
ローカル制御 (以下 Local) が自己位置推定と軌道追従を持つ。両者は同じ機体の上で別プロセスとして動き、
UDP (`127.0.0.1:30001` / `:30002`) で毎周期 1 通ずつやりとりする。

この repo (`ssl-RACOON-Pi3`) は、Wing の担当範囲と Local の担当範囲を**両方**持っている。
Local を作るにあたり、この repo をそのまま使うか、切り出すかを決める必要があった。

### 数えた事実

移植の対象を決めるために、パッケージごとに内部依存と隠れた入力を数えた (読んで判断せず、数えた)。

**持っていく核は、既に境界が切れていた。**

| パッケージ | 行数 (試験) | 内部依存 | `time.Now()` | 大域可変 |
|---|---|---|---|---|
| `internal/localization` | 4,638 (1,747) | **なし (葉)** | 0 | 0 |
| `internal/control` | 428 (407) | localization のみ | 0 | 0 |
| `internal/supervisor` | 231 (86) | localization のみ | 0 | 0 |

`localization` は時刻を全て `Stamp` として引数で受け、`time.Now()` を呼ばない。
`grep` で出た検出は「このパッケージは `time.Now()` を呼ばない」というコメント自身だった。
大域変数の 1 件は定数の索引表 (`paramIndices`)。

**汚れているのは、Wing に渡す部分だった。**

`internal/state` に 28 個のパッケージ変数があり、`api` / `link` / `mw` / `pi4` / `receive` / `rock5a` の
6 パッケージが無同期で読み書きしている。RACOON-Pi2 から引き継いだ形で、
Wing が「パッケージ変数を各 goroutine が無同期で読み書き → 不変スナップショットの atomic 差し替え」として
捨てたのと同じもの。**この 6 パッケージは全て Wing の担当範囲**である。

つまり切り出しは「汚れた層を捨てて、既にきれいな核を包み直す」作業になる。

### 参考にした 2 つの repo

- **ssl-RAVEN**: `control` に 11 の境界規則があり (`ArchitectureRulesTest`)、バイトコードから機械が確かめている。
  依存の向き、外に見せる型の一覧、設定を処理の中で読まないこと、事実のスナップショットを作る場所は 1 箇所、
  事実をフィールドに持たないこと、段が一本道であること、など。
- **ssl-RAVEN-Wing**: 17 パッケージが階層なしで並び、依存グラフは非循環。`config` / `mainboard` が葉で、
  `app` が全部を配線する。読みやすい。

**両者の差は「規則の検査」の有無**である。RAVEN にはあり、Wing と当 repo には無い。
当 repo は規律をコメントで書いているが、検査の無い約束は約束ではない。

## 決定

### 1. 新しい repo に切り出す

Local 専用の repo を作り、この repo からは Local の担当範囲だけを移す。

**移すもの** (本体 約 12,300 行 / 試験 約 7,200 行)

`localization` / `control` / `supervisor` / `locsim` / `loclog` / `locadapter` / `receive` /
`cmd/loc_replay` / `cmd/loc_ident` / `cmd/traj_gen` / `cmd/traj_eval` / `cmd/vision_*` /
`docs/traj-poc-log.md` などの実測の記録 / 機体諸元の JSON

**捨てるもの** (約 6,000 行。全て Wing の担当範囲)

`state` / `rock5a` / `stmframe` / `link` / `pi4` / `mw` / `api` / `upgrade` / `wheelgraph` / `app` /
`cmd/spi_diag` / `cmd/spi_test` / `cmd/dip_test` / `cmd/kick_test` / `cmd/drive_test`

この repo は凍結する。履歴と実測の記録は残るので、経緯はここを見れば追える。

### 2. 周期の本体を「値 → 値」の関数にする

**構造を決めた性質はこれ 1 つ。**

> リプレイと実機が、同じコードを通ること。

いま最も価値のある資産は、`loc_replay` が実機 56 本に対して実機と同じ推定器を回せることである。
IMU が読まれていなかったこと、ジャイロが向きの誤差を 10〜31% 改善することは、これで分かった。
この性質を**規律ではなく構造で**保証する。そのために:

> 250 Hz の本体を `Step(Snapshot) -> Output` の関数にし、その中に I/O を置かない。

実機は UDP から `Snapshot` を作り、リプレイはファイルから作る。呼び手が違うだけになる。

### 3. パッケージの構成

```
cmd/
  raven-local/     本体 (winglink → loop → winglink を配線するだけ)
  loc_replay/      リプレイ。実機と同じ loop.Step を呼ぶ
  loc_ident/       機体パラメータの同定
  traj_gen/        軌道の生成 (試験用)

internal/
  localization/    ESKF と観測・推定の値の型      [移す・葉]
  cycle/           1 周期の入出力 (Snapshot / Output)  [新規・葉]
  control/         追従                           [移す]
  supervisor/      安全検査 (枠・車輪 vs vision)   [移す]
  loop/            250 Hz の権限。Step(Snapshot) -> Output  [新規]
  winglink/        UDP と proto ↔ cycle の変換     [新規]
  geometry/        機体諸元の読み込み              [新規]
  loclog/          MCAP 記録                      [移す]
  locsim/          真値つき合成データ              [移す]

proto/             Wing から local_link.proto / raven_wing.proto を取得
config/            geometry-*.json (機体諸元の正本)
```

依存の向きは一本道で、循環しない。

```
localization  ← control, supervisor, loclog, locsim, cycle, geometry
cycle         ← loop, winglink
loop          ← cmd/raven-local, cmd/loc_replay
winglink      ← cmd/raven-local
```

- **`loop` は I/O を持たない。** だから `cmd/loc_replay` が同じ `Step` を呼べる。
- **`winglink` は `loop` を知らない。** 通信 (下) が判断 (上) を知らない、という向き。
- **`internal/state` は作らない。** 周期をまたぐ状態は推定器・軌道の控え・検査の窓の 3 つだけで、
  全て `loop` が持つ。数えられることが「権限が 1 つ」の確かめ方になる。

### 4. 番人を 5 つ置く

Go に ArchUnit は無いが、どれも 50 行程度で書ける。**規則は負例で落ちることを確認してから入れる。**

| # | 検査 | 守るもの |
|---|---|---|
| 1 | 依存の向きの DAG を固定する | 通信が判断を知らない |
| 2 | `proto/` を import してよいのは `winglink` だけ | 電文の形が核まで漏れない |
| 3 | `loop` / `localization` / `control` / `supervisor` に `time.Now()`・`os.`・`net.` が無い | 引数の関数であること |
| 4 | 同じ `Snapshot` の列を 2 回流して一致する | 決定論。リプレイが意味を持つ前提 |
| 5 | 実機 56 本に対する指標の固定 (許容つき) | 移植で壊れていないこと |

2 番が特に効く。Wing 側の proto はまだ変わる (`CLOCK_MONOTONIC` 化が確定している)。
変換を 1 箇所に閉じ込めれば、その変更が `winglink` の中で終わる。

5 番は bit 一致ではなく許容つきで比べる (CPU が違うと落ちるため)。

### 5. 速度・加速度の制限器は状態を持たない

Wing が毎周期 `state.applied_velocity` (直前に MainBoard へ送った速度) をくれるので、
制限器を `(目標, applied_velocity, 上限, dt)` の純粋関数にする。

- 状態の持ち主が 1 つ減る
- **自動復帰の段差が構造的に消える。** Wing が減速停止させた後、`applied_velocity` は減速途中の値なので、
  そこから続けるのが自動的に正しくなる。「復帰時に滑らかにつなぐ」処理を書かなくてよい

## 捨てた案

| 案 | 捨てた理由 |
|---|---|
| **この repo をそのまま使う** | Wing の担当範囲を抱えたままになり、どちらが持ち主か分からなくなる。`state` の大域変数も残る |
| **段を `control/` の下に入れ子にする** (RAVEN 流) | RAVEN は上位 19 パッケージあるので入れ子に意味がある。9 パッケージでは階層が情報を増やさない。段の一本道は import の向きの規則で守れる |
| **ports / adapters (ヘキサゴナル)** | 「リプレイ ≡ 実機」は値の境界だけで達成できる。呼び手が 2 つしかないのに interface を挟むのは遠回り |
| **1 つの大きなパッケージ + 薄い cmd** | 段の順序を強制する手段が消える。12,000 行は読めない |
| **観測の値の型を `localization` から外に出す** | 概念的にはその方が正しい (事実と推定は持ち主が違う)。だが検証済みの 4,638 行 + 試験 1,747 行に手を入れる代償が大きい。**歴史的理由としてここに記録し、二つ目の推定器を作るときに再検討する** |
| **`state` パッケージを作る** | 逃げ出す対象そのもの |

## 帰結

**楽になること**

- リプレイと実機の食い違いが構造的に起きない
- Wing 側の proto の変更が `winglink` の中で終わる
- 大域状態が無くなるので、どのスレッドが何を書くかを追う必要がなくなる
- 制限器が純粋関数になり、復帰の段差の処理が要らなくなる

**苦しくなること**

- 実機で走らせるには Wing と Local の 2 プロセスが要る。片方だけでは動かない
- proto の版が両 repo にまたがる。`version` の不一致で起動を拒否する仕掛けが要る
- 番人 5 つを書いて保守する手間が増える

**払うべき宿題**

- `locadapter` / `receive` の入口を Wing の `WingToLocal` に差し替える
- `timesync` は Wing が時刻を直してくれるので、大半が不要になる。何が残るか精査する
- 番人 3 (隠れた入力) を入れると、`localization.LoadGeometryFile` が `os.Open` を呼んでいて落ちる。
  読み込みを `geometry/` に移す

## 未決

- **言語**。Go のまま移せば約 19,500 行 (うち試験 7,200 行) がそのまま使える。
  C++ に書き直す理由があるとすれば Eigen / Ceres を使いたい場合。決まっていない
- **repo の名前**。Wing の文書が一貫して「Local」と呼んでいるので `ssl-RAVEN-Local` を候補とする
- **推定器を制御に通すこと**。今日までの実機 8 本は全て vision の生値で走っており、
  推定器は横で回していただけである。**まだ一度も閉ループで使っていない**
