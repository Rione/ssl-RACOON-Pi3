# 時刻つき軌道追従の PoC

RAVEN から降りてくる予定の時刻つき軌道 (TimedPoint の列) を、ロボット上でどう追従すると良いかを
実機で比べるための PoC。RAVEN のプロトコルを作る前に、軌道を SSH の標準入力で直接流し込む。

```
開発 PC: traj_gen (仮想の TimedPoint[] を作る)
   │ JSON Lines を SSH の標準入力へ
   ▼
ロボット: racoon-pi3 -trajpoc -trajmethod <手法>
   ├ SSL-Vision を直接受信して自機の姿勢を得る
   ├ SPI の周期 (125 Hz) ごとに手法の速度を作り、速度指令を差し替える
   └ 終わったら誤差の指標を表示し、CSV に記録する
```

## 点の形式

中身は RAVEN の `TimedTrajectoryPoint` (x, y [mm], theta [rad], t [ns]) と同じ。ただし

- **t は先頭からの相対時刻 [ns]**。PC とロボットの時計は揃っていないので絶対時刻は送らない
- **座標は開始位置が原点・ロボットの前方が +x の相対**。走り出す瞬間の vision の姿勢に貼り付ける
- 最後に `END` の行。これが来てから走り出す (途中で EOF になったら走らない)

```
{"x":0.00,"y":0.00,"theta":0.00000,"t":0}
{"x":0.13,"y":0.00,"theta":0.00000,"t":16000000}
...
END
```

## 比べる手法

| `-trajmethod` | 中身 |
|---|---|
| `p` | 位置の P 制御だけ。参照速度を使わないので走行中は「速度 / Kp」だけ遅れ続ける。ベースライン |
| `ffp` | 参照速度を先回し (FF) し、位置の誤差を P で閉じる。Pi2 の `internal/control` と同じ形 |
| `ffp_lead` | `ffp` + 遅れ補償。vision の位置を直前の指令で「今」まで進め、`-trajlead` だけ先の参照を狙う |

`-trajinterp` で点の間の埋め方も選べる。`linear` は速度が区間ごとの階段 (RAVEN の OC が差分で
速度を作るのと同じ)、`hermite` は前後の点から速度を決めて滑らかに結ぶ。

### 机上シミュレーションでの見込み

`go test -v -run ClosedLoop ./internal/trajpoc/` が、仮の機体 (指令のむだ時間 40 ms・一次遅れ 60 ms・
vision の遅延 30 ms) で 8 の字 (0.45 m/s) を走らせた結果を出す。**機体モデルは仮の値なので、
実機で確かめる前の見込みにすぎない。**

| 手法 | 位置誤差 RMS | 輪郭誤差 RMS | 遅れ |
|---|---|---|---|
| p | 119 mm | 7 mm | 276 ms |
| ffp | 27 mm | 23 mm | 30 ms |
| ffp_lead (先読み 20〜40 ms) | 23 mm | 21 mm | ±11 ms |
| ffp_lead (先読み 80 ms) | 31 mm | 21 mm | −55 ms (先走り) |

- 先読みは大きすぎると先走る。実機では `-trajlead` を振って決める (既定 30 ms は仮の値)
- **輪郭誤差 (経路の形からのずれ) は先読みでは変わらない**。曲がりで内側へ切れ込むのは遅れではなく
  機体の応答の鈍さから来るので、別の手当てが要る

## 安全

- **STM には指令途絶のタイムアウトが無い**。Pi のプロセスが異常終了すると、ロボットは最後の速度で
  走り続ける。**機体の電源を切れる人がそばにいる状態で**回すこと
- PoC 側の止め方: Ctrl+C / SIGTERM / SIGHUP / 標準入力の EOF (SSH が切れた) / 軌道の終わり /
  vision が 250 ms 途切れたら 0 で待ち、1 s 途切れたら打ち切り / 開始位置から 枠 + 0.2 m 出たら打ち切り。
  どれでも 0 を 150 ms 送ってから非常停止のフレームへ戻して終了する
- SIGPIPE は無視する (Go は既定で、標準出力の相手が消えると即死する。即死すると上のとおり走り続ける)
- PoC の間は自己更新をしない (上書き後に os.Exit で即死するため)
- 走る前に軌道を検査する: 原点から始まること、枠 (`-trajfence`、既定 1.0 m) に収まること、
  速度が `-trajmaxspeed` (既定 0.5 m/s) を超えないこと。フラグの天井は 2.0 m/s・3.0 m
- 指令は速度・角速度 (既定 2.0 rad/s)・変化 (既定 2.0 m/s²) を制限する
- PoC の間は PC (RAVEN) からの指令を反映しない。キックもしない (コンデンサの充電も止める)
- Rock5A 専用 (Pi 4B のフレーム配置には対応していない)

Wi-Fi が切れた場合、SSH の切断がロボット側に伝わるまで時間がかかることがある。ただし PoC は
軌道の終わり・vision の途切れ・枠の外で自分で止まるので、通信が切れても走り去らない。

## 回し方

前提: 開発 PC の SSH 鍵をロボットに登録してあること (`ssh-copy-id root@<robot>`)。
ロボットの vision 上の ID (カバーの模様) が DIP スイッチの ID と違うときは `-trajvisionid` で指定する。

```bash
export ROBOT=172.15.0.34
scripts/trajpoc.sh deploy        # /root/trajpoc/racoon-pi3 に置く (Pi2 の /root には触れない。scp は使わない)

# 1 回走らせる。traj_gen の引数 -- racoon-pi3 の引数
scripts/trajpoc.sh run -shape line -size 0.4 -speed 0.3 -- -trajmethod ffp -trajvisionid 12

scripts/trajpoc.sh fetch         # CSV とログを trajpoc-results/ へ
scripts/trajpoc.sh restore       # 終わったら Pi2 のサービスを起動し直す
```

ロボット側の出力は `/root/trajpoc/trajpoc-<時刻>.log` にも残る。Ctrl+C で止めると止まった理由が
手元に届かないので、そのときはこちらを見る。

`run` は Pi2 のサービス (`ssl-racoon.service`) を止めてから走らせる (SPI を取り合わないため)。
終わったら `restore` で戻すこと。ロボットを再起動しても Pi2 に戻る。

### 最初の数回の進め方

1. `-shape line -size 0.3 -speed 0.2` の `ffp` で、動き出し・止まり方・安全停止 (途中で Ctrl+C) を確かめる
2. 同じ軌道で `p` / `ffp` / `ffp_lead` を比べる
3. `-shape square` / `circle` / `fig8` と速度を上げて、差が開くところを見る
4. `ffp_lead` の `-trajlead` を 0〜80 ms で振る

### 出てくる指標

| 指標 | 意味 |
|---|---|
| position error | 同じ時刻の参照位置との距離。時刻の遅れも含む |
| contour error | 時刻を無視した、経路 (折れ線) からの距離。形の崩れ |
| along-track mean / lag | 進行方向の誤差の平均と、それを時間に直した遅れ。負 (lag 正) は遅れている |
| cross-track | 進行方向に直交する誤差 |
| final (arrival) | 軌道の終わりから `Settle` (0.8 s) 後の誤差 |
| command accel RMS | 指令の変化の激しさ |

誤差は「vision が撮影した瞬間の参照」と比べる。vision の遅れそのものは追従の誤差に混ぜない。
