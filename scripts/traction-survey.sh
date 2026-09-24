#!/usr/bin/env bash
# コートの traction を測る (開発 PC で実行)。会場に着いたら、使う場所ごとに 1 回ずつ回す。
#
#   ROBOT=172.15.0.26 scripts/traction-survey.sh 自陣ゴール前
#   ROBOT=172.15.0.26 VENUE=立命館 scripts/traction-survey.sh センター 5   # 会場名と本数
#
# ロボットをその場所に置いて呼ぶと、短い直線を数本走らせて、
# 「車輪が言う移動量に対して実際に進んだ割合」を出す。**ロボットが走る。**
#
# 結果は trajpoc-dataset/traction-survey.csv に貯まる (git で追う)。場所を変えて繰り返せば、
# コートのどこが滑るかの地図になり、会場をまたいで比べられる。
# 走行の生の記録は trajpoc-results/ (git 管理外)。
#
# 読み方 (実測、2026-09-24):
#   0.95 前後  正常
#   0.9 未満   滑っている。ここで取った追従の数字は信用できない
#   0.0 のまま vision が別の物を見ている (traction の問題ではない)
set -uo pipefail

ROBOT="${ROBOT:?ROBOT=<robot ip> を指定してください}"
LABEL="${1:?場所の名前を渡してください (例: 自陣ゴール前)}"
RUNS="${2:-3}"
VISION_ID="${VISION_ID:-15}"
TEAM="${TEAM:-blue}"
VISION_IFACE="${VISION_IFACE:-wlan0}"
PC_IFACE="${PC_IFACE:-wlp113s0f0}"
SIZE="${SIZE:-0.4}"   # 走る距離 [m]
SPEED="${SPEED:-0.3}" # 巡航速度 [m/s]

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$HERE/trajpoc-results"
SURVEY="$HERE/trajpoc-dataset/traction-survey.csv"
mkdir -p "$OUT"
SSH=(ssh -o StrictHostKeyChecking=no -o ConnectTimeout=6 "root@${ROBOT}")

# いまの位置を vision から読む (どこで測ったかを残すため)。
pos="$(cd "$HERE" && timeout 20 go run ./cmd/vision_list -iface "$PC_IFACE" 2>/dev/null \
    | awk -v t="$TEAM" -v id="$VISION_ID" '$1==t && $2==id {gsub(/[(),]/,""); print $3" "$4; exit}')"
if [ -z "$pos" ]; then
    echo "NG  vision に $TEAM $VISION_ID が見えない。-trajvisionid / 模様 / カメラを確かめること" >&2
    exit 1
fi
x="${pos% *}"; y="${pos#* }"
echo "場所 \"$LABEL\" = ($x, $y) mm で $RUNS 本走らせます"

files=()
for i in $(seq 1 "$RUNS"); do
    printf "  %d/%d ... " "$i" "$RUNS"
    line="$(ROBOT="$ROBOT" TEAM="$TEAM" VISION_IFACE="$VISION_IFACE" \
        timeout 120 "$HERE/scripts/trajpoc.sh" run -shape line -size "$SIZE" -speed "$SPEED" \
        -- -trajmethod ffp_vlead -trajlead 90 -trajvisionid "$VISION_ID" 2>&1 \
        | grep -E "^(end|csv)" | tr '\n' ' ')"
    echo "$line"
    f="$(echo "$line" | grep -o 'trajpoc-[0-9-]*-ffp_vlead\.csv' | head -1)"
    [ -n "$f" ] && files+=("$f")
done

if [ ${#files[@]} -eq 0 ]; then
    echo "NG  記録が 1 本も取れなかった" >&2
    exit 1
fi

# 走った分だけ取ってくる (全部 tar すると溜まったぶん重い)。
for f in "${files[@]}"; do
    "${SSH[@]}" "cat /root/trajpoc/$f" > "$OUT/$f" 2>/dev/null
done

echo
python3 "$HERE/trajpoc-dataset/scripts/trajpoc-traction.py" "${files[@]/#/$OUT/}"

# 中央値の中央値を、その場所の代表値として貯める。
med="$(python3 - "${files[@]/#/$OUT/}" <<'PY'
import sys, statistics, importlib.util, pathlib
spec = importlib.util.spec_from_file_location(
    "tr", pathlib.Path(__file__).parent / "trajpoc-dataset/scripts/trajpoc-traction.py")
here = pathlib.Path(sys.argv[0]).parent
spec = importlib.util.spec_from_file_location("tr", here / "trajpoc-dataset/scripts/trajpoc-traction.py")
tr = importlib.util.module_from_spec(spec); spec.loader.exec_module(tr)
meds = []
for p in sys.argv[1:]:
    rs, _ = tr.ratios(p)
    if rs:
        meds.append(statistics.median(rs))
print(f"{statistics.median(meds):.3f}" if meds else "")
PY
)"
if [ -n "$med" ]; then
    [ -f "$SURVEY" ] || echo "date,venue,label,x_mm,y_mm,runs,median_traction" > "$SURVEY"
    echo "$(date +%Y-%m-%dT%H:%M),${VENUE:-?},$LABEL,$x,$y,${#files[@]},$med" >> "$SURVEY"
    echo
    echo "=== \"$LABEL\" の代表値: $med ==="
    awk -F, 'NR>1{printf "  %-10s %-20s (%6s,%6s)  %s\n", $2, $3, $4, $5, $7}' "$SURVEY"
fi
