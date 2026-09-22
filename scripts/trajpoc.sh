#!/usr/bin/env bash
# 時刻つき軌道追従の PoC を 1 回走らせる (開発 PC で実行)。詳しくは docs/traj-poc.md。
#
#   scripts/trajpoc.sh deploy                         # ロボットへ PoC 用のバイナリを置く
#   scripts/trajpoc.sh run   [traj_gen の引数] -- [racoon-pi3 の引数]
#   scripts/trajpoc.sh fetch                          # 記録 CSV を手元へ集める
#   scripts/trajpoc.sh restore                        # Pi2 のサービスを起動し直す
#
# 例: ROBOT=172.15.0.34 scripts/trajpoc.sh run -shape square -size 0.5 -- -trajmethod ffp -trajvisionid 12
#
# 止めるときは Ctrl+C。手元の ssh が終わるとロボット側の標準入力が閉じ、
# PoC は 0 を送ってから終了する。
set -euo pipefail

ROBOT="${ROBOT:?ROBOT=<robot ip> を指定してください}"
TEAM="${TEAM:-blue}"
VISION_ADDR="${VISION_ADDR:-224.5.23.2:10006}"
VISION_IFACE="${VISION_IFACE:-}"
REMOTE_DIR="${REMOTE_DIR:-/root/trajpoc}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SSH=(ssh -o ServerAliveInterval=1 -o ServerAliveCountMax=3 "root@${ROBOT}")

cmd="${1:-}"
shift || true

case "$cmd" in
deploy)
    (cd "$HERE" && GOOS=linux GOARCH=arm64 go build -tags rock5a -o /tmp/racoon-pi3-trajpoc ./cmd/racoon-pi3)
    "${SSH[@]}" "mkdir -p '$REMOTE_DIR'"
    scp -q /tmp/racoon-pi3-trajpoc "root@${ROBOT}:${REMOTE_DIR}/racoon-pi3"
    echo "deployed to ${ROBOT}:${REMOTE_DIR}/racoon-pi3 ($(cd "$HERE" && git rev-parse --short HEAD))"
    ;;
run)
    gen=() poc=()
    while [[ $# -gt 0 && "$1" != "--" ]]; do gen+=("$1"); shift; done
    [[ "${1:-}" == "--" ]] && shift
    poc=("$@")
    traj="$(mktemp)"
    (cd "$HERE" && go run ./cmd/traj_gen "${gen[@]}") > "$traj"
    iface=()
    [[ -n "$VISION_IFACE" ]] && iface=(-visioniface "$VISION_IFACE")
    # Pi2 と SPI を取り合わないよう止める (戻すのは restore)。
    "${SSH[@]}" "systemctl stop ssl-racoon.service"
    echo ">>> running on ${ROBOT}. Ctrl+C to stop."
    # 軌道を流したあとも標準入力を開けたままにする (閉じる = 止める合図)。
    # プロセス置換にしておくと、ロボット側が終われば sleep を待たずに戻る。
    "${SSH[@]}" "cd '$REMOTE_DIR' && ./racoon-pi3 -trajpoc -team '$TEAM' -visionaddr '$VISION_ADDR' ${iface[*]} ${poc[*]}" \
        < <(cat "$traj"; sleep 3600)
    rm -f "$traj"
    ;;
fetch)
    mkdir -p "$HERE/trajpoc-results"
    scp -q "root@${ROBOT}:${REMOTE_DIR}/trajpoc-*.csv" "$HERE/trajpoc-results/"
    ls -1t "$HERE/trajpoc-results" | head
    ;;
restore)
    "${SSH[@]}" "systemctl start ssl-racoon.service && systemctl is-active ssl-racoon.service"
    ;;
*)
    sed -n '2,12p' "$0"
    exit 2
    ;;
esac
