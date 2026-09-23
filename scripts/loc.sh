#!/usr/bin/env bash
# この PC (macOS) から SSH で Rock5A の機体を操作し、自己位置推定を回す道具。
#
# 走行機能には触らない。推定は観測に徹し、結果は
#   - ロボットのログ (1 秒ごとの [EST] 行)
#   - MCAP (/est/state, /est/innovation, /est/timing)
#   - HTTP  http://<robot>:9191/localization
# に出る。解析は `go run ./cmd/loc_replay` で行う。
#
# 使い方:
#   scripts/loc.sh find [--sweep]       MAC からロボットの IP を探す
#   scripts/loc.sh check                SSH・サービス・ディスク・電池を見る
#   scripts/loc.sh deploy               ビルドして送り込む
#   scripts/loc.sh run                  推定を走らせる (Ctrl+C で止めて元に戻す)
#   scripts/loc.sh status               HTTP で今の推定を見る
#   scripts/loc.sh fetch                MCAP を手元へ取り出す
#   scripts/loc.sh restore              元のサービスに戻す
#   scripts/loc.sh clean                送り込んだものを消す
#
# 環境変数:
#   ROBOT          ロボットの IP。未指定なら find の結果を使う
#   ROBOT_MAC      find が探す MAC (既定 20:bd:1d:d7:3f:f2 = racoon-56011)
#   SSH_KEY        秘密鍵 (既定 ~/.ssh/id_ed25519)
#   TEAM           自機のチーム色 (既定 blue)
#   VISION_IFACE   ロボット側で vision を受ける NIC (既定 wlan0)
#   VISION_DELAY_MS vision の定数遅延の補償 [ms] (既定 0。loc_replay で測る)
#   GEOMETRY       機体パラメータ JSON (省略すると組み込みの有効値)
#   REMOTE_DIR     ロボット上の置き場所 (既定 /root/loc)
set -euo pipefail

ROBOT_MAC="${ROBOT_MAC:-20:bd:1d:d7:3f:f2}"
SSH_KEY="${SSH_KEY:-$HOME/.ssh/id_ed25519}"
TEAM="${TEAM:-blue}"
VISION_IFACE="${VISION_IFACE:-wlan0}"
VISION_DELAY_MS="${VISION_DELAY_MS:-0}"
GEOMETRY="${GEOMETRY:-}"
REMOTE_DIR="${REMOTE_DIR:-/root/loc}"
REMOTE_BIN="$REMOTE_DIR/racoon-pi3-loc"
REMOTE_LOGS="$REMOTE_DIR/logs"
SERVICE="ssl-racoon.service"
LOCAL_OUT="${LOCAL_OUT:-logs}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

die() { echo "loc.sh: $*" >&2; exit 1; }
note() { echo "--- $*"; }

# ssh の共通オプション。**BatchMode でパスワードを待たない** (固まるので)。
ssh_opts=(-i "$SSH_KEY" -o BatchMode=yes -o StrictHostKeyChecking=accept-new
          -o ConnectTimeout=5 -o ServerAliveInterval=5 -o ServerAliveCountMax=3)

rsh() { ssh "${ssh_opts[@]}" "root@${ROBOT}" "$@"; }

need_robot() {
  if [[ -z "${ROBOT:-}" ]]; then
    ROBOT="$(cmd_find --quiet)" || die "ロボットが見つからない。\`scripts/loc.sh find\` の案内を見るか、ROBOT=<ip> を指定する"
    echo "--- robot: $ROBOT (found by MAC $ROBOT_MAC)" >&2
  fi
  [[ -f "$SSH_KEY" ]] || die "SSH 鍵がない: $SSH_KEY"
}

# ---------------------------------------------------------------- find
# DHCP で IP が変わるので MAC から引く。
# macOS には ip(8) が無いので arp(8) を使う。ARP キャッシュに載っていなければ
# ブロードキャストを 1 回投げて載せる。
cmd_find() {
  local quiet=0 sweep=0
  for a in "$@"; do
    [[ "$a" == "--quiet" ]] && quiet=1
    [[ "$a" == "--sweep" ]] && sweep=1
  done
  local mac_re
  # arp は先頭ゼロを落とすので (0b -> b)、正規表現側も落として比べる。
  mac_re="$(echo "$ROBOT_MAC" | tr 'A-Z' 'a-z' | sed -E 's/(^|:)0([0-9a-f])/\1\2/g')"

  lookup() {
    arp -an 2>/dev/null | tr 'A-Z' 'a-z' \
      | awk -v m="$mac_re" '$0 ~ m {print $2}' | tr -d '()' | head -1
  }

  local ip; ip="$(lookup || true)"
  if [[ -z "$ip" && $sweep -eq 1 ]]; then
    # **サブネットを一巡する。会場の共用ネットワークでは周りに迷惑なので、
    # 明示的に --sweep を付けたときだけ行う。**
    local myip base
    myip="$(ipconfig getifaddr "$(route -n get default 2>/dev/null | awk '/interface:/{print $2}')" 2>/dev/null || true)"
    [[ -n "$myip" ]] || { [[ $quiet -eq 1 ]] || echo "loc.sh: 自分の IP が分からないので sweep できない" >&2; return 1; }
    base="${myip%.*}"
    [[ $quiet -eq 1 ]] || note "sweep ${base}.0/24 (数秒)"
    for i in $(seq 1 254); do (ping -c 1 -W 200 "${base}.${i}" >/dev/null 2>&1 &) ; done
    sleep 3
    ip="$(lookup || true)"
  fi
  if [[ -z "$ip" ]]; then
    [[ $quiet -eq 1 ]] || cat >&2 <<'MSG'
loc.sh: MAC からロボットが見つからない。
  - 同じ LAN にいるか (会場の AP に繋がっているか)
  - 一度 `ping <ロボットの IP>` して ARP キャッシュに載せる
  - サブネットを一巡してよければ `scripts/loc.sh find --sweep`
  - IP が分かっているなら `ROBOT=<ip> scripts/loc.sh ...`
MSG
    return 1
  fi
  echo "$ip"
}

# ---------------------------------------------------------------- check
cmd_check() {
  need_robot
  note "ssh"
  rsh 'echo "  host      $(hostname)"; echo "  uptime   $(uptime | sed "s/^ *//")"'
  note "service"
  rsh "systemctl is-active $SERVICE || true; systemctl is-enabled $SERVICE || true"
  note "disk (MCAP は 10 分で数十 MB 出る)"
  rsh 'df -h /root | tail -1'
  note "running racoon processes"
  # **pgrep -f / pkill -f は自分のコマンド行にも一致する** (PoC で自分のシェルを 2 回殺した)。
  # comm を見て選ぶ。
  rsh 'ps -eo pid,comm,args --no-headers | awk "\$2 ~ /racoon|loc/ {print \"  \" \$0}" || true'
  note "http"
  curl -s --max-time 3 "http://${ROBOT}:9191/status" | head -20 || echo "  (no answer on 9191)"
}

# ---------------------------------------------------------------- build
cmd_build() {
  note "build racoon-pi3 (linux/arm64, tag rock5a)"
  # **バージョンを (devel) に固定して自己更新を確実に無効化する。**
  # -noupdate も渡すが、二重の歯止めにしておく (PoC では走行中に
  # 上書きされて os.Exit した事故が起きている)。
  GOOS=linux GOARCH=arm64 go build -tags rock5a \
    -ldflags "-X github.com/Rione/ssl-RACOON-Pi3/internal/upgrade.Version=(devel)" \
    -o build/racoon-pi3-loc ./cmd/racoon-pi3
  ls -lh build/racoon-pi3-loc
}

# ---------------------------------------------------------------- deploy
cmd_deploy() {
  need_robot
  cmd_build
  note "send to ${ROBOT}:${REMOTE_BIN}"
  # **この機体には sftp-server が無いので scp は使えない** (PoC の実測)。
  # ssh の標準入力で流し込む。
  rsh "mkdir -p '$REMOTE_DIR' '$REMOTE_LOGS' && cat > '$REMOTE_BIN.new'" < build/racoon-pi3-loc
  rsh "chmod +x '$REMOTE_BIN.new' && mv '$REMOTE_BIN.new' '$REMOTE_BIN' && ls -l '$REMOTE_BIN'"
  if [[ -n "$GEOMETRY" ]]; then
    [[ -f "$GEOMETRY" ]] || die "幾何ファイルがない: $GEOMETRY"
    note "send geometry $GEOMETRY"
    rsh "cat > '$REMOTE_DIR/geometry.json'" < "$GEOMETRY"
  fi
}

# ---------------------------------------------------------------- run
cmd_run() {
  need_robot
  local extra=("$@")

  # -locident はロボットが自走する。**STM に指令途絶のタイムアウトが無い**ので、
  # Pi のプロセスが死ぬと最後の速度で走り続ける。人がそばにいるときだけ。
  for a in "${extra[@]:-}"; do
    if [[ "$a" == "-locident" ]]; then
      echo "*** -locident: ロボットが自走します。周囲を空けて、非常停止を用意してください。" >&2
      echo "*** 続けるなら 5 秒以内に Ctrl+C しないでください。" >&2
      sleep 5
    fi
  done

  note "stop $SERVICE (元に戻すのは restore、または Ctrl+C で自動)"
  rsh "systemctl stop $SERVICE" || true
  trap 'echo; note "restoring $SERVICE"; ssh "${ssh_opts[@]}" "root@${ROBOT}" "systemctl start '"$SERVICE"'" || true' EXIT

  local geo=()
  [[ -n "$GEOMETRY" ]] && geo=(-locgeometry "$REMOTE_DIR/geometry.json")

  note "run (Ctrl+C to stop)"
  # **標準入力を握ったまま起動する。** ssh を切ると EOF が届き、ロボット側の
  # プロセスも終わる。ここを外すと、手元を切ってもロボットが走り続ける。
  # -t を付けないのは、TTY があると Ctrl+C の扱いが分かれるため。
  rsh "cd '$REMOTE_DIR' && exec '$REMOTE_BIN' \
        -noupdate \
        -locestimate \
        -loclog '$REMOTE_LOGS' \
        -team '$TEAM' \
        -visioniface '$VISION_IFACE' \
        -locvisiondelay '$VISION_DELAY_MS' \
        ${geo[*]:-} ${extra[*]:-} 2>&1" < /dev/null
}

# ---------------------------------------------------------------- status
cmd_status() {
  need_robot
  local url="http://${ROBOT}:9191/localization"
  if command -v jq >/dev/null 2>&1; then
    curl -s --max-time 3 "$url" | jq .
  else
    curl -s --max-time 3 "$url"
  fi
}

# ---------------------------------------------------------------- fetch
cmd_fetch() {
  need_robot
  mkdir -p "$LOCAL_OUT"
  local files
  files="$(rsh "ls -1 '$REMOTE_LOGS'/*.mcap 2>/dev/null || true")"
  [[ -n "$files" ]] || { note "no MCAP on the robot"; return 0; }
  while IFS= read -r f; do
    [[ -n "$f" ]] || continue
    local base; base="$(basename "$f")"
    note "fetch $base"
    # scp が使えないので cat で吸い出す。
    rsh "cat '$f'" > "$LOCAL_OUT/$base"
    ls -lh "$LOCAL_OUT/$base"
  done <<< "$files"
  note "analyse with:  go run ./cmd/loc_replay -log $LOCAL_OUT/<file>.mcap -compare"
}

# ---------------------------------------------------------------- misc
cmd_restore() {
  need_robot
  note "start $SERVICE"
  rsh "systemctl start $SERVICE; sleep 1; systemctl is-active $SERVICE"
}

cmd_logs() {
  need_robot
  rsh "journalctl -u $SERVICE -n ${1:-100} --no-pager"
}

cmd_clean() {
  need_robot
  note "remove $REMOTE_DIR"
  rsh "rm -rf '$REMOTE_DIR'"
}

cmd_help() { sed -n '2,32p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

case "${1:-help}" in
  find)    shift; cmd_find "$@" ;;
  check)   shift; cmd_check "$@" ;;
  build)   shift; cmd_build "$@" ;;
  deploy)  shift; cmd_deploy "$@" ;;
  run)     shift; cmd_run "$@" ;;
  status)  shift; cmd_status "$@" ;;
  fetch)   shift; cmd_fetch "$@" ;;
  restore) shift; cmd_restore "$@" ;;
  logs)    shift; cmd_logs "$@" ;;
  clean)   shift; cmd_clean "$@" ;;
  help|-h|--help) cmd_help ;;
  *) die "unknown command: $1 (try: help)" ;;
esac
