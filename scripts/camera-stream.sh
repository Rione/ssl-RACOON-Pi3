#!/usr/bin/env bash
# Low-latency MJPEG HTTP stream from Rock 5A MIPI camera.
#
# View in VLC: Media -> Open Network Stream
#   http://<host-ip>:8080/stream.mjpg
#
# Stop ssl-racoon.service before starting (camera is exclusive).

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEVICE="${CAMERA_DEVICE:-11}"
WIDTH="${CAMERA_WIDTH:-640}"
HEIGHT="${CAMERA_HEIGHT:-480}"
FPS="${CAMERA_FPS:-15}"
PORT="${STREAM_PORT:-8080}"
QUALITY="${STREAM_QUALITY:-65}"
EXPOSURE="${CAMERA_EXPOSURE:-900}"
GAIN="${CAMERA_GAIN:-80}"
GAMMA="${STREAM_GAMMA:-1.1}"
BRIGHTNESS="${STREAM_BRIGHTNESS:-0}"
SENSOR_SUBDEV="${CAMERA_SENSOR_SUBDEV:-/dev/v4l-subdev2}"
AUTO_EXPOSURE="${CAMERA_AUTO_EXPOSURE:-0}"

stop_stream() {
    local pids self=$$
    pids=$(
        pgrep -f "${SCRIPT_DIR}/camera-stream.py" || true
        pgrep -f "v4l2-ctl -d /dev/video${DEVICE}" || true
        pgrep -f "ffmpeg.*${PORT}/stream.mjpg" || true
    )
    for pid in ${pids}; do
        [[ "${pid}" -eq "${self}" ]] && continue
        kill "${pid}" 2>/dev/null || true
    done
    sleep 1
    for pid in ${pids}; do
        [[ "${pid}" -eq "${self}" ]] && continue
        kill -9 "${pid}" 2>/dev/null || true
    done
}

start_stream() {
    stop_stream

    echo "Starting low-latency camera stream on http://0.0.0.0:${PORT}/stream.mjpg"
    args=(
        --device "${DEVICE}"
        --width "${WIDTH}"
        --height "${HEIGHT}"
        --fps "${FPS}"
        --port "${PORT}"
        --quality "${QUALITY}"
        --sensor-subdev "${SENSOR_SUBDEV}"
        --exposure "${EXPOSURE}"
        --gain "${GAIN}"
        --gamma "${GAMMA}"
        --brightness "${BRIGHTNESS}"
    )
    if [[ "${AUTO_EXPOSURE}" == "1" ]]; then
        args+=(--auto-exposure)
    fi

    exec python3 "${SCRIPT_DIR}/camera-stream.py" "${args[@]}"
}

case "${1:-start}" in
    start)
        start_stream
        ;;
    stop)
        stop_stream
        echo "Stream stopped."
        ;;
    restart)
        stop_stream
        start_stream
        ;;
    *)
        echo "Usage: $0 {start|stop|restart}" >&2
        exit 1
        ;;
esac
