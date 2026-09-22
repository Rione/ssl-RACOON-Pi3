#!/usr/bin/env bash
# Raspberry Pi 4 MIPI camera auto-selection (IMX219 / OV5647).
#
# When camera_auto_detect fails (vcgencmd reports detected=0), this script
# disables auto-detect and tries explicit dtoverlays one at a time:
#   imx219,cam0 -> imx219,cam1 -> ov5647,cam0 -> ov5647,cam1
# A state file prevents reboot loops (at most one full cycle).
#
# Intended as a oneshot systemd service ordered Before ssl-racoon.service.

set -uo pipefail

CONFIG_FILE="${PI_CONFIG_FILE:-/boot/firmware/config.txt}"
STATE_FILE="${CAMERA_STATE_FILE:-/var/lib/racoon-camera-autoselect.state}"

# Pi overlay names (config.txt dtoverlay= lines).
CAMERA_OVERLAYS=(
    "imx219,cam0"
    "imx219,cam1"
    "ov5647,cam0"
    "ov5647,cam1"
)

log() { echo "[camera-autoselect] $*"; }

camera_detected() {
    if command -v rpicam-hello >/dev/null 2>&1; then
        rpicam-hello --list-cameras 2>/dev/null | grep -qE '^[0-9]+ :'
        return $?
    fi
    python3 -c "from picamera2 import Picamera2; import sys; sys.exit(0 if Picamera2.global_camera_info() else 1)" 2>/dev/null
}

# Echoes active camera overlay keys (e.g. imx219,cam0) from config.txt.
present_camera_overlays() {
    local line key
    for line in "${CAMERA_OVERLAYS[@]}"; do
        if grep -qE "^dtoverlay=${line//,/\\,}(\s|$)" "${CONFIG_FILE}" 2>/dev/null \
            || grep -qE "^dtoverlay=${line}(\s|$)" "${CONFIG_FILE}" 2>/dev/null; then
            echo "${line}"
        fi
    done
}

remove_camera_overlays() {
    local overlay
    for overlay in "${CAMERA_OVERLAYS[@]}"; do
        sed -i "/^dtoverlay=${overlay//,/\\,}/d" "${CONFIG_FILE}" 2>/dev/null || true
        sed -i "/^dtoverlay=${overlay}/d" "${CONFIG_FILE}" 2>/dev/null || true
    done
}

set_camera_overlay() {
    local want="$1"
    remove_camera_overlays
    if grep -qE '^camera_auto_detect=' "${CONFIG_FILE}"; then
        sed -i 's/^camera_auto_detect=.*/camera_auto_detect=0/' "${CONFIG_FILE}"
    else
        echo "camera_auto_detect=0" >> "${CONFIG_FILE}"
    fi
    echo "dtoverlay=${want}" >> "${CONFIG_FILE}"
    log "Set camera_auto_detect=0 and dtoverlay=${want}"
}

next_overlay() {
    local tried="$1"
    local overlay
    for overlay in "${CAMERA_OVERLAYS[@]}"; do
        if [[ "${tried}" != *"${overlay}"* ]]; then
            echo "${overlay}"
            return 0
        fi
    done
    return 1
}

main() {
    if [[ ! -f "${CONFIG_FILE}" ]]; then
        log "${CONFIG_FILE} not found; not a Raspberry Pi boot. Skipping."
        return 0
    fi

    if camera_detected; then
        log "Camera detected; no change needed."
        rm -f "${STATE_FILE}" 2>/dev/null || true
        return 0
    fi

    log "No camera detected."

    local tried=""
    [[ -r "${STATE_FILE}" ]] && tried="$(tr '\n' ' ' < "${STATE_FILE}" 2>/dev/null || true)"

    local all_tried=1
    local overlay
    for overlay in "${CAMERA_OVERLAYS[@]}"; do
        if [[ "${tried}" != *"${overlay}"* ]]; then
            all_tried=0
            break
        fi
    done
    if (( all_tried )); then
        log "Already tried all camera overlays without success."
        log "Check the CSI cable orientation and connector (CAM0/CAM1). Giving up."
        return 1
    fi

    local chosen
    chosen="$(next_overlay "${tried}")" || {
        log "No remaining overlay to try."
        return 1
    }

    local present
    read -r -a present <<<"$(present_camera_overlays)"
    if (( ${#present[@]} > 0 )); then
        log "Current overlay(s) (${present[*]}) did not produce a camera; trying ${chosen}."
    else
        log "No camera overlay configured; trying ${chosen}."
    fi

    cp -a "${CONFIG_FILE}" "${CONFIG_FILE}.bak-camera-autoselect" 2>/dev/null || true
    set_camera_overlay "${chosen}"

    mkdir -p "$(dirname "${STATE_FILE}")"
    printf '%s %s\n' "${tried}" "${chosen}" > "${STATE_FILE}"

    log "Rebooting to apply dtoverlay=${chosen}."
    sync
    systemctl reboot 2>/dev/null || reboot
}

main "$@"
