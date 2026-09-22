#!/usr/bin/env bash
# Rock 5A MIPI camera auto-selection.
#
# The Raspberry Pi Camera v1.3 (OV5647) and v2 (IMX219) use different device
# tree overlays on the Rock 5A, and only ONE may be active at a time: enabling
# both breaks the CSI pipeline (rkcif "get remote terminal sensor failed").
#
# This script checks whether a usable sensor came up after boot. If not, it
# rewrites /boot/dietpiEnv.txt so that exactly one camera overlay is active and
# reboots once. A state file records which overlays have been tried so the box
# never reboot-loops: at most one full IMX219<->OV5647 cycle is attempted.
#
# Intended to run as a oneshot systemd service ordered Before ssl-racoon.service.

set -uo pipefail

ENV_FILE="${DIETPI_ENV_FILE:-/boot/dietpiEnv.txt}"
STATE_FILE="${CAMERA_STATE_FILE:-/var/lib/racoon-camera-autoselect.state}"

IMX219_OVERLAY="rpi-camera-v2"
OV5647_OVERLAY="rpi-camera-v1_3"
CAMERA_OVERLAYS=("${IMX219_OVERLAY}" "${OV5647_OVERLAY}")

log() { echo "[camera-autoselect] $*"; }

# A bound sensor exposes a v4l-subdev whose name contains imx219/ov5647.
sensor_present() {
    local f
    for f in /sys/class/video4linux/v4l-subdev*/name; do
        [[ -r "$f" ]] || continue
        if grep -qiE 'imx219|ov5647' "$f"; then
            return 0
        fi
    done
    return 1
}

# Echoes the camera overlays currently listed in overlays= (space separated).
present_camera_overlays() {
    local line token out=()
    line="$(grep -E '^overlays=' "${ENV_FILE}" 2>/dev/null || true)"
    for token in ${line#overlays=}; do
        case " ${CAMERA_OVERLAYS[*]} " in
            *" ${token} "*) out+=("${token}") ;;
        esac
    done
    echo "${out[*]}"
}

other_overlay() {
    if [[ "$1" == "${IMX219_OVERLAY}" ]]; then
        echo "${OV5647_OVERLAY}"
    else
        echo "${IMX219_OVERLAY}"
    fi
}

# Rewrites overlays= so it contains exactly the given camera overlay, dropping
# any other camera overlay while preserving all non-camera overlays.
set_camera_overlay() {
    local want="$1"
    local line token kept=()
    line="$(grep -E '^overlays=' "${ENV_FILE}" 2>/dev/null || true)"
    for token in ${line#overlays=}; do
        case " ${CAMERA_OVERLAYS[*]} " in
            *" ${token} "*) ;;            # drop any camera overlay
            *) kept+=("${token}") ;;
        esac
    done
    kept+=("${want}")
    local newline="overlays=${kept[*]}"

    cp -a "${ENV_FILE}" "${ENV_FILE}.bak-camera-autoselect" 2>/dev/null || true
    if grep -qE '^overlays=' "${ENV_FILE}"; then
        sed -i "s|^overlays=.*|${newline}|" "${ENV_FILE}"
    else
        echo "${newline}" >> "${ENV_FILE}"
    fi
    log "Set overlay line: ${newline}"
}

main() {
    if [[ ! -f "${ENV_FILE}" ]]; then
        log "${ENV_FILE} not found; not a DietPi/Rock 5A boot. Skipping."
        return 0
    fi

    if sensor_present; then
        log "Camera sensor detected; no change needed."
        rm -f "${STATE_FILE}" 2>/dev/null || true
        return 0
    fi

    log "No camera sensor detected."

    local tried=""
    [[ -r "${STATE_FILE}" ]] && tried="$(cat "${STATE_FILE}" 2>/dev/null || true)"

    # Stop once both overlays have been tried to avoid reboot loops.
    if [[ "${tried}" == *"${IMX219_OVERLAY}"* && "${tried}" == *"${OV5647_OVERLAY}"* ]]; then
        log "Already tried both camera overlays without success."
        log "Check the camera cable/module orientation. Giving up."
        return 1
    fi

    local present chosen
    read -r -a present <<<"$(present_camera_overlays)"

    if (( ${#present[@]} > 1 )); then
        # Misconfiguration (both overlays active): dedupe to the first one.
        chosen="${present[0]}"
        log "Multiple camera overlays active (${present[*]}); reducing to ${chosen}."
    elif (( ${#present[@]} == 1 )); then
        chosen="$(other_overlay "${present[0]}")"
        log "Current overlay ${present[0]} produced no sensor; trying ${chosen}."
    else
        chosen="${IMX219_OVERLAY}"
        log "No camera overlay configured; trying ${chosen}."
    fi

    set_camera_overlay "${chosen}"

    mkdir -p "$(dirname "${STATE_FILE}")"
    printf '%s %s\n' "${tried}" "${chosen}" > "${STATE_FILE}"

    log "Rebooting to apply camera overlay ${chosen}."
    sync
    systemctl reboot 2>/dev/null || reboot
}

main "$@"
