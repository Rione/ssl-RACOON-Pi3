#!/usr/bin/env bash
# Install YOLO calibration weights (last.pt) from a GitHub Release.
#
# Board tarballs no longer bundle .pt files to keep downloads small.
# Run once per robot (or after a model update), from the directory that
# contains the racoon binary and camera/ package:
#
#   ./scripts/install-yolo-model.sh              # latest release
#   ./scripts/install-yolo-model.sh v1.2.3       # specific tag

set -euo pipefail

REPO="${RACOON_GITHUB_REPO:-Rione/ssl-RACOON-Pi2}"
TAG="${1:-}"
DEST_DIR="${RACOON_MODEL_DIR:-camera/yolo}"

log() { echo "[install-yolo-model] $*"; }

resolve_tag() {
    if [[ -n "${TAG}" ]]; then
        echo "${TAG}"
        return
    fi
    curl -fsSL "https://api.github.com/repos/${REPO}/releases/latest" \
        | sed -n 's/.*"tag_name": *"\([^"]*\)".*/\1/p' \
        | head -1
}

main() {
    local tag version asset url
    tag="$(resolve_tag)"
    if [[ -z "${tag}" ]]; then
        log "Could not resolve release tag for ${REPO}"
        exit 1
    fi

    version="${tag#v}"
    asset="racoon-pi2-yolo_${version}_last.pt"
    mkdir -p "${DEST_DIR}"
    url="https://github.com/${REPO}/releases/download/${tag}/${asset}"

    log "Downloading ${url}"
    log "-> ${DEST_DIR}/last.pt"
    curl -fL "${url}" -o "${DEST_DIR}/last.pt"
    log "Done."
}

main "$@"
