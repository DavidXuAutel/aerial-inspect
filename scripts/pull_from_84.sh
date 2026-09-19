#!/usr/bin/env bash
# Pull mission artifacts from 84 to local Mac (public SSH).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "$ROOT/scripts/remote_84.env"

MISSION_ID="${1:?usage: pull_from_84.sh <mission_id>}"
REMOTE="${AERIAL_RSYNC_REMOTE:-${AERIAL_SSH_HOST:-cursor-84-public}}"
REMOTE_DIR="${AERIAL_REMOTE_ROOT}"
RSYNC_SSH="${AERIAL_RSYNC_SSH:-ssh}"

mkdir -p "$ROOT/artifacts/$MISSION_ID" "$ROOT/artifacts/models/$MISSION_ID"

rsync -az -e "${RSYNC_SSH}" "${REMOTE}:${REMOTE_DIR}/artifacts/${MISSION_ID}/" \
  "$ROOT/artifacts/${MISSION_ID}/"

rsync -az -e "${RSYNC_SSH}" "${REMOTE}:${REMOTE_DIR}/artifacts/models/${MISSION_ID}/" \
  "$ROOT/artifacts/models/${MISSION_ID}/" 2>/dev/null || true

echo "[pull_from_84] artifacts/$MISSION_ID + models/$MISSION_ID"
