#!/usr/bin/env bash
# Push local aerial-inspect tree to 84 (public SSH).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "$ROOT/scripts/remote_84.env"

REMOTE="${AERIAL_RSYNC_REMOTE:-${AERIAL_SSH_HOST:-cursor-84-public}}"
REMOTE_DIR="${AERIAL_REMOTE_ROOT}"
RSYNC_SSH="${AERIAL_RSYNC_SSH:-ssh}"

rsync -az -e "${RSYNC_SSH}" --delete \
  --exclude '.git' \
  --exclude 'artifacts' \
  --exclude '.venv' \
  --exclude '__pycache__' \
  "$ROOT/" "${REMOTE}:${REMOTE_DIR}/"

echo "[sync_to_84] pushed -> ${REMOTE}:${REMOTE_DIR}"
