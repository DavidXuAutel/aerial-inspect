#!/usr/bin/env bash
# Mac: public SSH → re-run hybrid COLMAP on existing Humen 48 frames → pull preview.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "$ROOT/scripts/remote_84.env"

MISSION_ID=bridge_humen_001

HOST="${AERIAL_SSH_HOST:-cursor-84-public}"

echo "=== probe SSH ${HOST} ==="
ssh -o ConnectTimeout=25 "${HOST}" 'hostname'

echo "=== sync scripts ==="
bash "$ROOT/scripts/sync_to_84.sh"

echo "=== reconstruct on 84 (no re-capture) ==="
ssh "${HOST}" bash -s <<REMOTE
set -euo pipefail
cd ${AERIAL_REMOTE_ROOT}
source scripts/env_sim_84_humen.sh
\$AERIAL_PY -m pip install -e . -q
bash scripts/run_reconstruct_only.sh artifacts/${MISSION_ID}/capture_survey
REMOTE

echo "=== pull artifacts ==="
bash "$ROOT/scripts/pull_from_84.sh" "$MISSION_ID"

echo "Done. Open: $ROOT/artifacts/models/${MISSION_ID}/sparse_preview.png"
