#!/usr/bin/env bash
# Mac: sync → full Humen sim-pipeline on 84 → pull artifacts.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG="${1:-configs/missions/bridge_humen.yaml}"
# shellcheck source=/dev/null
source "$ROOT/scripts/remote_84.env"

MISSION_ID=$(python3 -c "import yaml; print(yaml.safe_load(open('$ROOT/$CONFIG'))['mission_id'])")
HOST="${AERIAL_SSH_HOST:-cursor-84-public}"

echo "=== sync_to_84 ==="
bash "$ROOT/scripts/sync_to_84.sh"

echo "=== Humen sim-pipeline on 84 (${HOST}) ==="
ssh "${HOST}" bash -s <<REMOTE
set -euo pipefail
cd ${AERIAL_REMOTE_ROOT}
source scripts/env_sim_84_humen.sh
\$AERIAL_PY -m pip install -e . -q
export FORCE_SEARCH=\${FORCE_SEARCH:-1}
export RESET_MISSION=\${RESET_MISSION:-1}
bash scripts/run_humen_sim_pipeline.sh $CONFIG
REMOTE

echo "=== pull_from_84 ==="
bash "$ROOT/scripts/pull_from_84.sh" "$MISSION_ID"

echo "Done."
echo "  video: $ROOT/artifacts/$MISSION_ID/capture_survey/survey_flythrough.mp4"
echo "  preview: $ROOT/artifacts/models/$MISSION_ID/sparse_preview.png"
