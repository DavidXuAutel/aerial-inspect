#!/usr/bin/env bash
# Export survey frames, fly-through video, and COLMAP model for a mission.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MISSION_DIR="${1:-$ROOT/artifacts/building_solo_001}"
cd "$ROOT"
# shellcheck source=/dev/null
source "$ROOT/scripts/env_sim_84.sh"

PY="${AERIAL_PY:-${PYTHON_BIN:-python3}}"
MISSION_ID="$(basename "$MISSION_DIR")"

echo "[deliverables] mission=$MISSION_ID"
"$PY" "$ROOT/scripts/export_sim_survey_frames.py" "$MISSION_DIR"
"$PY" "$ROOT/scripts/export_sim_survey_video.py" "$MISSION_DIR"
"$PY" "$ROOT/scripts/run_pycolmap_reconstruct.py" "$MISSION_DIR/capture_survey" \
  --workspace "$ROOT/artifacts/models/${MISSION_ID}"

echo "[deliverables] done:"
echo "  frames: $MISSION_DIR/capture_survey/frames/"
echo "  video:  $MISSION_DIR/capture_survey/survey_flythrough.mp4"
echo "  model:  $ROOT/artifacts/models/${MISSION_ID}/"
