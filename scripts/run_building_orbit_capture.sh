#!/usr/bin/env bash
# Plan horizontal orbit + teleport capture (no WAM visual hover-spin).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG="${1:-configs/missions/building_solo.yaml}"
cd "$ROOT"
# shellcheck source=/dev/null
source "$ROOT/scripts/env_sim_84.sh"

PY="${AERIAL_PY:-${PYTHON_BIN:-python3}}"
MISSION_ID=$("$PY" -c "import yaml; print(yaml.safe_load(open('$CONFIG'))['mission_id'])")
MISSION_DIR="$ROOT/artifacts/$MISSION_ID"

"$PY" -m aerial_inspect.cli plan "$CONFIG" -o "$MISSION_DIR"
PATTERN=$("$PY" -c "import yaml; print(yaml.safe_load(open('$CONFIG'))['survey']['pattern'])")
if [[ "$PATTERN" == "clearance_orbit" ]]; then
  echo "[orbit_capture] probing freeform path (radial+lateral) on AirSim..."
  "$PY" "$ROOT/scripts/probe_orbit_clearance.py" "$MISSION_DIR" --config "$CONFIG"
else
  "$PY" -m aerial_inspect.cli plan "$CONFIG" -o "$MISSION_DIR" --include-survey
fi
N=$("$PY" -c "import json; print(len(json.load(open('$MISSION_DIR/waypoints.json'))))")
echo "[orbit_capture] mission=$MISSION_ID waypoints=$N pattern=$PATTERN"

rm -rf "$MISSION_DIR/capture_survey"
"$PY" "$ROOT/scripts/export_sim_survey_frames.py" "$MISSION_DIR"
if [[ "${SKIP_VIDEO:-0}" != "1" ]]; then
  rm -rf "$MISSION_DIR/capture_survey/video_frames"
  "$PY" "$ROOT/scripts/export_sim_survey_video.py" "$MISSION_DIR"
else
  echo "[orbit_capture] SKIP_VIDEO=1 — frames only, proceeding to COLMAP"
fi
"$PY" "$ROOT/scripts/run_pycolmap_reconstruct.py" "$MISSION_DIR/capture_survey" \
  --workspace "$ROOT/artifacts/models/${MISSION_ID}" \
  --mode hybrid

if [[ "${SKIP_VALIDATE:-0}" != "1" ]]; then
  echo "[orbit_capture] validating path clearance..."
  "$PY" "$ROOT/scripts/validate_path_clearance.py" "$MISSION_DIR"
fi

echo "[orbit_capture] done -> $MISSION_DIR/capture_survey/"
