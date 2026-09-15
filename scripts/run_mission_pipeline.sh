#!/usr/bin/env bash
# End-to-end: plan → SEARCH → replan survey → APPROACH → SURVEY → (optional) reconstruct.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG="${1:?usage: run_mission_pipeline.sh configs/missions/bridge_default.yaml [survey_capture_run_dir]}"
SURVEY_CAPTURE="${2:-}"

cd "$ROOT"
source .venv/bin/activate 2>/dev/null || true

export AERIAL_WAM_ROOT="${AERIAL_WAM_ROOT:-$HOME/Projects/aerial-wam-v2}"
export AERIAL_VGOAL_ROOT="${AERIAL_VGOAL_ROOT:-$HOME/Projects/aerial-vgoal-wam}"

MISSION_ID=$(python3 -c "import yaml; print(yaml.safe_load(open('$CONFIG'))['mission_id'])")
MISSION_DIR="$ROOT/artifacts/$MISSION_ID"

echo "=== plan (pre-search) ==="
aerial-inspect plan "$CONFIG" -o "$MISSION_DIR"
aerial-inspect export-wam-phases "$MISSION_DIR"

echo "=== SEARCH (semantic) ==="
bash "$ROOT/scripts/run_wam_search.sh" "$MISSION_DIR"

echo "=== replan survey (auto centroid from SEARCH traj) ==="
aerial-inspect replan-survey "$MISSION_DIR" --export-wam

echo "=== APPROACH ==="
bash "$ROOT/scripts/run_wam_approach.sh" "$MISSION_DIR"

echo "=== SURVEY ==="
bash "$ROOT/scripts/run_wam_survey.sh" "$MISSION_DIR"

if [[ -n "$SURVEY_CAPTURE" ]]; then
  echo "=== OFFLINE RECONSTRUCT ==="
  python3 "$ROOT/scripts/run_offline_reconstruct.py" "$SURVEY_CAPTURE"
else
  echo "Tip: pass survey capture run dir as 2nd arg to run COLMAP offline"
fi

echo "Done. Mission artifacts: $MISSION_DIR"
