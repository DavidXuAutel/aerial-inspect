#!/usr/bin/env bash
# Plan + SURVEY only for a single building (centroid preset in YAML, skip wide SEARCH).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG="${1:-configs/missions/building_solo.yaml}"
cd "$ROOT"
# shellcheck source=/dev/null
source "$ROOT/scripts/env_sim_84.sh"

PY="${AERIAL_PY:-${PYTHON_BIN:-python3}}"
MISSION_ID=$("$PY" -c "import yaml; print(yaml.safe_load(open('$CONFIG'))['mission_id'])")
MISSION_DIR="$ROOT/artifacts/$MISSION_ID"

"$PY" -m aerial_inspect.cli plan "$CONFIG" -o "$MISSION_DIR" --include-survey
N=$("$PY" -c "import json; print(len(json.load(open('$MISSION_DIR/waypoints.json'))))")
echo "[building_solo] mission=$MISSION_ID waypoints=$N"
"$PY" -m aerial_inspect.cli sim-survey "$MISSION_DIR"
echo "[building_solo] done -> $MISSION_DIR/sim_runs/survey"
