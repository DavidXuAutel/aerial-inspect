#!/usr/bin/env bash
# AirSim simulation on 10.229.20.84 (or any host with WAM + AirSim).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG="${1:?usage: run_sim_pipeline.sh configs/missions/bridge_default.yaml}"
SURVEY_TRAJ="${2:-}"

cd "$ROOT"
if [[ -f "$ROOT/scripts/env_sim_84.sh" ]]; then
  # shellcheck source=/dev/null
  source "$ROOT/scripts/env_sim_84.sh"
else
  source .venv/bin/activate 2>/dev/null || true
fi

export AERIAL_WAM_ROOT="${AERIAL_WAM_ROOT:-$HOME/Projects/aerial-wam-v2}"
export AERIAL_VGOAL_ROOT="${AERIAL_VGOAL_ROOT:-$HOME/Projects/aerial-vgoal-wam}"
export SIM_DETECTOR="${SIM_DETECTOR:-open_vocab}"
export SIM_DEVICE="${SIM_DEVICE:-cuda}"

PY="${AERIAL_PY:-${PYTHON_BIN:-python3}}"
echo "WAM_ROOT=$AERIAL_WAM_ROOT"
echo "SIM_DETECTOR=$SIM_DETECTOR"
echo "PYTHON=$PY"

"$PY" -m aerial_inspect.cli sim-pipeline "$CONFIG"

MISSION_ID=$(python3 -c "import yaml; print(yaml.safe_load(open('$CONFIG'))['mission_id'])")
MISSION_DIR="$ROOT/artifacts/$MISSION_ID"

echo ""
echo "=== Acceptance checks ==="
cat "$MISSION_DIR/detected_centroid.json"
python3 -c "
import json
from pathlib import Path
d = Path('$MISSION_DIR/sim_runs/survey')
trajs = list(d.glob('wp_*/traj.jsonl'))
rows = sum(sum(1 for _ in open(t)) for t in trajs)
print(f'survey waypoints run: {len(trajs)}, total traj rows: {rows}')
"

if [[ -n "$SURVEY_TRAJ" && -f "$SURVEY_TRAJ" ]]; then
  python3 "$ROOT/scripts/run_offline_reconstruct.py" "$SURVEY_TRAJ"
fi

echo "Done: $MISSION_DIR"
