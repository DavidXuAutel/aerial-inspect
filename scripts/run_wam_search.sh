#!/usr/bin/env bash
# SEARCH phase: visual-prompt area scan via wam_vgoal_deploy.
set -euo pipefail
MISSION_DIR="${1:?usage: run_wam_search.sh artifacts/mission_xxx}"
WAM_ROOT="${AERIAL_WAM_ROOT:-$HOME/Projects/aerial-wam-v2}"
VGOAL_ROOT="${AERIAL_VGOAL_ROOT:-$HOME/Projects/aerial-vgoal-wam}"
PHASE_JSON="$MISSION_DIR/wam_phases.json"
MOCK="${MOCK_CAMERA:-1}"
MAVLINK_PORT="${MAVLINK_PORT:-/dev/ttyACM0}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

if [[ ! -f "$PHASE_JSON" ]]; then
  echo "missing $PHASE_JSON — run: aerial-inspect plan ... && aerial-inspect export-wam-phases $MISSION_DIR"
  exit 1
fi

export PYTHONPATH="$WAM_ROOT"
python3 <<PY
import json, subprocess, sys, time
from pathlib import Path

wam = Path("$WAM_ROOT")
data = json.loads(Path("$PHASE_JSON").read_text())
visual = data.get("visual_prompt", "bridge")
search = data.get("search", {})
cmd = [
    sys.executable, "-m", "experiments.aerial.scripts.wam_vgoal_deploy",
    "--vgoal-repo", "$VGOAL_ROOT",
    "--mavlink-port", "$MAVLINK_PORT",
    "--visual-prompt", visual,
    "--max-steps", str(int(search.get("max_steps", 400))),
]
if search.get("record_auto", True):
    cmd.append("--record-auto")
center = search.get("center_xy") or [0.0, 0.0]
alt = float(search.get("altitude_m", 40.0))
cmd.extend([
    "--goal-x", str(center[0]),
    "--goal-y", str(center[1]),
    "--goal-z", str(alt),
    "--corpus-leg", "search",
])
if "$MOCK" == "1":
    cmd.append("--mock-camera")
else:
    cmd.extend(["--offboard", "--run", "--i-know-props-are-on"])
print("[SEARCH]", " ".join(cmd), flush=True)
rc = subprocess.run(cmd, cwd=str(wam), check=False).returncode
sys.exit(rc)
PY

cd "$ROOT"
source .venv/bin/activate 2>/dev/null || true
aerial-inspect record-phase-run "$MISSION_DIR" search || true
