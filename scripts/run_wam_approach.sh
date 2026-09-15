#!/usr/bin/env bash
# APPROACH phase: fly to standoff waypoint with visual-prompt lock.
set -euo pipefail
MISSION_DIR="${1:?usage: run_wam_approach.sh artifacts/mission_xxx}"
WAM_ROOT="${AERIAL_WAM_ROOT:-$HOME/Projects/aerial-wam-v2}"
VGOAL_ROOT="${AERIAL_VGOAL_ROOT:-$HOME/Projects/aerial-vgoal-wam}"
PHASE_JSON="$MISSION_DIR/wam_phases.json"
MOCK="${MOCK_CAMERA:-1}"
MAVLINK_PORT="${MAVLINK_PORT:-/dev/ttyACM0}"

if [[ ! -f "$PHASE_JSON" ]]; then
  echo "missing $PHASE_JSON — run: aerial-inspect plan ... && aerial-inspect export-wam-phases $MISSION_DIR"
  exit 1
fi

export PYTHONPATH="$WAM_ROOT"
python3 <<PY
import json, subprocess, sys
from pathlib import Path

wam = Path("$WAM_ROOT")
data = json.loads(Path("$PHASE_JSON").read_text())
visual = data.get("visual_prompt", "bridge")
approach = data.get("approach", {})
goal = approach.get("goal") or {}
cmd = [
    sys.executable, "-m", "experiments.aerial.scripts.wam_vgoal_deploy",
    "--vgoal-repo", "$VGOAL_ROOT",
    "--mavlink-port", "$MAVLINK_PORT",
    "--visual-prompt", visual,
    "--max-steps", str(int(approach.get("max_steps", 200))),
    "--goal-x", str(goal["x"]), "--goal-y", str(goal["y"]), "--goal-z", str(goal["z"]),
    "--corpus-leg", "approach",
]
if approach.get("record_auto", True):
    cmd.append("--record-auto")
if "$MOCK" == "1":
    cmd.append("--mock-camera")
else:
    cmd.extend(["--offboard", "--run", "--i-know-props-are-on"])
print("[APPROACH]", goal, flush=True)
print(" ".join(cmd), flush=True)
subprocess.run(cmd, cwd=str(wam), check=False)
PY
