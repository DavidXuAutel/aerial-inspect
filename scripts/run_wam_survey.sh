#!/usr/bin/env bash
# Execute survey waypoints via WAM vgoal deploy (bench: --mock-camera; flight: add OFFBOARD).
set -euo pipefail
MISSION_DIR="${1:?usage: run_wam_survey.sh artifacts/mission_xxx}"
WAM_ROOT="${AERIAL_WAM_ROOT:-$HOME/Projects/aerial-wam-v2}"
VGOAL_ROOT="${AERIAL_VGOAL_ROOT:-$HOME/Projects/aerial-vgoal-wam}"
WP_JSON="$MISSION_DIR/wam_waypoints.json"
MOCK="${MOCK_CAMERA:-1}"
MAVLINK_PORT="${MAVLINK_PORT:-/dev/ttyACM0}"

export PYTHONPATH="$WAM_ROOT"
VISUAL=$(python3 -c "import json; d=json.load(open('$WP_JSON')); print(d.get('defaults',{}).get('visual_prompt') or 'bridge')")

python3 <<PY
import json, subprocess, sys, os
from pathlib import Path
wam = Path("$WAM_ROOT")
data = json.loads(Path("$WP_JSON").read_text())
for i, wp in enumerate(data["waypoints"]):
    cmd = [
        sys.executable, "-m", "experiments.aerial.scripts.wam_vgoal_deploy",
        "--vgoal-repo", "$VGOAL_ROOT",
        "--mavlink-port", "$MAVLINK_PORT",
        "--visual-prompt", "$VISUAL",
        "--goal-x", str(wp["x"]), "--goal-y", str(wp["y"]), "--goal-z", str(wp["z"]),
        "--max-steps", "150", "--record-auto",
    ]
    if "$MOCK" == "1":
        cmd.append("--mock-camera")
    else:
        cmd.extend(["--offboard", "--run", "--i-know-props-are-on"])
    print(f"[{i+1}/{len(data['waypoints'])}]", wp.get("label", ""), flush=True)
    subprocess.run(cmd, cwd=str(wam), check=False)
PY
