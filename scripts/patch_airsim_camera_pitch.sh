#!/usr/bin/env bash
# Tilt front_custom camera down for bridge / ground inspection at altitude.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/scene_common.sh"

PERSIST="$AIRSIM_PERSISTENT"
PITCH="${BRIDGE_CAMERA_PITCH:-${AIRSIM_CAMERA_PITCH:--25}}"
PYTHON_BIN="${AERIAL_PY:-/data/linux/workspace/venvs/aerial-wam/bin/python}"
if [[ ! -x "$PYTHON_BIN" ]]; then
  PYTHON_BIN="$(command -v python3)"
fi

patch_json() {
  local f="$1"
  [[ -f "$f" ]] || return 0
  "$PYTHON_BIN" <<PY
import json
from pathlib import Path
p = Path("$f")
data = json.loads(p.read_text())
cam = data.get("Vehicles", {}).get("drone_1", {}).get("Cameras", {}).get("front_custom")
if cam is None:
    raise SystemExit(f"front_custom missing in {p}")
cam["Pitch"] = float("$PITCH")
p.write_text(json.dumps(data, indent=2) + "\n")
print(f"patched {p} front_custom.Pitch={cam['Pitch']}")
PY
}

patch_json "$PERSIST/AirSim/settings.json"
patch_json "$AIRSIM_SCENE_SETTINGS"
echo "[patch_camera_pitch] Pitch=$PITCH (restart renderer to apply)"
