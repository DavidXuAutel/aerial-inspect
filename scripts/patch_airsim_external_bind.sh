#!/usr/bin/env bash
# Bind AirSim RPC to all interfaces (0.0.0.0:41451) for cross-host clients (Mac / H100).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/scene_common.sh"

PERSIST="$AIRSIM_PERSISTENT"
PUBLIC_IP="${AIRSIM_PUBLIC_HOST:-10.229.20.84}"
BIND_IP="${AIRSIM_BIND_IP:-0.0.0.0}"

patch_json() {
  local f="$1"
  [[ -f "$f" ]] || return 0
  "$PYTHON_BIN" <<PY
import json
from pathlib import Path
p = Path("$f")
data = json.loads(p.read_text())
data["LocalHostIp"] = "$BIND_IP"
data["ApiServerPort"] = int(data.get("ApiServerPort", 41451))
bak = p.with_suffix(p.suffix + ".bak_external")
if not bak.exists():
    bak.write_text(p.read_text())
p.write_text(json.dumps(data, indent=2) + "\n")
print(f"patched {p} LocalHostIp=$BIND_IP")
PY
}

PYTHON_BIN="${AERIAL_PY:-/data/linux/workspace/venvs/aerial-wam/bin/python}"
if [[ ! -x "$PYTHON_BIN" ]]; then
  PYTHON_BIN="$(command -v python3)"
fi

patch_json "$PERSIST/AirSim/settings.json"
patch_json "$AIRSIM_SCENE_SETTINGS"

mkdir -p "$HOME/Documents/AirSim"
ln -sfn "$PERSIST/AirSim/settings.json" "$HOME/Documents/AirSim/settings.json"
echo "[patch_external] public clients: ${PUBLIC_IP}:41451 (restart renderer if running)"
