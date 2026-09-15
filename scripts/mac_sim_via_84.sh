#!/usr/bin/env bash
# Mac / remote client: connect to bridge AirSim renderer on 84 (external LAN).
set -euo pipefail

export AIRSIM_HOST="${AIRSIM_HOST:-10.229.20.84}"
export AIRSIM_PORT="${AIRSIM_PORT:-41451}"
export AIRSIM_PUBLIC_HOST="${AIRSIM_PUBLIC_HOST:-10.229.20.84}"

echo "AIRSIM=$AIRSIM_HOST:$AIRSIM_PORT"
python3 - <<'PY'
import os, socket, sys
host = os.environ["AIRSIM_HOST"]
port = int(os.environ["AIRSIM_PORT"])
try:
    socket.create_connection((host, port), timeout=5).close()
    print(f"RPC reachable: {host}:{port}")
except OSError as e:
    print(f"RPC NOT reachable: {host}:{port} ({e})", file=sys.stderr)
    sys.exit(1)
try:
    import airsim
    c = airsim.MultirotorClient(ip=host, port=port)
    c.confirmConnection()
    print("AirSim confirmConnection: OK")
except Exception as e:
    print(f"AirSim client failed: {e}", file=sys.stderr)
    sys.exit(1)
PY
