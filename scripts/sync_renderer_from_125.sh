#!/usr/bin/env bash
# Sync AirSim *render params* from 125 (settings.json). Scene map is env_airsim_16 bridge zone on 84.
set -euo pipefail

SRC_HOST="${SRC_HOST:-10.229.20.125}"
SRC_USER="${SRC_USER:-yao}"
SRC_PASS="${SRC_PASS:?set SRC_PASS for yao@125}"
DEST="${AIRSIM_PERSISTENT:-$HOME/aerial_airsim_persistent}"
SCENE_DIR="$DEST/scene/env_airsim_16"

_rsync() {
  local remote_path="$1"
  local local_path="$2"
  expect <<EOF
set timeout -1
spawn rsync -avz -e "ssh -o StrictHostKeyChecking=accept-new" ${SRC_USER}@${SRC_HOST}:${remote_path} ${local_path}
expect {
  "password:" { send "${SRC_PASS}\r"; exp_continue }
  "Password:" { send "${SRC_PASS}\r"; exp_continue }
  eof
}
EOF
}

mkdir -p "$DEST/AirSim" "$SCENE_DIR"

echo "[sync_renderer] settings.json from 125 (render params)"
_rsync "aerial_airsim_persistent/AirSim/settings.json" "$DEST/AirSim/settings.json"

if [[ ! -x "$SCENE_DIR/LinuxNoEditor/start.sh" ]]; then
  echo "[sync_renderer] bridge map env_airsim_16 (~1.9G) — first-time download"
  _rsync "aerial_airsim_persistent/scene/env_airsim_16/" "$SCENE_DIR/"
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
bash "$ROOT/scripts/patch_airsim_external_bind.sh"
echo "[sync_renderer] done. Bridge scene at $SCENE_DIR (external RPC ${AIRSIM_PUBLIC_HOST:-10.229.20.84}:41451)"
