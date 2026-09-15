#!/usr/bin/env bash
# Copy AirSim persistent tree from 125 → local host (84). Keeps same settings/scene as 125.
set -euo pipefail

SRC_HOST="${SRC_HOST:-10.229.20.125}"
SRC_USER="${SRC_USER:-yao}"
SRC_PASS="${SRC_PASS:?set SRC_PASS for yao@125}"
DEST="${AIRSIM_PERSISTENT:-$HOME/aerial_airsim_persistent}"

mkdir -p "$DEST/scene/env_airsim_16" "$DEST/AirSim"

_rsync() {
  local remote_path="$1"
  local local_path="$2"
  expect <<EOF
set timeout -1
spawn rsync -avz --progress -e "ssh -o StrictHostKeyChecking=accept-new" ${SRC_USER}@${SRC_HOST}:${remote_path} ${local_path}
expect {
  "password:" { send "${SRC_PASS}\r"; exp_continue }
  "Password:" { send "${SRC_PASS}\r"; exp_continue }
  eof
}
EOF
}

echo "[sync_renderer] AirSim settings + recover script"
_rsync "aerial_airsim_persistent/AirSim/" "$DEST/AirSim/"
_rsync "aerial_airsim_persistent/recover_renderer.sh" "$DEST/recover_renderer.sh"

echo "[sync_renderer] scene env_airsim_16 (~1.9G)"
_rsync "aerial_airsim_persistent/scene/env_airsim_16/" "$DEST/scene/env_airsim_16/"

sed -i "s|/home/yao|$HOME|g" "$DEST/recover_renderer.sh"
chmod +x "$DEST/recover_renderer.sh"
mkdir -p "$HOME/Documents/AirSim"
ln -sfn "$DEST/AirSim/settings.json" "$HOME/Documents/AirSim/settings.json"
echo "[sync_renderer] done -> $DEST"
