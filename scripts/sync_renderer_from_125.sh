#!/usr/bin/env bash
# Sync AirSim render params (settings.json) from 125 → local. Bridge map env_airsim_16 on demand.
#
# Default SSH: cursor-125-public (Cloudflare, see ~/.ssh/config). Override:
#   SRC_SSH=yao@10.229.20.125 SRC_PASS=...   # direct LAN + password (e.g. on 84 without cf config)
set -euo pipefail

SRC_SSH="${SRC_SSH:-cursor-125-public}"
SRC_HOST="${SRC_HOST:-10.229.20.125}"
SRC_USER="${SRC_USER:-yao}"
SRC_PASS="${SRC_PASS:-}"
DEST="${AIRSIM_PERSISTENT:-$HOME/aerial_airsim_persistent}"
SCENE_DIR="$DEST/scene/env_airsim_16"
SSH_OPTS="-o StrictHostKeyChecking=accept-new"

_rsync_expect() {
  local remote_path="$1"
  local local_path="$2"
  expect <<EOF
set timeout -1
spawn rsync -avz -e "ssh ${SSH_OPTS}" ${SRC_USER}@${SRC_HOST}:${remote_path} ${local_path}
expect {
  "password:" { send "${SRC_PASS}\r"; exp_continue }
  "Password:" { send "${SRC_PASS}\r"; exp_continue }
  eof
}
EOF
}

_rsync() {
  local remote_path="$1"
  local local_path="$2"
  if [[ -n "$SRC_PASS" && "$SRC_SSH" != cursor-125-public ]]; then
    _rsync_expect "$remote_path" "$local_path"
  else
    rsync -avz -e "ssh ${SSH_OPTS}" "${SRC_SSH}:${remote_path}" "${local_path}"
  fi
}

mkdir -p "$DEST/AirSim" "$SCENE_DIR"

echo "[sync_renderer] from ${SRC_SSH} (125 render params)"
_rsync "aerial_airsim_persistent/AirSim/settings.json" "$DEST/AirSim/settings.json"

if [[ ! -x "$SCENE_DIR/LinuxNoEditor/start.sh" ]]; then
  echo "[sync_renderer] bridge map env_airsim_16 (~1.9G) — first-time download"
  _rsync "aerial_airsim_persistent/scene/env_airsim_16/" "$SCENE_DIR/"
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
bash "$ROOT/scripts/patch_airsim_external_bind.sh"
echo "[sync_renderer] done -> $SCENE_DIR (84 public RPC ${AIRSIM_PUBLIC_HOST:-10.229.20.84}:41451)"
