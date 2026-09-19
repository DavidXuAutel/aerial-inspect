#!/usr/bin/env bash
# Mac / remote — HumenCorridor AirSim on 84 via public RPC (port 41463).
set -euo pipefail
_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
# shellcheck source=/dev/null
source "$_ROOT/scripts/mac_env_sim_84.sh"

export AIRSIM_HOST="${AIRSIM_HOST:-$AIRSIM_PUBLIC_HOST}"
export AIRSIM_PORT=41463
export AIRSIM_VEHICLE="${AIRSIM_VEHICLE:-SimpleFlight}"
export AIRSIM_CAMERA="${AIRSIM_CAMERA:-0}"
export AIRSIM_SCENE_ID="${AIRSIM_SCENE_ID:-avant_humen_corridor}"
export AIRSIM_SCENE_CONFIG="${AIRSIM_SCENE_CONFIG:-configs/sim/humen_scene.yaml}"
export BRIDGE_CAMERA_PITCH="${BRIDGE_CAMERA_PITCH:--20}"

echo "[mac_env_sim_84_humen] AIRSIM=${AIRSIM_HOST}:${AIRSIM_PORT} vehicle=${AIRSIM_VEHICLE} camera=${AIRSIM_CAMERA}"
