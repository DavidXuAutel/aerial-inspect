#!/usr/bin/env bash
# 84 + Avant-AirSim HumenCorridor (ports 2200 / 41463; OpenFly on 41451 unchanged).
# source scripts/env_sim_84_humen.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "$ROOT/scripts/env_sim_84.sh"

export AIRSIM_SCENE_ID=avant_humen_corridor
export AIRSIM_SCENE_CONFIG=configs/sim/humen_scene.yaml
export AIRSIM_PORT=41463
export AIRSIM_CAMERA=0
export AIRSIM_VEHICLE=SimpleFlight
export AVANT_AIRSIM_ROOT="${AVANT_AIRSIM_ROOT:-/data/linux/workspace/Avant-AirSim/Avant-AirSim}"
export BRIDGE_CAMERA_PITCH="${BRIDGE_CAMERA_PITCH:--20}"

echo "[env_sim_84_humen] AIRSIM=$AIRSIM_HOST:$AIRSIM_PORT scene=$AIRSIM_SCENE_CONFIG"
echo "[env_sim_84_humen] AVANT_AIRSIM_ROOT=$AVANT_AIRSIM_ROOT"
echo "[env_sim_84_humen] vehicle=$AIRSIM_VEHICLE camera=$AIRSIM_CAMERA pitch=$BRIDGE_CAMERA_PITCH"
