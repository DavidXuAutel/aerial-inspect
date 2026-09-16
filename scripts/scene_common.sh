#!/usr/bin/env bash
# Shared AirSim scene paths (source from other scripts).
# Override: export AIRSIM_SCENE_ID=env_airsim_16
set -euo pipefail

export AIRSIM_SCENE_ID="${AIRSIM_SCENE_ID:-env_airsim_16}"
export AIRSIM_PERSISTENT="${AIRSIM_PERSISTENT:-$HOME/aerial_airsim_persistent}"
export AIRSIM_SCENE_ROOT="${AIRSIM_SCENE_ROOT:-$AIRSIM_PERSISTENT/scene/$AIRSIM_SCENE_ID}"
export AIRSIM_SCENE_BIN="${AIRSIM_SCENE_BIN:-$AIRSIM_SCENE_ROOT/LinuxNoEditor}"
export AIRSIM_SCENE_SETTINGS="${AIRSIM_SCENE_SETTINGS:-$AIRSIM_SCENE_BIN/AirVLN/Binaries/Linux/settings.json}"
