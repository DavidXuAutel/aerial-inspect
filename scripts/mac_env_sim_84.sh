#!/usr/bin/env bash
# Mac / remote client — connect to bridge AirSim renderer on 84 (no mixed 125).
set -euo pipefail

export AIRSIM_HOST="${AIRSIM_HOST:-10.229.20.84}"
export AIRSIM_PORT="${AIRSIM_PORT:-41451}"
export AIRSIM_PUBLIC_HOST="${AIRSIM_PUBLIC_HOST:-10.229.20.84}"
export AERIAL_WAM_ROOT="${AERIAL_WAM_ROOT:-$HOME/Projects/aerial-wam-v2}"
export AERIAL_VGOAL_ROOT="${AERIAL_VGOAL_ROOT:-$HOME/Projects/aerial-vgoal-wam}"
export AERIAL_INSPECT_ROOT="${AERIAL_INSPECT_ROOT:-$HOME/Projects/aerial-inspect}"

echo "[mac_env_sim_84] AIRSIM=${AIRSIM_HOST}:${AIRSIM_PORT}"
echo "[mac_env_sim_84] probe: python3 -c \"import socket;socket.create_connection(('${AIRSIM_HOST}',${AIRSIM_PORT}),5);print('ok')\""
