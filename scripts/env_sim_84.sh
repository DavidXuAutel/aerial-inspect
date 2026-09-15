#!/usr/bin/env bash
# 84 已有环境 — 复用 /data/linux/workspace/venvs/aerial-wam（勿重复 conda aerial_sim）
# source scripts/env_sim_84.sh
set -euo pipefail

export AERIAL_PY="${AERIAL_PY:-/data/linux/workspace/venvs/aerial-wam/bin/python}"
export PYTHON_BIN="$AERIAL_PY"
export AERIAL_WAM_ROOT="${AERIAL_WAM_ROOT:-$HOME/Projects/aerial-wam-v2}"
export AERIAL_VGOAL_ROOT="${AERIAL_VGOAL_ROOT:-$HOME/Projects/aerial-vgoal-wam}"
export PYTHONPATH="$AERIAL_WAM_ROOT:${PYTHONPATH:-}"

# 84 本机跑 eval 用 loopback；Mac/H100 等外部客户端用 AIRSIM_PUBLIC_HOST（见 mac_env_sim_84.sh）
export AIRSIM_PUBLIC_HOST="${AIRSIM_PUBLIC_HOST:-10.229.20.84}"
export AIRSIM_HOST="${AIRSIM_HOST:-127.0.0.1}"
export AIRSIM_PORT="${AIRSIM_PORT:-41451}"
export AIRSIM_BIND_IP="${AIRSIM_BIND_IP:-0.0.0.0}"
export AIRSIM_CAMERA="${AIRSIM_CAMERA:-front_custom}"
export AIRSIM_VEHICLE="${AIRSIM_VEHICLE:-drone_1}"
export AIRSIM_PERSISTENT="${AIRSIM_PERSISTENT:-$HOME/aerial_airsim_persistent}"
export SIM_DEVICE="${SIM_DEVICE:-cuda}"
export SIM_DETECTOR="${SIM_DETECTOR:-open_vocab}"

export PATH="$(dirname "$AERIAL_PY"):$PATH"

echo "[env_sim_84] PYTHON_BIN=$PYTHON_BIN ($($PYTHON_BIN --version 2>&1))"
echo "[env_sim_84] AERIAL_WAM_ROOT=$AERIAL_WAM_ROOT"
echo "[env_sim_84] AIRSIM=$AIRSIM_HOST:$AIRSIM_PORT (public ${AIRSIM_PUBLIC_HOST}:$AIRSIM_PORT)"
echo "[env_sim_84] cuda=$($PYTHON_BIN -c 'import torch; print(torch.cuda.is_available())' 2>/dev/null || echo unknown)"
