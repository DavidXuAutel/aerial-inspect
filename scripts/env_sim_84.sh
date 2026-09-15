#!/usr/bin/env bash
# 84 已有环境 — 复用 /data/linux/workspace/venvs/aerial-wam（勿重复 conda aerial_sim）
# source scripts/env_sim_84.sh
set -euo pipefail

export AERIAL_PY="${AERIAL_PY:-/data/linux/workspace/venvs/aerial-wam/bin/python}"
export PYTHON_BIN="$AERIAL_PY"
export AERIAL_WAM_ROOT="${AERIAL_WAM_ROOT:-$HOME/Projects/aerial-wam-v2}"
export AERIAL_VGOAL_ROOT="${AERIAL_VGOAL_ROOT:-$HOME/Projects/aerial-vgoal-wam}"
export PYTHONPATH="$AERIAL_WAM_ROOT:${PYTHONPATH:-}"

# AirSim 渲染在 125，84 作 eval 客户端（见 workspace setup_aerial.sh）
export AIRSIM_HOST="${AIRSIM_HOST:-10.229.20.125}"
export AIRSIM_PORT="${AIRSIM_PORT:-41451}"
export SIM_DEVICE="${SIM_DEVICE:-cuda}"
export SIM_DETECTOR="${SIM_DETECTOR:-open_vocab}"

export PATH="$(dirname "$AERIAL_PY"):$PATH"

echo "[env_sim_84] PYTHON_BIN=$PYTHON_BIN ($($PYTHON_BIN --version 2>&1))"
echo "[env_sim_84] AERIAL_WAM_ROOT=$AERIAL_WAM_ROOT"
echo "[env_sim_84] AIRSIM=$AIRSIM_HOST:$AIRSIM_PORT"
echo "[env_sim_84] cuda=$($PYTHON_BIN -c 'import torch; print(torch.cuda.is_available())' 2>/dev/null || echo unknown)"
