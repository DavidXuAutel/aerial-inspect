#!/bin/bash
# Bridge-inspect renderer on 84 — same AirSim params as 125, bridge map zone (env_airsim_16).
set -euo pipefail

ROOT="${AIRSIM_PERSISTENT:-$HOME/aerial_airsim_persistent}"
SCENE="$ROOT/scene/env_airsim_16/LinuxNoEditor"
PIDFILE="$ROOT/airsim.pid"
LOG="$ROOT/airsim.log"
SETTINGS="$ROOT/AirSim/settings.json"

stop_renderer() {
  [ -f "$PIDFILE" ] || return 0
  local pid cwd
  pid=$(cat "$PIDFILE" 2>/dev/null || true)
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    cwd=$(readlink "/proc/$pid/cwd" 2>/dev/null || true)
    if [ "$cwd" != "$SCENE" ]; then
      echo "Refusing to stop unrelated PID $pid (cwd=$cwd)" >&2
      exit 1
    fi
    kill -- "-$pid" 2>/dev/null || kill "$pid"
    sleep 3
  fi
  rm -f "$PIDFILE"
}

if [ "${1:-}" = "stop" ]; then
  stop_renderer
  echo "stopped"
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -x "$SCRIPT_DIR/patch_airsim_external_bind.sh" ]]; then
  bash "$SCRIPT_DIR/patch_airsim_external_bind.sh"
else
  mkdir -p "$HOME/Documents/AirSim"
  ln -sfn "$SETTINGS" "$HOME/Documents/AirSim/settings.json"
fi

export VK_ICD_FILENAMES="${VK_ICD_FILENAMES:-/usr/share/vulkan/icd.d/nvidia_icd.json}"
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader || true
if command -v vulkaninfo >/dev/null 2>&1; then
  vulkaninfo --summary 2>&1 | awk '/deviceName|driverName|driverInfo/{print}' | sort -u || true
fi

stop_renderer
cd "$SCENE"
nohup setsid ./start.sh \
  -Vulkan \
  -RenderOffScreen \
  -windowed \
  -ResX=1920 \
  -ResY=1080 \
  -nosound >"$LOG" 2>&1 </dev/null &
pid=$!
echo "$pid" >"$PIDFILE"

sleep 20
if ! kill -0 "$pid" 2>/dev/null; then
  echo "AirSim failed to stay running; inspect $LOG" >&2
  tail -40 "$LOG" >&2 || true
  exit 1
fi

echo "bridge scene=env_airsim_16 pid=$pid log=$LOG"
