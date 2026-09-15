#!/usr/bin/env bash
# Start local AirSim on 84 — same recover_renderer.sh / settings as 125.
set -euo pipefail
PERSIST="${AIRSIM_PERSISTENT:-$HOME/aerial_airsim_persistent}"
SCRIPT="$PERSIST/recover_renderer.sh"
ACTION="${1:-start}"

mkdir -p "$HOME/Documents/AirSim"
ln -sfn "$PERSIST/AirSim/settings.json" "$HOME/Documents/AirSim/settings.json"

if [[ ! -x "$SCRIPT" ]]; then
  echo "missing $SCRIPT — run: bash scripts/sync_renderer_from_125.sh" >&2
  exit 1
fi

export VK_ICD_FILENAMES="${VK_ICD_FILENAMES:-/usr/share/vulkan/icd.d/nvidia_icd.json}"
if [[ "$ACTION" == "stop" ]]; then
  bash "$SCRIPT" stop
else
  bash "$SCRIPT"
fi
