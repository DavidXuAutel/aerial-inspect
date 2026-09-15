#!/usr/bin/env bash
# Start bridge-inspect AirSim renderer locally on 84.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ACTION="${1:-start}"

if [[ "$ACTION" == "stop" ]]; then
  bash "$ROOT/scripts/recover_renderer_bridge.sh" stop
else
  bash "$ROOT/scripts/recover_renderer_bridge.sh"
fi
