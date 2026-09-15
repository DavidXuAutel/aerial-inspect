#!/usr/bin/env bash
# Bootstrap AirSim on 84: rsync scene + settings from 125 (same params), then start locally.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
bash "$ROOT/scripts/sync_renderer_from_125.sh"
echo "[setup_renderer_84] done. Next: bash scripts/start_renderer_84.sh"
