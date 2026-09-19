#!/usr/bin/env bash
# Humen bridge: full WAM sim pipeline (SEARCH→navigate→orbit capture→COLMAP).
# Do NOT use teleport-only export_sim_survey_frames for production capture.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG="${1:-configs/missions/bridge_humen.yaml}"
exec bash "$ROOT/scripts/run_humen_sim_pipeline.sh" "$CONFIG"
