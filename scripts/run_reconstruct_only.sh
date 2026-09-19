#!/usr/bin/env bash
# Re-run hybrid COLMAP on existing capture_survey/ (no re-capture).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CAPTURE="${1:?usage: run_reconstruct_only.sh artifacts/<id>/capture_survey}"
cd "$ROOT"
PY="${AERIAL_PY:-python3}"
MISSION_ID=$(basename "$(dirname "$CAPTURE")")
WORKSPACE="$ROOT/artifacts/models/$MISSION_ID"

"$PY" "$ROOT/scripts/run_pycolmap_reconstruct.py" "$CAPTURE" \
  --workspace "$WORKSPACE" \
  --mode hybrid \
  "${@:2}"

echo "preview: $WORKSPACE/sparse_preview.png"
