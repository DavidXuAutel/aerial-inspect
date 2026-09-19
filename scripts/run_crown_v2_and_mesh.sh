#!/bin/bash
set -euo pipefail
PY=/data/linux/workspace/venvs/aerial-wam/bin/python
ROOT=/home/ubantu/Projects/aerial-inspect
CROWN_DIR=$ROOT/artifacts/bridge_humen_001/deck_scan_cable_crown_v2
MERGED=$ROOT/artifacts/bridge_humen_001/deck_scan_cable_crown_merged
CONT=$ROOT/artifacts/bridge_humen_001/deck_scan_continuous
DUAL=$ROOT/artifacts/bridge_humen_001/deck_scan_dual
OUT=$ROOT/artifacts/bridge_humen_001/full_bridge_final
LOG=$ROOT/artifacts/bridge_humen_001/finalize_full_bridge.log

mkdir -p "$CROWN_DIR" "$MERGED" "$OUT"
cd "$ROOT"

echo "[$(date)] crown v2 start" | tee -a "$CROWN_DIR/pipeline.log"
$PY scripts/fly_continuous_cable_crown.py --out "$CROWN_DIR" | tee "$CROWN_DIR/run.log"
echo "[$(date)] crown v2 done" | tee -a "$CROWN_DIR/pipeline.log"

$PY - <<'PY'
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, "/home/ubantu/Projects/aerial-inspect/scripts")
from finalize_full_bridge import load_ply_xyzrgb, write_ply
root = Path("/home/ubantu/Projects/aerial-inspect/artifacts/bridge_humen_001")
parts, cols = [], []
for p in [
    root / "deck_scan_cable_crown/cable_crown_points.ply",
    root / "deck_scan_cable_crown_v2/cable_crown_points.ply",
]:
    if p.exists():
        a, b = load_ply_xyzrgb(p)
        parts.append(a)
        cols.append(b)
        print(p.name, len(a), flush=True)
pts = np.concatenate(parts)
c = np.concatenate(cols)
out = root / "deck_scan_cable_crown_merged"
out.mkdir(exist_ok=True)
write_ply(out / "cable_crown_points.ply", pts, c)
print("merged", len(pts), flush=True)
PY

echo "[$(date)] finalize+mesh start" | tee -a "$CROWN_DIR/pipeline.log"
$PY "$ROOT/scripts/finalize_full_bridge.py" \
  --continuous "$CONT/continuous_points.ply" \
  --tower "$CONT/continuous_tower_points.ply" \
  --dual "$DUAL/dual_points.ply" \
  --crown "$MERGED/cable_crown_points.ply" \
  --out-dir "$OUT" \
  --voxel 0.7 --poisson-depth 10 | tee -a "$LOG"
echo "[$(date)] ALL_DONE" | tee -a "$CROWN_DIR/pipeline.log"
