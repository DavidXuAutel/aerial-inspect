#!/bin/bash
set -euo pipefail
PY=/data/linux/workspace/venvs/aerial-wam/bin/python
ROOT=/home/ubantu/Projects/aerial-inspect
TPL=$ROOT/artifacts/bridge_humen_001/deck_scan_cable_tpl
CONT=$ROOT/artifacts/bridge_humen_001/deck_scan_continuous
DUAL=$ROOT/artifacts/bridge_humen_001/deck_scan_dual
CROWN=$ROOT/artifacts/bridge_humen_001/deck_scan_cable_crown_merged
OUT=$ROOT/artifacts/bridge_humen_001/full_bridge_final
LOG=$ROOT/artifacts/bridge_humen_001/cable_tpl_pipeline.log

mkdir -p "$TPL" "$OUT"
cd "$ROOT"
echo "[$(date)] cable tpl densify (no look-at)" | tee "$LOG"
$PY scripts/fly_continuous_cable_tpl.py --out "$TPL" --x0 2580 --x1 2960 2>&1 | tee "$TPL/run.log"
echo "[$(date)] merge clean crown+tpl (drop look-at body sheet)" | tee -a "$LOG"

$PY - <<'PY'
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, "/home/ubantu/Projects/aerial-inspect/scripts")
from finalize_full_bridge import load_ply_xyzrgb, write_ply
root = Path("/home/ubantu/Projects/aerial-inspect/artifacts/bridge_humen_001")
parts, cols = [], []
# ONLY crown (corridor-clipped) + new template pass — NOT look-at body sheet
for p in [
    root / "deck_scan_cable_crown_merged/cable_crown_points.ply",
    root / "deck_scan_cable_tpl/cable_tpl_points.ply",
]:
    if p.exists():
        a, b = load_ply_xyzrgb(p)
        parts.append(a); cols.append(b)
        print(p.name, len(a), flush=True)
pts = np.concatenate(parts); c = np.concatenate(cols)
out = root / "deck_scan_cable_extras_clean"
out.mkdir(exist_ok=True)
write_ply(out / "cable_extras_points.ply", pts, c)
print("extras_clean", len(pts), flush=True)
PY

$PY scripts/finalize_full_bridge.py \
  --continuous "$CONT/continuous_points.ply" \
  --tower "$CONT/continuous_tower_points.ply" \
  --dual "$DUAL/dual_points.ply" \
  --crown "$ROOT/artifacts/bridge_humen_001/deck_scan_cable_extras_clean/cable_extras_points.ply" \
  --out-dir "$OUT" \
  --voxel 0.7 --poisson-depth 10 2>&1 | tee -a "$LOG"
echo "[$(date)] ALL_DONE" | tee -a "$LOG"
