#!/bin/bash
# One-shot complete bridge scan → clean QC deliverables (no patch merge).
set -euo pipefail
PY=/data/linux/workspace/venvs/aerial-wam/bin/python
ROOT=/home/ubantu/Projects/aerial-inspect
SCAN=$ROOT/artifacts/bridge_humen_001/deck_scan_complete
OUT=$ROOT/artifacts/bridge_humen_001/full_bridge_complete
LOG=$ROOT/artifacts/bridge_humen_001/complete_scan_pipeline.log

mkdir -p "$SCAN" "$OUT"
cd "$ROOT"
echo "[$(date)] COMPLETE full-bridge scan start" | tee "$LOG"
$PY scripts/fly_continuous_complete.py --out "$SCAN" --step-m 4.0 2>&1 | tee "$SCAN/run.log"
echo "[$(date)] scan done; finalize QC (this cloud alone)" | tee -a "$LOG"

# Finalize using complete cloud as sole source (no legacy patch merges)
$PY - <<'PY'
import sys
from pathlib import Path
import json
import numpy as np
sys.path.insert(0, "/home/ubantu/Projects/aerial-inspect/scripts")
from finalize_full_bridge import (
    load_ply_xyzrgb, write_ply, midspan_high_filter, along_track_bins, render_qc
)
import shutil

scan = Path("/home/ubantu/Projects/aerial-inspect/artifacts/bridge_humen_001/deck_scan_complete")
out = Path("/home/ubantu/Projects/aerial-inspect/artifacts/bridge_humen_001/full_bridge_complete")
out.mkdir(parents=True, exist_ok=True)
qc = out / "qc"; qc.mkdir(exist_ok=True)

pts, cols = load_ply_xyzrgb(scan / "complete_points.ply")
print("raw", len(pts), flush=True)
pts, cols = midspan_high_filter(pts, cols)
print("after filter", len(pts), flush=True)
write_ply(out / "FULL_BRIDGE_complete.ply", pts, cols)
write_ply(out / "看这个_整桥点云.ply", pts, cols)
shutil.copy2(out / "FULL_BRIDGE_complete.ply", out / "FULL_BRIDGE_merged.ply")
render_qc(pts, out, "complete-scan")
bins = along_track_bins(pts)
weak_deck = [b for b in bins if b["deck"] < 2000]
weak_cable = [b for b in bins if b["cable"] < 200]
report = {
    "status": "PASS" if not weak_deck and not weak_cable else "WEAK",
    "n_points": int(len(pts)),
    "source": "deck_scan_complete/complete_points.ply",
    "mode": "one_shot_complete",
    "weak_deck_bins": weak_deck,
    "weak_cable_bins": weak_cable,
    "bins": bins,
}
(qc / "complete_QC_report.json").write_text(json.dumps(report, indent=2))
shutil.copy2(out / "看这个_整桥QC.png", qc / "complete_QC.png")
shutil.copy2(out / "看这个_中段侧视.png", qc / "complete_mid_side.png")
(out / "README_看这里.json").write_text(json.dumps({
    "看这个_整桥点云.ply": "一次完整连续扫描（双面×全高度层）",
    "看这个_整桥QC.png": "俯视+侧视",
    "看这个_中段侧视.png": "中段侧视",
    "n_points": int(len(pts)),
    "status": report["status"],
    "mode": "one_shot_complete",
}, ensure_ascii=False, indent=2))
print(json.dumps({k: report[k] for k in ("status","n_points","weak_deck_bins","weak_cable_bins")}, indent=2), flush=True)

# mesh if open3d present
try:
    import open3d as o3d
except ImportError:
    print("no open3d, skip mesh", flush=True)
    raise SystemExit(0)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pts)
pcd.colors = o3d.utility.Vector3dVector(cols.astype(np.float64) / 255.0)
pcd = pcd.voxel_down_sample(0.7)
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=2.8, max_nn=40))
pcd.orient_normals_consistent_tangent_plane(50)
mesh, dens = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=10)
dens = np.asarray(dens)
mesh.remove_vertices_by_mask(dens < np.quantile(dens, 0.18))
mv = np.asarray(mesh.vertices)
deck = pts[(pts[:,2]>=120)&(pts[:,2]<140)]
if len(deck) >= 1000 and len(mv):
    mean = deck[:,:2].mean(0)
    _,_,vt = np.linalg.svd((deck[:,:2]-mean).astype(np.float64), full_matrices=False)
    nrm = np.array([-vt[0,1], vt[0,0]])
    mesh.remove_vertices_by_mask(np.abs((mv[:,:2]-mean)@nrm) > 45)
mesh.remove_degenerate_triangles(); mesh.remove_duplicated_triangles()
mesh.remove_duplicated_vertices(); mesh.remove_non_manifold_edges()
o3d.io.write_triangle_mesh(str(out/"bridge_mesh.ply"), mesh, write_ascii=True)
print(f"mesh verts={len(mesh.vertices)} faces={len(mesh.triangles)}", flush=True)

# mesh preview — readable side + top, not foggy isometric
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
v = np.asarray(mesh.vertices)
rng = np.random.default_rng(1)
idx = rng.choice(len(v), min(250000, len(v)), replace=False)
vp = v[idx]
fig, axes = plt.subplots(2, 1, figsize=(16, 9), gridspec_kw={"height_ratios": [1.1, 1]})
axes[0].scatter(vp[:,0], vp[:,1], s=0.2, c=vp[:,2], cmap="turbo", linewidths=0)
axes[0].set_aspect("equal", adjustable="datalim")
axes[0].set_title(f"mesh 俯视 verts={len(v)} faces={len(mesh.triangles)}")
axes[0].grid(True, alpha=0.2)
axes[1].scatter(vp[:,0], vp[:,2], s=0.25, c=vp[:,2], cmap="turbo", linewidths=0)
axes[1].set_ylim(float(v[:,2].min())-5, float(v[:,2].max())+5)
axes[1].set_title("mesh 侧视（桥面+主缆+塔）")
axes[1].grid(True, alpha=0.25)
fig.tight_layout()
fig.savefig(out/"看这个_整桥网格.png", dpi=160)
plt.close(fig)
fig, axes = plt.subplots(1,2, figsize=(14,5))
axes[0].scatter(vp[:,0], vp[:,1], s=0.2, c="k"); axes[0].set_aspect("equal"); axes[0].set_title("mesh top-down")
axes[1].scatter(vp[:,0], vp[:,2], s=0.2, c="k"); axes[1].set_title("mesh side"); axes[1].set_ylim(float(v[:,2].min())-5, float(v[:,2].max())+5)
fig.tight_layout(); fig.savefig(out/"看这个_网格正交.png", dpi=160)
(out/"mesh_report.json").write_text(json.dumps({
    "n_verts": int(len(mesh.vertices)), "n_faces": int(len(mesh.triangles)), "status": "ok"
}, indent=2))
PY

echo "[$(date)] ALL_DONE" | tee -a "$LOG"
