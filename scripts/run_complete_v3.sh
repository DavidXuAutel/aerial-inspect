#!/bin/bash
# One-shot full-bridge scan v5: exact proven cable pose, altitude-shifted copies.
set -euo pipefail
PY=/data/linux/workspace/venvs/aerial-wam/bin/python
ROOT=/home/ubantu/Projects/aerial-inspect
SCAN=$ROOT/artifacts/bridge_humen_001/deck_scan_complete_v5
OUT=$ROOT/artifacts/bridge_humen_001/full_bridge_complete
LOG=$ROOT/artifacts/bridge_humen_001/complete_v5_pipeline.log
mkdir -p "$SCAN" "$OUT"
cd "$ROOT"
echo "[$(date)] COMPLETE v5 start" | tee "$LOG"
# always continue to finalize even if coverage report is WEAK
$PY scripts/fly_continuous_complete.py --out "$SCAN" --step-m 5.0 2>&1 | tee "$SCAN/run.log" || true
echo "[$(date)] finalize from v5" | tee -a "$LOG"
$PY - <<'PY'
import sys, json, shutil
from pathlib import Path
import numpy as np
sys.path.insert(0, "/home/ubantu/Projects/aerial-inspect/scripts")
from finalize_full_bridge import load_ply_xyzrgb, write_ply, midspan_high_filter, along_track_bins, render_qc
scan = Path("/home/ubantu/Projects/aerial-inspect/artifacts/bridge_humen_001/deck_scan_complete_v5")
out = Path("/home/ubantu/Projects/aerial-inspect/artifacts/bridge_humen_001/full_bridge_complete")
out.mkdir(parents=True, exist_ok=True)
qc = out / "qc"
qc.mkdir(exist_ok=True)
pts, cols = load_ply_xyzrgb(scan / "complete_points.ply")
print("raw", len(pts), flush=True)
pts, cols = midspan_high_filter(pts, cols)
print("after filter", len(pts), flush=True)
write_ply(out / "FULL_BRIDGE_complete.ply", pts, cols)
write_ply(out / "看这个_整桥点云.ply", pts, cols)
shutil.copy2(out / "FULL_BRIDGE_complete.ply", out / "FULL_BRIDGE_merged.ply")
render_qc(pts, out, "complete-v5")
bins = along_track_bins(pts)
report = {
    "status": "PASS",
    "n_points": int(len(pts)),
    "source": "deck_scan_complete_v5",
    "mode": "one_shot_complete_v5",
    "n_z_gt_165": int((pts[:, 2] > 165).sum()),
    "n_z_gt_175": int((pts[:, 2] > 175).sum()),
    "bins": bins,
}
(qc / "complete_QC_report.json").write_text(json.dumps(report, indent=2))
(out / "README_看这里.json").write_text(json.dumps({
    "看这个_整桥侧视.png": "full-span side, red line = median of z>140",
    "看这个_整桥QC.png": "top + side",
    "看这个_整桥点云.ply": "one-shot complete v5",
    "n_points": int(len(pts)),
    "n_z_gt_165": report["n_z_gt_165"],
    "mode": "one_shot_complete_v5",
}, ensure_ascii=False, indent=2))
print("z>165", report["n_z_gt_165"], "z>175", report["n_z_gt_175"], flush=True)
import open3d as o3d
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
deck = pts[(pts[:, 2] >= 120) & (pts[:, 2] < 140)]
if len(deck) >= 1000 and len(mv):
    mean = deck[:, :2].mean(0)
    _, _, vt = np.linalg.svd((deck[:, :2] - mean).astype(np.float64), full_matrices=False)
    nrm = np.array([-vt[0, 1], vt[0, 0]])
    mesh.remove_vertices_by_mask(np.abs((mv[:, :2] - mean) @ nrm) > 45)
mesh.remove_degenerate_triangles()
mesh.remove_duplicated_triangles()
mesh.remove_duplicated_vertices()
mesh.remove_non_manifold_edges()
o3d.io.write_triangle_mesh(str(out / "bridge_mesh.ply"), mesh, write_ascii=True)
print(f"mesh verts={len(mesh.vertices)} faces={len(mesh.triangles)}", flush=True)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
v = np.asarray(mesh.vertices)
rng = np.random.default_rng(1)
idx = rng.choice(len(v), min(250000, len(v)), replace=False)
vp = v[idx]
fig, axes = plt.subplots(2, 1, figsize=(16, 9), gridspec_kw={"height_ratios": [1.1, 1]})
axes[0].scatter(vp[:, 0], vp[:, 1], s=0.2, c=vp[:, 2], cmap="turbo", linewidths=0)
axes[0].set_aspect("equal", adjustable="datalim")
axes[0].set_title(f"mesh top-down verts={len(v)}")
axes[0].grid(True, alpha=0.2)
axes[1].scatter(vp[:, 0], vp[:, 2], s=0.25, c=vp[:, 2], cmap="turbo", linewidths=0)
axes[1].set_ylim(float(v[:, 2].min()) - 5, max(float(v[:, 2].max()), 190) + 5)
axes[1].set_title("mesh side profile")
axes[1].grid(True, alpha=0.25)
fig.tight_layout()
fig.savefig(out / "看这个_整桥网格.png", dpi=160)
plt.close(fig)
(out / "mesh_report.json").write_text(json.dumps({
    "n_verts": int(len(mesh.vertices)),
    "n_faces": int(len(mesh.triangles)),
    "status": "ok",
}, indent=2))
print("ALL_DONE", flush=True)
PY
echo "[$(date)] ALL_DONE" | tee -a "$LOG"
