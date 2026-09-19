#!/usr/bin/env python3
"""Merge continuous + tower + dual clouds → clean → Poisson mesh → 看这个_* QC assets."""
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np


def load_ply_xyzrgb(path: Path) -> tuple[np.ndarray, np.ndarray]:
    pts, cols = [], []
    started = False
    with path.open() as f:
        for line in f:
            if line.strip() == "end_header":
                started = True
                continue
            if not started:
                continue
            p = line.split()
            if len(p) < 3:
                continue
            pts.append((float(p[0]), float(p[1]), float(p[2])))
            if len(p) >= 6:
                cols.append((int(p[3]), int(p[4]), int(p[5])))
            else:
                cols.append((160, 160, 160))
    return np.asarray(pts, dtype=np.float64), np.asarray(cols, dtype=np.uint8)


def write_ply(path: Path, pts: np.ndarray, cols: np.ndarray) -> None:
    with path.open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        for (x, y, z), (r, g, b) in zip(pts, cols):
            f.write(f"{x:.3f} {y:.3f} {z:.3f} {int(r)} {int(g)} {int(b)}\n")


def midspan_high_filter(
    pts: np.ndarray,
    cols: np.ndarray,
    *,
    centerline: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Drop midspan false sheets, keep real cable near the span axis.

    Old hard cut (z>=158 off towers) erased the midspan catenary that the
    complete look-at pass now captures. Keep high midspan points only if they
    sit within ``corridor_m`` of the centerline (or PCA span axis fallback).
    """
    high = pts[:, 2] >= 158.0
    near_tower = (pts[:, 0] < 2500) | (pts[:, 0] > 2940)
    mid_high = high & (~near_tower)
    if not mid_high.any():
        return pts, cols

    if centerline is not None and len(centerline) >= 2:
        cl = np.asarray(centerline, dtype=np.float64)
        tang = np.gradient(cl, axis=0)
        tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-9
        perp = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
        # nearest centerline vertex (numpy only — scipy not in remote venv)
        mh = pts[mid_high]
        near_cable = np.zeros(len(mh), dtype=bool)
        chunk = 50000
        for i0 in range(0, len(mh), chunk):
            sl = mh[i0 : i0 + chunk, :2]
            d2 = ((sl[:, None, :] - cl[None, :, :]) ** 2).sum(axis=2)
            j = d2.argmin(axis=1)
            cross = np.abs(((sl - cl[j]) * perp[j]).sum(axis=1))
            near_cable[i0 : i0 + chunk] = cross <= 28.0
    else:
        deck = pts[(pts[:, 2] >= 120) & (pts[:, 2] < 140)]
        if len(deck) < 1000:
            # fail-safe: keep hard cut
            keep = ~mid_high
            return pts[keep], cols[keep]
        mean = deck[:, :2].mean(axis=0)
        _, _, vt = np.linalg.svd((deck[:, :2] - mean).astype(np.float64), full_matrices=False)
        nrm = np.array([-vt[0, 1], vt[0, 0]], dtype=np.float64)
        cross = np.abs((pts[mid_high, :2] - mean) @ nrm)
        near_cable = cross <= 28.0

    drop = np.zeros(len(pts), dtype=bool)
    drop_idx = np.where(mid_high)[0][~near_cable]
    drop[drop_idx] = True
    keep = ~drop
    return pts[keep], cols[keep]


def tower_midspan_strip(pts: np.ndarray, cols: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Tower look-at pass floods midspan with a thick sheet above the deck.

    Keep tower points only near the pylons; midspan upper structure comes from
    continuous/dual/crown instead.
    """
    near_tower = (pts[:, 0] < 2520) | (pts[:, 0] > 2920)
    keep = near_tower | (pts[:, 2] < 145.0)
    return pts[keep], cols[keep]


def append_crown(
    pts: np.ndarray, cols: np.ndarray, crown_pts: np.ndarray, crown_cols: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Append crown points after sheet filter (crown already corridor-clipped)."""
    if len(crown_pts) == 0:
        return pts, cols
    # light cap: drop absurd sky
    keep = crown_pts[:, 2] < 175.0
    return np.concatenate([pts, crown_pts[keep]]), np.concatenate([cols, crown_cols[keep]])


def along_track_bins(pts: np.ndarray, x0: float = 2320.0, x1: float = 3120.0, dx: float = 40.0):
    edges = np.arange(x0, x1 + dx, dx)
    deck = pts[(pts[:, 2] >= 120) & (pts[:, 2] < 142)]
    cable = pts[(pts[:, 2] >= 136) & (pts[:, 2] < 175)]
    rows = []
    for a, b in zip(edges[:-1], edges[1:]):
        nd = int(((deck[:, 0] >= a) & (deck[:, 0] < b)).sum())
        nc = int(((cable[:, 0] >= a) & (cable[:, 0] < b)).sum())
        rows.append({"x0": float(a), "x1": float(b), "deck": nd, "cable": nc})
    return rows


def _setup_qc_font() -> None:
    import matplotlib

    matplotlib.rcParams["font.sans-serif"] = [
        "PingFang SC",
        "Heiti SC",
        "Arial Unicode MS",
        "Noto Sans CJK SC",
        "DejaVu Sans",
    ]
    matplotlib.rcParams["axes.unicode_minus"] = False


# AirSim / Unreal world frame used by the scan poses: z up, metres.
_XLAB = "x  沿桥向（米，AirSim 世界坐标）"
_YLAB = "y  横桥向（米，AirSim 世界坐标）"
_ZLAB = "z  高度（米，向上为正，AirSim 世界坐标）"


def render_qc(pts: np.ndarray, out_dir: Path, tag: str) -> None:
    """Readable bridge QC — equal aspect, clear side profile, no foggy 3D."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    _setup_qc_font()
    out_dir = Path(out_dir)
    # subsample for plot speed / clarity
    rng = np.random.default_rng(0)
    if len(pts) > 400000:
        idx = rng.choice(len(pts), 400000, replace=False)
        plot = pts[idx]
    else:
        plot = pts

    # --- mid-span side (dense window) ---
    mid = pts[(pts[:, 0] >= 2600) & (pts[:, 0] <= 2800)]
    if len(mid) > 200000:
        mid = mid[rng.choice(len(mid), 200000, replace=False)]
    fig, ax = plt.subplots(figsize=(14, 6))
    if len(mid):
        ax.scatter(mid[:, 0], mid[:, 2], s=0.4, c=mid[:, 2], cmap="turbo", linewidths=0)
    ax.set_xlabel(_XLAB)
    ax.set_ylabel(_ZLAB)
    sc = ax.collections[-1] if ax.collections else None
    if sc is not None:
        cb = fig.colorbar(sc, ax=ax, fraction=0.03, pad=0.02)
        cb.set_label("颜色 = 高度 z（米）")
    ax.set_title(f"{tag}  中跨侧视  x∈[2600, 2800] m   点数={len(mid)}")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_dir / "看这个_中段侧视.png", dpi=160)
    plt.close(fig)

    # --- full bridge: top + side with equal-ish aspect so shape is obvious ---
    fig, axes = plt.subplots(2, 1, figsize=(16, 10), gridspec_kw={"height_ratios": [1.2, 1]})
    axes[0].scatter(plot[:, 0], plot[:, 1], s=0.15, c=plot[:, 2], cmap="turbo", linewidths=0)
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].set_title(f"{tag}  俯视（颜色 = 高度）  点数={len(pts)}")
    axes[0].set_xlabel(_XLAB)
    axes[0].set_ylabel(_YLAB)
    axes[0].grid(True, alpha=0.2)

    axes[1].scatter(plot[:, 0], plot[:, 2], s=0.2, c=plot[:, 2], cmap="turbo", linewidths=0)
    # median height along x so the catenary reads even when the cloud is thin
    xb = np.arange(pts[:, 0].min(), pts[:, 0].max(), 8.0)
    med = []
    for a in xb:
        sl = pts[(pts[:, 0] >= a) & (pts[:, 0] < a + 8) & (pts[:, 2] > 140)]
        med.append(float(np.median(sl[:, 2])) if len(sl) > 30 else np.nan)
    axes[1].plot(xb + 4, med, color="red", lw=1.2, label="z>140 m 的中位数")
    z0, z1 = float(pts[:, 2].min()), float(pts[:, 2].max())
    axes[1].set_ylim(z0 - 5, max(z1, 190) + 5)
    axes[1].set_title("整桥侧视：下条为桥面，上条为主缆，两端竖条为桥塔")
    axes[1].set_xlabel(_XLAB)
    axes[1].set_ylabel(_ZLAB)
    axes[1].grid(True, alpha=0.25)
    # mark approximate tower zones
    axes[1].axvline(2400, color="red", ls="--", lw=0.8, alpha=0.6, label="tower zone")
    axes[1].axvline(3100, color="red", ls="--", lw=0.8, alpha=0.6)
    axes[1].legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_dir / "看这个_整桥QC.png", dpi=160)
    plt.close(fig)

    # --- dedicated clear side profile (no top) for "can I see the bridge" ---
    fig, ax = plt.subplots(figsize=(16, 5.5))
    ax.scatter(plot[:, 0], plot[:, 2], s=0.25, c=plot[:, 2], cmap="turbo", linewidths=0)
    ax.plot(xb + 4, med, color="red", lw=1.4, label="z>140 m 的中位数（不是主缆上沿）")
    ax.set_ylim(z0 - 5, max(z1, 190) + 5)
    ax.set_xlabel(_XLAB)
    ax.set_ylabel(_ZLAB)
    cb = fig.colorbar(ax.collections[0], ax=ax, fraction=0.046, pad=0.02)
    cb.set_label("颜色 = 高度 z（米）")
    ax.set_title(f"{tag}  整桥侧视  点数={len(pts)}  单位：米")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_dir / "看这个_整桥侧视.png", dpi=160)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--continuous", required=True)
    ap.add_argument("--tower", default="")
    ap.add_argument("--dual", default="")
    ap.add_argument("--crown", default="")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--voxel", type=float, default=0.8)
    ap.add_argument("--poisson-depth", type=int, default=9)
    ap.add_argument("--skip-mesh", action="store_true")
    args = ap.parse_args()

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    qc = out / "qc"
    qc.mkdir(exist_ok=True)

    clouds = [("continuous", Path(args.continuous))]
    if args.tower:
        clouds.append(("tower", Path(args.tower)))
    if args.dual:
        clouds.append(("dual", Path(args.dual)))
    crown_path = Path(args.crown) if args.crown else None

    parts_pts, parts_cols, counts = [], [], {}
    for name, path in clouds:
        if not path.exists():
            raise SystemExit(f"missing {name}: {path}")
        p, c = load_ply_xyzrgb(path)
        if name == "tower":
            before = len(p)
            p, c = tower_midspan_strip(p, c)
            print(f"tower midspan strip {before} -> {len(p)}", flush=True)
        counts[name] = int(len(p))
        print(f"loaded {name}: {len(p)}", flush=True)
        parts_pts.append(p)
        parts_cols.append(c)

    pts = np.concatenate(parts_pts)
    cols = np.concatenate(parts_cols)
    print(f"merged raw {len(pts)}", flush=True)
    pts, cols = midspan_high_filter(pts, cols)
    print(f"after midspan-high filter {len(pts)}", flush=True)

    if crown_path is not None:
        if not crown_path.exists():
            raise SystemExit(f"missing crown: {crown_path}")
        cp, cc = load_ply_xyzrgb(crown_path)
        counts["crown"] = int(len(cp))
        print(f"loaded crown: {len(cp)}", flush=True)
        pts, cols = append_crown(pts, cols, cp, cc)
        print(f"after crown append {len(pts)}", flush=True)

    merged = out / "FULL_BRIDGE_merged.ply"
    write_ply(merged, pts, cols)
    shutil.copy2(merged, out / "FULL_BRIDGE_continuous.ply")
    shutil.copy2(merged, out / "看这个_整桥点云.ply")

    bins = along_track_bins(pts)
    weak_deck = [b for b in bins if b["deck"] < 2000]
    weak_cable = [b for b in bins if b["cable"] < 50]
    render_qc(pts, out, "dual+continuous")

    report = {
        "status": "PASS" if not weak_deck else "WEAK_DECK",
        "n_points": int(len(pts)),
        "counts": counts,
        "weak_deck_bins": weak_deck,
        "weak_cable_bins": weak_cable,
        "bins": bins,
        "merged_ply": str(merged),
    }
    (qc / "continuous_QC_report.json").write_text(json.dumps(report, indent=2))
    shutil.copy2(out / "看这个_整桥QC.png", qc / "continuous_QC.png")
    shutil.copy2(out / "看这个_中段侧视.png", qc / "continuous_mid_side.png")
    (out / "README_看这里.json").write_text(
        json.dumps(
            {
                "看这个_整桥点云.ply": "merged continuous+tower+dual cleaned cloud",
                "看这个_整桥QC.png": "top+side QC",
                "看这个_中段侧视.png": "mid-span side",
                "bridge_mesh.ply": "Poisson mesh (if built)",
                "n_points": int(len(pts)),
                "status": report["status"],
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    print(json.dumps({k: report[k] for k in ("status", "n_points", "counts", "weak_deck_bins", "weak_cable_bins")}, indent=2), flush=True)

    if args.skip_mesh:
        return 0

    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(cols.astype(np.float64) / 255.0)
    pcd = pcd.voxel_down_sample(voxel_size=float(args.voxel))
    print(f"voxel {args.voxel} -> {len(pcd.points)}", flush=True)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=args.voxel * 4, max_nn=40)
    )
    pcd.orient_normals_consistent_tangent_plane(50)
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=int(args.poisson_depth), width=0, scale=1.1, linear_fit=False
    )
    dens = np.asarray(densities)
    thr = np.quantile(dens, 0.18)
    mesh.remove_vertices_by_mask(dens < thr)
    # keep only verts near the span ribbon (drop Poisson fill far off deck)
    mv = np.asarray(mesh.vertices)
    deck = pts[(pts[:, 2] >= 120) & (pts[:, 2] < 140)]
    if len(deck) >= 1000 and len(mv):
        mean = deck[:, :2].mean(axis=0)
        _, _, vt = np.linalg.svd((deck[:, :2] - mean).astype(np.float64), full_matrices=False)
        axis = vt[0]
        nrm = np.array([-axis[1], axis[0]], dtype=np.float64)
        cross = np.abs((mv[:, :2] - mean) @ nrm)
        mesh.remove_vertices_by_mask(cross > 45.0)
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    print(f"mesh verts={len(mesh.vertices)} faces={len(mesh.triangles)}", flush=True)
    mesh_path = out / "bridge_mesh.ply"
    o3d.io.write_triangle_mesh(str(mesh_path), mesh, write_ascii=True)
    o3d.io.write_point_cloud(str(out / "bridge_cloud_vox.ply"), pcd)

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    v = np.asarray(mesh.vertices)
    f = np.asarray(mesh.triangles)
    # scatter verts (Poly3DCollection on large meshes looks like fog)
    if len(v) > 120000:
        idx = np.linspace(0, len(v) - 1, 120000).astype(int)
        v_plot = v[idx]
    else:
        v_plot = v
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(v_plot[:, 0], v_plot[:, 1], v_plot[:, 2], s=0.15, c=v_plot[:, 2], cmap="viridis")
    ax.set_xlim(v[:, 0].min(), v[:, 0].max())
    ax.set_ylim(v[:, 1].min(), v[:, 1].max())
    ax.set_zlim(v[:, 2].min(), v[:, 2].max())
    ax.view_init(elev=22, azim=-55)
    ax.set_title(f"bridge mesh verts={len(v)} faces={len(f)}")
    fig.tight_layout()
    fig.savefig(out / "看这个_整桥网格.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    axes[0].scatter(v[:, 0], v[:, 1], s=0.2, c="k")
    axes[0].set_aspect("equal")
    axes[0].set_title("mesh top-down")
    axes[1].scatter(v[:, 0], v[:, 2], s=0.2, c="k")
    axes[1].set_title("mesh side")
    fig.tight_layout()
    fig.savefig(out / "看这个_网格正交.png", dpi=140)
    plt.close(fig)

    mesh_report = {
        "status": "ok",
        "n_input": int(len(pts)),
        "n_voxel": int(len(pcd.points)),
        "n_verts": int(len(mesh.vertices)),
        "n_faces": int(len(mesh.triangles)),
        "mesh": str(mesh_path),
    }
    (out / "mesh_report.json").write_text(json.dumps(mesh_report, indent=2))
    print(json.dumps(mesh_report, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
