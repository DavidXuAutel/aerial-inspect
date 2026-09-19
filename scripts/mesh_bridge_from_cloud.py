#!/usr/bin/env python3
"""Build a bridge mesh from the continuous depth cloud (Poisson / ball pivoting)."""
from __future__ import annotations

import argparse
import json
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
            if len(p) < 6:
                continue
            pts.append((float(p[0]), float(p[1]), float(p[2])))
            cols.append((int(p[3]), int(p[4]), int(p[5])))
    return np.asarray(pts, dtype=np.float64), np.asarray(cols, dtype=np.uint8)


def write_ply_mesh(path: Path, verts: np.ndarray, faces: np.ndarray, cols: np.ndarray | None = None) -> None:
    with path.open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(verts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        if cols is not None:
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write(f"element face {len(faces)}\n")
        f.write("property list uchar int vertex_indices\n")
        f.write("end_header\n")
        if cols is None:
            for x, y, z in verts:
                f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")
        else:
            for (x, y, z), (r, g, b) in zip(verts, cols):
                f.write(f"{x:.4f} {y:.4f} {z:.4f} {int(r)} {int(g)} {int(b)}\n")
        for a, b, c in faces:
            f.write(f"3 {int(a)} {int(b)} {int(c)}\n")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("ply_in")
    p.add_argument("--out-dir", required=True)
    p.add_argument("--voxel", type=float, default=0.8)
    p.add_argument("--depth", type=int, default=9)
    args = p.parse_args()

    import open3d as o3d

    pts, cols = load_ply_xyzrgb(Path(args.ply_in))
    print(f"loaded {len(pts)} pts", flush=True)
    # drop midspan false high again (safety)
    near_tower = (pts[:, 0] < 2500) | (pts[:, 0] > 2940)
    keep = ~((pts[:, 2] >= 158) & (~near_tower))
    pts, cols = pts[keep], cols[keep]
    print(f"after high filter {len(pts)}", flush=True)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(cols.astype(np.float64) / 255.0)
    pcd = pcd.voxel_down_sample(voxel_size=float(args.voxel))
    print(f"voxel {args.voxel} -> {len(pcd.points)}", flush=True)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=args.voxel * 4, max_nn=40))
    pcd.orient_normals_consistent_tangent_plane(50)

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=int(args.depth), width=0, scale=1.1, linear_fit=False
    )
    dens = np.asarray(densities)
    # trim low-density fringe
    thr = np.quantile(dens, 0.08)
    mesh.remove_vertices_by_mask(dens < thr)
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    print(f"mesh verts={len(mesh.vertices)} faces={len(mesh.triangles)}", flush=True)

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    mesh_path = out / "bridge_mesh.ply"
    o3d.io.write_triangle_mesh(str(mesh_path), mesh, write_ascii=True)
    # also colored point cloud for preview
    o3d.io.write_point_cloud(str(out / "bridge_cloud_vox.ply"), pcd)

    # render previews
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    v = np.asarray(mesh.vertices)
    f = np.asarray(mesh.triangles)
    # subsample faces for plot speed
    if len(f) > 80000:
        f = f[np.linspace(0, len(f) - 1, 80000).astype(int)]

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")
    tris = v[f]
    coll = Poly3DCollection(tris, alpha=0.55, linewidths=0.05, edgecolors="none")
    coll.set_facecolor((0.55, 0.55, 0.58))
    ax.add_collection3d(coll)
    ax.set_xlim(v[:, 0].min(), v[:, 0].max())
    ax.set_ylim(v[:, 1].min(), v[:, 1].max())
    ax.set_zlim(v[:, 2].min(), v[:, 2].max())
    ax.view_init(elev=22, azim=-55)
    ax.set_title(f"bridge mesh verts={len(v)} faces={len(np.asarray(mesh.triangles))}")
    fig.tight_layout()
    fig.savefig(out / "bridge_mesh_3d.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    axes[0].scatter(v[:, 0], v[:, 1], s=0.2, c="k")
    axes[0].set_aspect("equal")
    axes[0].set_title("mesh verts top-down")
    axes[1].scatter(v[:, 0], v[:, 2], s=0.2, c="k")
    axes[1].set_title("mesh verts side")
    fig.tight_layout()
    fig.savefig(out / "bridge_mesh_ortho.png", dpi=140)

    report = {
        "status": "ok",
        "n_input": int(len(pts)),
        "n_voxel": int(len(pcd.points)),
        "n_verts": int(len(mesh.vertices)),
        "n_faces": int(len(mesh.triangles)),
        "mesh": str(mesh_path),
    }
    (out / "mesh_report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
