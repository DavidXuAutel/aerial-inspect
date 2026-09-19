#!/usr/bin/env python3
"""Corridor-filter + denoise a dense ground-truth depth cloud, then preview.

Motivation: a simple radial distance-from-centroid filter clips off long thin
structures (a bridge deck) while still letting in off-axis background clutter
(hills, opposite riverbank, city) that happens to be within radius. A
"corridor" filter aligned to the bridge span axis keeps the full deck length
while dropping off-axis clutter. Statistical outlier removal (kNN distance)
then cleans up floating noise points from low-res depth / grazing angles.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


def load_ply(path: Path) -> tuple[np.ndarray, np.ndarray]:
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


def write_ply(path: Path, pts: np.ndarray, cols: np.ndarray) -> None:
    with path.open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for (x, y, z), (r, g, b) in zip(pts, cols):
            f.write(f"{x:.4f} {y:.4f} {z:.4f} {int(r)} {int(g)} {int(b)}\n")


def corridor_filter(
    pts: np.ndarray,
    centroid_xy: tuple[float, float],
    span_axis_deg: float,
    along_max_m: float,
    cross_max_m: float,
    z_min: float | None,
    z_max: float | None,
) -> np.ndarray:
    cx, cy = centroid_xy
    rad = math.radians(span_axis_deg)
    ux, uy = math.cos(rad), math.sin(rad)
    dx = pts[:, 0] - cx
    dy = pts[:, 1] - cy
    along = dx * ux + dy * uy
    cross = -dx * uy + dy * ux
    keep = (np.abs(along) <= along_max_m) & (np.abs(cross) <= cross_max_m)
    if z_min is not None:
        keep &= pts[:, 2] >= z_min
    if z_max is not None:
        keep &= pts[:, 2] <= z_max
    return keep


def largest_component_filter(pts: np.ndarray, radius: float, min_size: int = 1) -> np.ndarray:
    """Keep only points in connected components with >= min_size points (drops small
    disconnected debris blobs that a per-point outlier filter would miss, since a
    small dense blob has low internal nearest-neighbor distance but is globally
    isolated)."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    from scipy.spatial import cKDTree

    tree = cKDTree(pts)
    pairs = tree.query_pairs(r=radius, output_type="ndarray")
    n = len(pts)
    if pairs.size == 0:
        return np.zeros(n, dtype=bool)
    rows = np.concatenate([pairs[:, 0], pairs[:, 1]])
    cols_ = np.concatenate([pairs[:, 1], pairs[:, 0]])
    data = np.ones(len(rows), dtype=np.int8)
    graph = coo_matrix((data, (rows, cols_)), shape=(n, n))
    n_comp, labels = connected_components(graph, directed=False)
    sizes = np.bincount(labels, minlength=n_comp)
    keep_labels = np.where(sizes >= min_size)[0]
    return np.isin(labels, keep_labels)


def outlier_filter(pts: np.ndarray, k: int, std_ratio: float) -> np.ndarray:
    """Drop points whose mean distance to k nearest neighbors is an outlier."""
    from scipy.spatial import cKDTree

    tree = cKDTree(pts)
    dist, _ = tree.query(pts, k=k + 1)  # includes self at dist 0
    mean_d = dist[:, 1:].mean(axis=1)
    mu, sigma = mean_d.mean(), mean_d.std()
    thresh = mu + std_ratio * sigma
    return mean_d <= thresh


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("ply_in")
    p.add_argument("--out-ply", required=True)
    p.add_argument("--centroid-xy", nargs=2, type=float, default=[2400.0, -20.0])
    p.add_argument("--span-axis-deg", type=float, default=26.0)
    p.add_argument("--along-max-m", type=float, default=650.0)
    p.add_argument("--cross-max-m", type=float, default=45.0)
    p.add_argument("--z-min", type=float, default=None)
    p.add_argument("--z-max", type=float, default=None)
    p.add_argument("--outlier-k", type=int, default=10)
    p.add_argument("--outlier-std-ratio", type=float, default=1.5)
    p.add_argument("--skip-outlier-filter", action="store_true")
    args = p.parse_args()

    pts, cols = load_ply(Path(args.ply_in))
    print(f"loaded {len(pts)} points")

    keep = corridor_filter(
        pts,
        tuple(args.centroid_xy),
        args.span_axis_deg,
        args.along_max_m,
        args.cross_max_m,
        args.z_min,
        args.z_max,
    )
    pts, cols = pts[keep], cols[keep]
    print(f"after corridor filter: {len(pts)} points")

    if not args.skip_outlier_filter and len(pts) > args.outlier_k + 1:
        keep2 = outlier_filter(pts, args.outlier_k, args.outlier_std_ratio)
        pts, cols = pts[keep2], cols[keep2]
        print(f"after outlier filter: {len(pts)} points")

    out_path = Path(args.out_ply)
    write_ply(out_path, pts, cols)
    print(f"wrote {out_path} ({len(pts)} points)")
    print(json.dumps({
        "n_points": int(len(pts)),
        "x_range": [float(pts[:, 0].min()), float(pts[:, 0].max())] if len(pts) else None,
        "y_range": [float(pts[:, 1].min()), float(pts[:, 1].max())] if len(pts) else None,
        "z_range": [float(pts[:, 2].min()), float(pts[:, 2].max())] if len(pts) else None,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
