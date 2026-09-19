#!/usr/bin/env python3
"""Diagnose uneven known-pose triangulation density across survey frames."""
from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np


def _frame_index(name: str) -> int:
    return int(Path(name).stem.split("_")[-1])


def _infer_lap_stride(n: int) -> int:
    return n // 2 if n >= 16 and n % 2 == 0 else 0


def _pair_ok(i1: int, i2: int, n: int, overlap: int, lap_stride: int) -> bool:
    if i1 == i2:
        return False
    d = abs(i2 - i1)
    if lap_stride > 0 and d == lap_stride:
        return True
    return min(d, n - d) <= overlap


def main() -> int:
    p = argparse.ArgumentParser(description="Analyze sparse point density per frame")
    p.add_argument("capture_dir", help="capture_survey with frames/ and traj.jsonl")
    p.add_argument("--workspace", help="models workspace with database.db")
    p.add_argument("--centroid", nargs=3, type=float, default=None)
    p.add_argument("--sequential-overlap", type=int, default=10)
    p.add_argument("--lap-stride", type=int, default=0)
    p.add_argument("--hfov-deg", type=float, default=0.0)
    args = p.parse_args()

    capture = Path(args.capture_dir).resolve()
    mission_dir = capture.parent
    if args.workspace:
        workspace = Path(args.workspace).resolve()
    else:
        workspace = mission_dir.parent / "models" / mission_dir.name
        if not (workspace / "database.db").is_file():
            workspace = mission_dir.parent.parent / "models" / mission_dir.name
    db_path = workspace / "database.db"
    traj_path = capture / "traj.jsonl"
    if not db_path.is_file():
        raise SystemExit(f"missing {db_path} — run reconstruct first")
    if not traj_path.is_file():
        raise SystemExit(f"missing {traj_path}")

    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import pycolmap
    from colmap_known_pose import _image_size, _intrinsics_from_hfov, load_traj_poses
    from triangulate_from_traj import (
        _pair_ok as pair_ok_names,
        _projection_matrix,
        _reproj_err,
        cv2_triangulate,
        traj_row_to_world_to_cam,
    )

    poses = load_traj_poses(traj_path)
    names = sorted(poses.keys(), key=_frame_index)
    n_frames = len(names)
    lap_stride = args.lap_stride or _infer_lap_stride(n_frames)
    overlap = args.sequential_overlap

    spec = mission_dir / "mission_spec.json"
    hfov = args.hfov_deg
    if hfov <= 0 and spec.is_file():
        hfov = float(json.loads(spec.read_text()).get("survey", {}).get("camera_hfov_deg", 80.0))
    if hfov <= 0:
        hfov = 80.0

    centroid = args.centroid
    if centroid is None and spec.is_file():
        c = json.loads(spec.read_text()).get("bridge_centroid_xyz")
        if c:
            centroid = [float(c[0]), float(c[1]), float(c[2])]
    centroid = np.asarray(centroid or [0.0, 0.0, 0.0], dtype=np.float64)

    width, height = _image_size(capture / "frames")
    fx, fy, cx, cy = _intrinsics_from_hfov(width, height, hfov)
    k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

    db = pycolmap.Database.open(str(db_path))
    images = {img.image_id: img for img in db.read_all_images()}
    keypoints: dict[int, np.ndarray] = {}
    feat_counts: dict[str, int] = {}
    for image_id, img in images.items():
        kp = db.read_keypoints(image_id)
        keypoints[image_id] = np.asarray(kp, dtype=np.float64)[:, :2]
        feat_counts[img.name] = int(len(kp))

    cam_by_name: dict[str, tuple] = {}
    for name, row in poses.items():
        r, t = traj_row_to_world_to_cam(row)
        cam_by_name[name] = (k, r, t)

    points_per_frame: dict[str, int] = defaultdict(int)
    pair_contrib: dict[str, int] = defaultdict(int)
    max_reproj = 6.0

    pair_ids = db.read_all_matches()[0]
    for pair_id in pair_ids:
        image_id2 = int(pair_id % 2147483648)
        image_id1 = int(pair_id // 2147483648)
        if image_id1 not in keypoints or image_id2 not in keypoints:
            continue
        name1 = images[image_id1].name
        name2 = images[image_id2].name
        if not pair_ok_names(name1, name2, n_frames, overlap, lap_stride):
            continue
        k1, r1, t1 = cam_by_name[name1]
        k2, r2, t2 = cam_by_name[name2]
        p1 = _projection_matrix(k1, r1, t1)
        p2 = _projection_matrix(k2, r2, t2)
        matches = np.asarray(db.read_matches(image_id1, image_id2), dtype=np.int64)
        n_pair_pts = 0
        for idx1, idx2 in matches:
            uv1 = keypoints[image_id1][int(idx1)]
            uv2 = keypoints[image_id2][int(idx2)]
            xyz = cv2_triangulate(p1, p2, uv1, uv2)
            if xyz is None:
                continue
            if _reproj_err(xyz, k1, r1, t1, uv1) > max_reproj:
                continue
            if _reproj_err(xyz, k2, r2, t2, uv2) > max_reproj:
                continue
            n_pair_pts += 1
            points_per_frame[name1] += 1
            points_per_frame[name2] += 1
        if n_pair_pts:
            pair_contrib[f"{name1}+{name2}"] = n_pair_pts

    rows = []
    for name in names:
        row = poses[name]
        pos = np.asarray(row["pos"], dtype=np.float64)
        dist = float(np.linalg.norm(pos - centroid))
        bearing = math.degrees(math.atan2(pos[1] - centroid[1], pos[0] - centroid[0]))
        alt = float(pos[2])
        lap = "L0" if _frame_index(name) < n_frames // 2 else "L1"
        rows.append(
            {
                "frame": name,
                "waypoint": row.get("waypoint", ""),
                "lap": lap,
                "dist_m": round(dist, 1),
                "bearing_deg": round(bearing, 1),
                "alt_m": round(alt, 1),
                "sift_features": feat_counts.get(name, 0),
                "tri_points": points_per_frame.get(name, 0),
            }
        )

    tri = [r["tri_points"] for r in rows]
    feat = [r["sift_features"] for r in rows]
    report = {
        "n_frames": n_frames,
        "total_tri_point_obs": int(sum(tri)),
        "tri_points_per_frame": {
            "min": int(min(tri)),
            "max": int(max(tri)),
            "mean": round(float(np.mean(tri)), 1),
            "std": round(float(np.std(tri)), 1),
            "ratio_max_min": round(max(tri) / max(min(tri), 1), 2),
        },
        "sift_per_frame": {
            "min": int(min(feat)),
            "max": int(max(feat)),
            "mean": round(float(np.mean(feat)), 1),
        },
        "by_lap": {
            lap: {
                "mean_tri": round(float(np.mean([r["tri_points"] for r in rows if r["lap"] == lap])), 1),
                "mean_sift": round(float(np.mean([r["sift_features"] for r in rows if r["lap"] == lap])), 1),
            }
            for lap in ("L0", "L1")
        },
        "sparse_frames_tri_lt_100": [r["frame"] for r in rows if r["tri_points"] < 100],
        "frames": rows,
    }

    out_path = workspace / "density_report.json"
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "frames"}, indent=2))
    print(f"full report -> {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
