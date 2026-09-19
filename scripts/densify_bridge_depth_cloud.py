#!/usr/bin/env python3
"""Ground-truth dense point cloud from AirSim DepthPlanar (not COLMAP SfM).

The sparse COLMAP known-pose triangulation only keeps SIFT-feature points that
survive two-view geometry — for this scene (reflective water, thin cables) that
leaves a sparse, fragmented cloud dominated by background clutter, which does
NOT visually read as "a bridge".

This script instead re-teleports to each *already-verified* survey pose from
traj.jsonl and captures the simulator's own DepthPlanar buffer (ground truth,
no feature matching / no CUDA needed) alongside the RGB frame, then
back-projects every (subsampled) pixel to world XYZ using the known camera
pose + pinhole intrinsics. This produces an order of magnitude more points,
correctly covering the deck/towers/cables, colored from the real frame.

Must run on the host with AirSim network access (the sim VM itself).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from colmap_known_pose import _intrinsics_from_hfov, _rotation_from_quat_wxyz  # noqa: E402


def _load_traj(traj_path: Path) -> list[dict]:
    rows = []
    for line in traj_path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def main() -> int:
    p = argparse.ArgumentParser(description="Dense ground-truth point cloud from AirSim depth")
    p.add_argument("capture_dir", help="artifacts/<mission_id>/capture_survey (has traj.jsonl + frames/)")
    p.add_argument("--workspace", help="output workspace (default artifacts/models/<mission_id>)")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=41463)
    p.add_argument("--camera", default="0")
    p.add_argument("--vehicle", default="SimpleFlight")
    p.add_argument("--hfov-deg", type=float, default=0.0)
    p.add_argument("--stride-px", type=int, default=6, help="pixel subsample stride")
    p.add_argument("--max-depth-m", type=float, default=400.0, help="drop pixels farther than this")
    p.add_argument("--min-depth-m", type=float, default=2.0)
    p.add_argument(
        "--max-dist-from-centroid-m",
        type=float,
        default=220.0,
        help="drop world points farther than this from bridge_centroid_xyz (0=off)",
    )
    args = p.parse_args()

    capture = Path(args.capture_dir).resolve()
    traj_path = capture / "traj.jsonl"
    if not traj_path.is_file():
        raise SystemExit(f"missing {traj_path}")
    rows = _load_traj(traj_path)
    print(f"loaded {len(rows)} traj rows", flush=True)

    spec_path = capture.parent / "mission_spec.json"
    hfov = args.hfov_deg
    centroid_xyz = None
    if spec_path.is_file():
        spec = json.loads(spec_path.read_text(encoding="utf-8"))
        if hfov <= 0:
            hfov = float(spec.get("survey", {}).get("camera_hfov_deg", 70.0))
        centroid_xyz = spec.get("bridge_centroid_xyz")
    if hfov <= 0:
        hfov = 70.0

    workspace = Path(args.workspace).resolve() if args.workspace else (capture.parent.parent / "models" / capture.parent.name)
    workspace.mkdir(parents=True, exist_ok=True)

    import airsim

    client = airsim.MultirotorClient(ip=args.host, port=args.port)
    client.confirmConnection()
    vk = {"vehicle_name": args.vehicle}
    client.enableApiControl(True, **vk)

    from airsim_pose_utils import set_pose_verified

    stride = max(1, int(args.stride_px))
    all_pts: list[np.ndarray] = []
    all_cols: list[np.ndarray] = []
    width = height = None
    fx = fy = cx = cy = None

    t0 = time.time()
    for i, row in enumerate(rows):
        pos = row["pos"]
        yaw = float(row["yaw_rad"])
        pitch = float(row["pitch_rad"])
        try:
            ax, ay, az, _, _, _ = set_pose_verified(
                client, vk, float(pos[0]), float(pos[1]), float(pos[2]), yaw, pitch=pitch
            )
        except RuntimeError as exc:
            print(f"WARN frame {i}: {exc} — skipping", flush=True)
            continue

        depth_req = airsim.ImageRequest(args.camera, airsim.ImageType.DepthPlanar, True, False)
        resp = client.simGetImages([depth_req], **vk)[0]
        if resp.width == 0 or resp.height == 0:
            print(f"WARN frame {i}: empty depth response — skipping", flush=True)
            continue
        depth = np.asarray(resp.image_data_float, dtype=np.float32).reshape(resp.height, resp.width)

        if width is None:
            width, height = resp.width, resp.height
            fx, fy, cx, cy = _intrinsics_from_hfov(width, height, hfov)
            print(f"depth image {width}x{height}, hfov={hfov} -> fx={fx:.1f} fy={fy:.1f}", flush=True)

        frame_name = row.get("frame")
        rgb_path = capture / "frames" / frame_name if frame_name else None
        rgb = None
        if rgb_path and rgb_path.is_file():
            from PIL import Image

            rgb_img = Image.open(rgb_path).convert("RGB")
            if rgb_img.size != (width, height):
                rgb_img = rgb_img.resize((width, height))
            rgb = np.asarray(rgb_img)

        cam_pos = row.get("cam_pos") or [ax, ay, az]
        cam_quat = row.get("cam_quat_wxyz")
        if cam_quat:
            r_world_to_cam = _rotation_from_quat_wxyz(cam_quat)  # rows = (right, down, forward) in world
        else:
            print(f"WARN frame {i}: no cam_quat_wxyz — skipping", flush=True)
            continue
        cam_c = np.asarray(cam_pos[:3], dtype=np.float64)

        us = np.arange(0, width, stride)
        vs = np.arange(0, height, stride)
        uu, vv = np.meshgrid(us, vs)
        d = depth[vv, uu]
        valid = (d >= args.min_depth_m) & (d <= args.max_depth_m) & np.isfinite(d)
        uu_v, vv_v, d_v = uu[valid], vv[valid], d[valid]
        if uu_v.size == 0:
            print(f"[{i + 1}/{len(rows)}] {frame_name}: 0 valid depth px", flush=True)
            continue

        xc = (uu_v.astype(np.float64) - cx) / fx * d_v.astype(np.float64)
        yc = (vv_v.astype(np.float64) - cy) / fy * d_v.astype(np.float64)
        zc = d_v.astype(np.float64)
        pts_cam = np.stack([xc, yc, zc], axis=1)  # (N,3) in (right, down, forward) camera axes

        # world = R^T @ cam_vec + cam_center  (R rows = right/down/forward expressed in world)
        r = np.stack(
            [r_world_to_cam[0], r_world_to_cam[1], r_world_to_cam[2]], axis=0
        )  # (3,3): world_to_cam s.t. cam = R @ world_delta
        pts_world = pts_cam @ r + cam_c  # since cam = R @ (world - C) -> world = cam @ R + C (R orthonormal)

        if args.max_dist_from_centroid_m > 0 and centroid_xyz:
            c = np.asarray(centroid_xyz[:3], dtype=np.float64)
            dist = np.linalg.norm(pts_world - c, axis=1)
            keep = dist <= args.max_dist_from_centroid_m
            pts_world = pts_world[keep]
            uu_v, vv_v = uu_v[keep], vv_v[keep]

        if pts_world.shape[0] == 0:
            print(f"[{i + 1}/{len(rows)}] {frame_name}: 0 pts after centroid filter", flush=True)
            continue

        if rgb is not None:
            cols = rgb[vv_v, uu_v]
        else:
            cols = np.full((pts_world.shape[0], 3), 180, dtype=np.uint8)

        all_pts.append(pts_world)
        all_cols.append(cols)
        print(f"[{i + 1}/{len(rows)}] {frame_name}: +{pts_world.shape[0]} pts (depth range {d_v.min():.0f}-{d_v.max():.0f}m)", flush=True)

    if not all_pts:
        raise SystemExit("no points collected — check sim connection / poses")

    pts = np.concatenate(all_pts, axis=0)
    cols = np.concatenate(all_cols, axis=0).astype(np.uint8)
    print(f"total {len(pts)} dense points from {len(rows)} frames in {time.time() - t0:.1f}s", flush=True)

    ply_path = workspace / "dense_depth_points.ply"
    with ply_path.open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for (x, y, z), (r8, g8, b8) in zip(pts, cols):
            f.write(f"{x:.4f} {y:.4f} {z:.4f} {int(r8)} {int(g8)} {int(b8)}\n")
    print(f"wrote {ply_path} ({len(pts)} points)", flush=True)

    report = {
        "status": "ok",
        "n_points": int(len(pts)),
        "n_frames_used": int(len(all_pts)),
        "n_frames_total": int(len(rows)),
        "stride_px": stride,
        "max_depth_m": args.max_depth_m,
        "max_dist_from_centroid_m": args.max_dist_from_centroid_m,
        "hfov_deg": hfov,
        "ply": str(ply_path),
    }
    (workspace / "dense_depth_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
