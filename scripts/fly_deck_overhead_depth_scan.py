#!/usr/bin/env python3
"""Overhead along-deck DepthPlanar scan — densify the roadway itself.

Side-facade scans see cables well but under-sample the thin deck surface when
looking across mid-span. This script flies *above* the deck centerline looking
nearly nadir so DepthPlanar hits the roadway, then keeps only a height band.
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

from airsim_pose_utils import set_pose_verified  # noqa: E402
from colmap_known_pose import _intrinsics_from_hfov, _rotation_from_quat_wxyz  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--out-dir", required=True)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=41463)
    p.add_argument("--camera", default="0")
    p.add_argument("--vehicle", default="SimpleFlight")
    p.add_argument("--centroid-xy", nargs=2, type=float, required=True)
    p.add_argument("--span-axis-deg", type=float, required=True)
    p.add_argument("--along-min-m", type=float, default=-150.0)
    p.add_argument("--along-max-m", type=float, default=150.0)
    p.add_argument("--along-step-m", type=float, default=8.0)
    p.add_argument("--cross-offsets-m", nargs="+", type=float, default=[0.0, 8.0, -8.0])
    p.add_argument("--z", type=float, default=155.0, help="flight altitude (above deck)")
    p.add_argument("--pitch-deg", type=float, default=-75.0)
    p.add_argument("--yaw-along-deck", action="store_true",
                   help="yaw aligned with span (forward look); default = look across span")
    p.add_argument("--hfov-deg", type=float, default=70.0)
    p.add_argument("--stride-px", type=int, default=1)
    p.add_argument("--max-depth-m", type=float, default=120.0)
    p.add_argument("--min-depth-m", type=float, default=5.0)
    p.add_argument("--z-keep-min", type=float, default=90.0)
    p.add_argument("--z-keep-max", type=float, default=200.0)
    args = p.parse_args()

    import airsim

    out_dir = Path(args.out_dir).resolve()
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    traj_path = out_dir / "traj.jsonl"

    client = airsim.MultirotorClient(ip=args.host, port=args.port)
    client.confirmConnection()
    vk = {"vehicle_name": args.vehicle}
    client.enableApiControl(True, **vk)

    cx, cy = args.centroid_xy
    rad = math.radians(args.span_axis_deg)
    ux, uy = math.cos(rad), math.sin(rad)
    # look across span (same convention as facade scan) unless --yaw-along-deck
    if args.yaw_along_deck:
        yaw_deg = math.degrees(math.atan2(uy, ux))
    else:
        yaw_deg = math.degrees(math.atan2(uy, ux)) - 90.0

    alongs = np.arange(args.along_min_m, args.along_max_m + 1e-6, args.along_step_m)
    stations = []
    for along in alongs:
        for cross in args.cross_offsets_m:
            stations.append((float(along), float(cross)))

    width = height = None
    fx = fy = cx_px = cy_px = None
    all_pts: list[np.ndarray] = []
    all_cols: list[np.ndarray] = []
    rows: list[dict] = []
    t0 = time.time()

    for i, (along, cross) in enumerate(stations):
        x = cx + along * ux - cross * uy
        y = cy + along * uy + cross * ux
        yaw = math.radians(yaw_deg)
        pitch = math.radians(args.pitch_deg)
        try:
            ax, ay, az, ayaw, apitch, quat = set_pose_verified(
                client, vk, x, y, args.z, yaw, pitch=pitch
            )
        except RuntimeError as exc:
            print(f"WARN station {i}: {exc} — skip", flush=True)
            continue

        rgb_req = airsim.ImageRequest(args.camera, airsim.ImageType.Scene, False, True)
        depth_req = airsim.ImageRequest(args.camera, airsim.ImageType.DepthPlanar, True, False)
        resp_rgb, resp_depth = client.simGetImages([rgb_req, depth_req], **vk)
        if resp_depth.width == 0 or resp_depth.height == 0:
            print(f"WARN station {i}: empty depth — skip", flush=True)
            continue

        if width is None:
            width, height = resp_depth.width, resp_depth.height
            fx, fy, cx_px, cy_px = _intrinsics_from_hfov(width, height, args.hfov_deg)
            print(f"depth {width}x{height} fx={fx:.1f}", flush=True)

        depth = np.asarray(resp_depth.image_data_float, dtype=np.float32).reshape(height, width)
        name = f"oh_{i:04d}_a{along:+.0f}_c{cross:+.0f}.jpg"
        raw = bytes(resp_rgb.image_data_uint8)
        rgb = None
        if raw:
            (frames_dir / name).write_bytes(raw)
            from PIL import Image
            import io

            rgb_img = Image.open(io.BytesIO(raw)).convert("RGB")
            if rgb_img.size != (width, height):
                rgb_img = rgb_img.resize((width, height))
            rgb = np.asarray(rgb_img)

        stride = max(1, int(args.stride_px))
        us = np.arange(0, width, stride)
        vs = np.arange(0, height, stride)
        uu, vv = np.meshgrid(us, vs)
        d = depth[vv, uu]
        valid = (d >= args.min_depth_m) & (d <= args.max_depth_m) & np.isfinite(d)
        uu_v, vv_v, d_v = uu[valid], vv[valid], d[valid]

        n_pts = 0
        if uu_v.size > 0:
            xc = (uu_v.astype(np.float64) - cx_px) / fx * d_v.astype(np.float64)
            yc = (vv_v.astype(np.float64) - cy_px) / fy * d_v.astype(np.float64)
            zc = d_v.astype(np.float64)
            pts_cam = np.stack([xc, yc, zc], axis=1)
            r = _rotation_from_quat_wxyz(quat)
            cam_c = np.array([ax, ay, az], dtype=np.float64)
            pts_world = pts_cam @ r + cam_c
            z_ok = (pts_world[:, 2] >= args.z_keep_min) & (pts_world[:, 2] <= args.z_keep_max)
            pts_world = pts_world[z_ok]
            uu_v, vv_v = uu_v[z_ok], vv_v[z_ok]
            n_pts = pts_world.shape[0]
            if n_pts > 0:
                if rgb is not None:
                    cols = rgb[vv_v, uu_v]
                else:
                    cols = np.full((n_pts, 3), 180, dtype=np.uint8)
                all_pts.append(pts_world)
                all_cols.append(cols)

        rows.append(
            {
                "frame": name,
                "along_m": along,
                "cross_m": cross,
                "pos": [ax, ay, az],
                "yaw_rad": ayaw,
                "pitch_rad": apitch,
                "n_pts": n_pts,
            }
        )
        print(f"[{i + 1}/{len(stations)}] along={along:6.0f} cross={cross:+5.0f} -> +{n_pts} pts", flush=True)

    with traj_path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row) + "\n")

    if not all_pts:
        raise SystemExit("no points collected — check poses / z-band")

    pts = np.concatenate(all_pts, axis=0)
    cols = np.concatenate(all_cols, axis=0).astype(np.uint8)
    ply_path = out_dir / "overhead_deck_points.ply"
    with ply_path.open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for (x, y, z), (r8, g8, b8) in zip(pts, cols):
            f.write(f"{x:.4f} {y:.4f} {z:.4f} {int(r8)} {int(g8)} {int(b8)}\n")

    report = {
        "status": "ok",
        "n_points": int(len(pts)),
        "n_stations": len(stations),
        "n_stations_used": len(all_pts),
        "elapsed_s": round(time.time() - t0, 1),
        "ply": str(ply_path),
        "z_range": [float(pts[:, 2].min()), float(pts[:, 2].max())],
        "x_range": [float(pts[:, 0].min()), float(pts[:, 0].max())],
        "y_range": [float(pts[:, 1].min()), float(pts[:, 1].max())],
    }
    (out_dir / "overhead_deck_report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
