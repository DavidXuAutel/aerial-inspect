#!/usr/bin/env python3
"""Fly a dedicated along-deck depth scan to cover the FULL bridge length.

The original survey only orbits the mid-span/tower area (facade arc), so a
depth-cloud built from those 26 frames only has reliable near-field geometry
close to the tower — beyond ~150-200m along the span axis the depth returns
are dominated by background clutter (hills, riverbank), not the deck.

This script "walks" the exact same successful near/far-facade camera template
(cross-track standoff, altitude, yaw, pitch — extracted from the frames that
produced the densest/cleanest near-field points in the original survey) along
the full span axis at regular intervals, capturing DepthPlanar + RGB at each
station and back-projecting to world XYZ. Two passes (near + far facade side)
give dual-side coverage, same as the original survey design.
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
    p.add_argument("--out-dir", required=True, help="output dir (frames/ + traj.jsonl written here)")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=41463)
    p.add_argument("--camera", default="0")
    p.add_argument("--vehicle", default="SimpleFlight")
    p.add_argument("--centroid-xy", nargs=2, type=float, default=[2400.0, -20.0])
    p.add_argument("--span-axis-deg", type=float, default=26.0)
    p.add_argument("--hfov-deg", type=float, default=70.0)
    p.add_argument("--along-min-m", type=float, default=-620.0)
    p.add_argument("--along-max-m", type=float, default=620.0)
    p.add_argument("--along-step-m", type=float, default=25.0)
    # (cross_offset_m, yaw_deg, z, pitch_deg) per facade side — from frame_0007/frame_0019 templates.
    p.add_argument("--near-cross-m", type=float, default=50.0)
    p.add_argument("--near-z", type=float, default=83.3)
    p.add_argument("--near-pitch-deg", type=float, default=-15.8)
    p.add_argument("--far-cross-m", type=float, default=-60.0)
    p.add_argument("--far-z", type=float, default=88.3)
    p.add_argument("--far-pitch-deg", type=float, default=-17.7)
    p.add_argument("--stride-px", type=int, default=1)
    p.add_argument("--max-depth-m", type=float, default=220.0, help="hard cap (also used as fallback / valid-range ceiling)")
    p.add_argument("--min-depth-m", type=float, default=2.0)
    p.add_argument(
        "--adaptive-near-field",
        action="store_true",
        help="per-station: keep only the nearest coherent cluster (percentile-based), "
        "instead of a fixed global --max-depth-m. Robust to unknown/varying target distance.",
    )
    p.add_argument("--near-percentile", type=float, default=15.0)
    p.add_argument("--near-margin-m", type=float, default=30.0)
    p.add_argument("--near-margin-ratio", type=float, default=1.3)
    p.add_argument("--min-valid-px", type=int, default=200, help="skip station if fewer valid depth px")
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
    span_yaw = math.degrees(math.atan2(uy, ux))

    stations = []
    alongs = np.arange(args.along_min_m, args.along_max_m + 1e-6, args.along_step_m)
    for along in alongs:
        stations.append((float(along), args.near_cross_m, span_yaw - 90.0, args.near_z, args.near_pitch_deg, "near"))
    for along in alongs:
        stations.append((float(along), args.far_cross_m, span_yaw + 90.0, args.far_z, args.far_pitch_deg, "far"))

    width = height = None
    fx = fy = cx_px = cy_px = None
    all_pts: list[np.ndarray] = []
    all_cols: list[np.ndarray] = []
    rows: list[dict] = []

    t0 = time.time()
    for i, (along, cross, yaw_deg, z, pitch_deg, side) in enumerate(stations):
        x = cx + along * ux - cross * uy
        y = cy + along * uy + cross * ux
        yaw = math.radians(yaw_deg)
        pitch = math.radians(pitch_deg)
        try:
            ax, ay, az, ayaw, apitch, quat = set_pose_verified(client, vk, x, y, z, yaw, pitch=pitch)
        except RuntimeError as exc:
            print(f"WARN station {i} ({side} along={along:.0f}): {exc} — skip", flush=True)
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
            print(f"depth image {width}x{height} fx={fx:.1f} fy={fy:.1f}", flush=True)

        depth = np.asarray(resp_depth.image_data_float, dtype=np.float32).reshape(height, width)

        name = f"deck_{side}_{i:04d}.jpg"
        raw = bytes(resp_rgb.image_data_uint8)
        if raw:
            (frames_dir / name).write_bytes(raw)
            from PIL import Image
            import io

            rgb_img = Image.open(io.BytesIO(raw)).convert("RGB")
            if rgb_img.size != (width, height):
                rgb_img = rgb_img.resize((width, height))
            rgb = np.asarray(rgb_img)
        else:
            rgb = None

        stride = max(1, int(args.stride_px))
        us = np.arange(0, width, stride)
        vs = np.arange(0, height, stride)
        uu, vv = np.meshgrid(us, vs)
        d = depth[vv, uu]
        valid = (d >= args.min_depth_m) & (d <= args.max_depth_m) & np.isfinite(d)
        uu_v, vv_v, d_v = uu[valid], vv[valid], d[valid]

        if args.adaptive_near_field and uu_v.size >= args.min_valid_px:
            d_near = float(np.percentile(d_v, args.near_percentile))
            cutoff = min(d_near * args.near_margin_ratio + args.near_margin_m, args.max_depth_m)
            keep2 = d_v <= cutoff
            uu_v, vv_v, d_v = uu_v[keep2], vv_v[keep2], d_v[keep2]
        elif args.adaptive_near_field:
            uu_v, vv_v, d_v = uu_v[:0], vv_v[:0], d_v[:0]

        n_pts = 0
        if uu_v.size > 0:
            xc = (uu_v.astype(np.float64) - cx_px) / fx * d_v.astype(np.float64)
            yc = (vv_v.astype(np.float64) - cy_px) / fy * d_v.astype(np.float64)
            zc = d_v.astype(np.float64)
            pts_cam = np.stack([xc, yc, zc], axis=1)
            r = _rotation_from_quat_wxyz(quat)
            cam_c = np.array([ax, ay, az], dtype=np.float64)
            pts_world = pts_cam @ r + cam_c
            n_pts = pts_world.shape[0]
            if rgb is not None:
                cols = rgb[vv_v, uu_v]
            else:
                cols = np.full((n_pts, 3), 180, dtype=np.uint8)
            all_pts.append(pts_world)
            all_cols.append(cols)

        rows.append(
            {
                "frame": name,
                "side": side,
                "along_m": along,
                "cross_m": cross,
                "pos": [ax, ay, az],
                "yaw_rad": ayaw,
                "pitch_rad": apitch,
                "quat_wxyz": list(quat),
            }
        )
        print(f"[{i + 1}/{len(stations)}] {side} along={along:6.0f} -> +{n_pts} pts", flush=True)

    with traj_path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row) + "\n")

    if not all_pts:
        raise SystemExit("no points collected")

    pts = np.concatenate(all_pts, axis=0)
    cols = np.concatenate(all_cols, axis=0).astype(np.uint8)
    ply_path = out_dir / "deck_length_points.ply"
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
    }
    (out_dir / "deck_length_report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
