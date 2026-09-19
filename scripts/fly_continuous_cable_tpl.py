#!/usr/bin/env python3
"""Midspan cable densify with FIXED pitch templates (no look-at sheet).

Previous look-at pass flooded a planar sheet at z≈160–168. This mirrors the
proven continuous deck/cable templates, dual-side, midspan stations only.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import airsim
import numpy as np
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from airsim_pose_utils import set_pose_verified  # noqa: E402
from colmap_known_pose import _intrinsics_from_hfov, _rotation_from_quat_wxyz  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="artifacts/bridge_humen_001/deck_scan_cable_tpl")
    ap.add_argument("--x0", type=float, default=2580.0)
    ap.add_argument("--x1", type=float, default=2960.0)
    args = ap.parse_args()

    xy_all = np.asarray(json.loads(Path("/tmp/humen_centerline.json").read_text()), dtype=np.float64)
    mask = (xy_all[:, 0] >= args.x0) & (xy_all[:, 0] <= args.x1)
    xy = xy_all[mask]
    tang = np.gradient(xy, axis=0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-9
    perp = np.stack([-tang[:, 1], tang[:, 0]], axis=1)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    client = airsim.MultirotorClient(ip="127.0.0.1", port=41463)
    client.confirmConnection()
    vk = {"vehicle_name": "SimpleFlight"}
    client.enableApiControl(True, **vk)

    # (name, cross, fly_z, pitch_deg, zlo, zhi)
    templates = [
        ("cable_lo", 48.0, 88.0, -32.0, 145.0, 158.0),
        ("cable_hi", 52.0, 92.0, -22.0, 152.0, 168.0),
    ]
    sides = (1.0, -1.0)
    corridor_m = 30.0

    all_pts: list[np.ndarray] = []
    all_cols: list[np.ndarray] = []
    fx = fy = cpx = cpy = None
    t0 = time.time()
    total = len(xy) * len(sides) * len(templates)
    k = 0

    for i, (p, t, nrm) in enumerate(zip(xy, tang, perp)):
        yaw_base = math.degrees(math.atan2(t[1], t[0]))
        for side in sides:
            yaw_deg = yaw_base - 90.0 if side > 0 else yaw_base + 90.0
            for name, cross, fly_z, pitch_deg, zlo, zhi in templates:
                k += 1
                pos = p + side * cross * nrm
                try:
                    ax, ay, az, _, _, quat = set_pose_verified(
                        client,
                        vk,
                        float(pos[0]),
                        float(pos[1]),
                        fly_z,
                        math.radians(yaw_deg),
                        pitch=math.radians(pitch_deg),
                        tol_xy=6.0,
                        tol_z=10.0,
                    )
                except RuntimeError as exc:
                    print(f"[{k}/{total}] skip {exc}", flush=True)
                    continue

                rgb, dep = client.simGetImages(
                    [
                        airsim.ImageRequest("0", airsim.ImageType.Scene, False, True),
                        airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, True, False),
                    ],
                    **vk,
                )
                if dep.width == 0:
                    continue
                d = np.asarray(dep.image_data_float, dtype=np.float32).reshape(dep.height, dep.width)
                if fx is None:
                    fx, fy, cpx, cpy = _intrinsics_from_hfov(dep.width, dep.height, 70.0)
                uu, vv = np.meshgrid(np.arange(0, dep.width, 2), np.arange(0, dep.height, 2))
                dd = d[vv, uu]
                m = (dd >= 2) & (dd <= 140) & np.isfinite(dd)
                npt = 0
                if m.any():
                    cam = np.stack(
                        [(uu[m] - cpx) / fx * dd[m], (vv[m] - cpy) / fy * dd[m], dd[m]], 1
                    )
                    pw = cam @ _rotation_from_quat_wxyz(quat) + np.array([ax, ay, az])
                    cross_dist = np.abs((pw[:, :2] - p[None, :]) @ nrm)
                    keep = (pw[:, 2] >= zlo) & (pw[:, 2] < zhi) & (cross_dist <= corridor_m)
                    pw = pw[keep]
                    npt = len(pw)
                    if npt:
                        cols = np.full((npt, 3), 210, np.uint8)
                        raw = bytes(rgb.image_data_uint8)
                        if raw:
                            from PIL import Image
                            import io

                            im = np.asarray(Image.open(io.BytesIO(raw)).convert("RGB"))
                            if im.shape[1] != dep.width:
                                im = np.asarray(Image.fromarray(im).resize((dep.width, dep.height)))
                            cols = im[vv[m][keep], uu[m][keep]]
                        all_pts.append(pw)
                        all_cols.append(cols)
                print(f"[{k}/{total}] i={i} x={p[0]:.0f} side={int(side):+d} {name} +{npt}", flush=True)

    if not all_pts:
        raise SystemExit("no points")
    pts = np.concatenate(all_pts)
    cols = np.concatenate(all_cols).astype(np.uint8)
    ply = out / "cable_tpl_points.ply"
    with ply.open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        for (x, y, z), (r, g, b) in zip(pts, cols):
            f.write(f"{x:.3f} {y:.3f} {z:.3f} {int(r)} {int(g)} {int(b)}\n")
    report = {
        "status": "ok",
        "n_points": int(len(pts)),
        "n_stations": int(len(xy)),
        "elapsed_s": round(time.time() - t0, 1),
        "mode": "continuous_cable_template_midspan",
        "ply": str(ply),
    }
    (out / "cable_tpl_report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
