#!/usr/bin/env python3
"""Continuous dual-side full-bridge depth scan → surface-ready cloud.

One pass along centerline. At each station: near+far × deck+cable templates.
This fills the missing opposite facade so Poisson meshing can form a closed shell.
"""
from __future__ import annotations

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
    center_path = Path("/tmp/humen_centerline.json")
    xy = np.asarray(json.loads(center_path.read_text()), dtype=np.float64)
    tang = np.gradient(xy, axis=0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-9
    perp = np.stack([-tang[:, 1], tang[:, 0]], axis=1)

    out = Path("artifacts/bridge_humen_001/deck_scan_dual")
    out.mkdir(parents=True, exist_ok=True)

    client = airsim.MultirotorClient(ip="127.0.0.1", port=41463)
    client.confirmConnection()
    vk = {"vehicle_name": "SimpleFlight"}
    client.enableApiControl(True, **vk)

    # (name, |cross|, z, pitch, zlo, zhi)
    templates = [
        ("deck", 40.0, 78.0, -48.0, 120.0, 142.0),
        ("cable", 50.0, 83.3, -28.0, 136.0, 156.0),
    ]
    sides = (1.0, -1.0)

    all_pts: list[np.ndarray] = []
    all_cols: list[np.ndarray] = []
    fx = fy = cpx = cpy = None
    t0 = time.time()
    total = len(xy) * len(templates) * len(sides)
    k = 0

    for i, (p, t, nrm) in enumerate(zip(xy, tang, perp)):
        yaw_base = math.degrees(math.atan2(t[1], t[0]))
        for side in sides:
            yaw_deg = yaw_base - 90.0 if side > 0 else yaw_base + 90.0
            for name, cross, z, pitch_deg, zlo, zhi in templates:
                k += 1
                pos = p + side * cross * nrm
                try:
                    ax, ay, az, _, _, quat = set_pose_verified(
                        client,
                        vk,
                        float(pos[0]),
                        float(pos[1]),
                        z,
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
                m = (dd >= 2) & (dd <= 150) & np.isfinite(dd)
                npt = 0
                if m.any():
                    cam = np.stack(
                        [(uu[m] - cpx) / fx * dd[m], (vv[m] - cpy) / fy * dd[m], dd[m]], 1
                    )
                    pw = cam @ _rotation_from_quat_wxyz(quat) + np.array([ax, ay, az])
                    keep = (pw[:, 2] >= zlo) & (pw[:, 2] < zhi)
                    # drop midspan false high from this pass already by zhi<=156
                    pw = pw[keep]
                    npt = len(pw)
                    if npt:
                        cols = np.full((npt, 3), 170, np.uint8)
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
                print(f"[{k}/{total}] i={i} side={int(side):+d} {name} +{npt}", flush=True)

    if not all_pts:
        raise SystemExit("no points")
    pts = np.concatenate(all_pts)
    cols = np.concatenate(all_cols).astype(np.uint8)
    ply = out / "dual_points.ply"
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
        "mode": "continuous_dual_side",
        "ply": str(ply),
    }
    (out / "dual_report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
