#!/usr/bin/env python3
"""ONE continuous full-bridge depth scan for later reconstruction.

- Single pass along the whole main span (not gap patches)
- Dual facade at every station
- Fixed-pitch stack for underside/deck/mid/cable_lo
- Look-at toward a parabolic main-cable height for cable_hi/peak/tower
  (corridor-clipped so the old midspan sheet does not return)
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
from airsim_pose_utils import set_pose_verified, look_at_yaw_pitch  # noqa: E402
from colmap_known_pose import _intrinsics_from_hfov, _rotation_from_quat_wxyz  # noqa: E402


def densify_polyline(xy: np.ndarray, step_m: float) -> np.ndarray:
    if len(xy) < 2:
        return xy
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    if total < step_m:
        return xy
    samples = np.arange(0.0, total + 1e-6, step_m)
    if samples[-1] < total:
        samples = np.append(samples, total)
    out = []
    for t in samples:
        i = int(np.searchsorted(s, t, side="right") - 1)
        i = max(0, min(i, len(xy) - 2))
        ds = s[i + 1] - s[i]
        a = 0.0 if ds < 1e-9 else (t - s[i]) / ds
        out.append(xy[i] * (1 - a) + xy[i + 1] * a)
    return np.asarray(out, dtype=np.float64)


def write_ply(path: Path, pts: np.ndarray, cols: np.ndarray) -> None:
    with path.open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        for (x, y, z), (r, g, b) in zip(pts, cols):
            f.write(f"{x:.3f} {y:.3f} {z:.3f} {int(r)} {int(g)} {int(b)}\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="artifacts/bridge_humen_001/deck_scan_complete")
    ap.add_argument("--step-m", type=float, default=5.0)
    ap.add_argument(
        "--centerline",
        default="configs/sim/humen_centerline.json",
        help="Main-channel deck centerline (repo path or /tmp override)",
    )
    args = ap.parse_args()

    cl_path = Path(args.centerline)
    if not cl_path.is_file() and Path("/tmp/humen_centerline.json").is_file():
        cl_path = Path("/tmp/humen_centerline.json")
    raw = json.loads(cl_path.read_text())
    if isinstance(raw, dict):
        raw = raw.get("points_xy") or raw.get("centerline") or raw.get("xy")
    xy0 = np.asarray(raw, dtype=np.float64)
    xy = densify_polyline(xy0, float(args.step_m))
    tang = np.gradient(xy, axis=0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-9
    perp = np.stack([-tang[:, 1], tang[:, 0]], axis=1)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    client = airsim.MultirotorClient(ip="127.0.0.1", port=41463)
    client.confirmConnection()
    vk = {"vehicle_name": "SimpleFlight"}
    client.enableApiControl(True, **vk)

    # Fixed-pitch stack (proven).
    templates = [
        ("underside", 35.0, 70.0, -55.0, 118.0, 130.0, 0.0),
        ("deck", 40.0, 78.0, -48.0, 120.0, 142.0, 0.0),
        ("mid", 45.0, 86.0, -32.0, 132.0, 150.0, 0.0),
        ("cable_lo", 50.0, 83.3, -28.0, 136.0, 158.0, 0.0),
    ]
    # Look-at toward parabolic cable; corridor rejects sheets.
    # v6: midspan aim was 156 → z window topped ~168 and clipped the real
    # upper catenary (tower-side envelope continues ~170–176 into midspan).
    lookat_tpls = [
        ("cable_hi", 48.0, 92.0, 14.0, 24.0),
        ("cable_peak", 45.0, 100.0, 16.0, 22.0),
        ("cable_top", 42.0, 108.0, 14.0, 20.0),
    ]
    tower_look = ("tower", 42.0, 115.0, 14.0, 35.0)

    x_min, x_max = float(xy[:, 0].min()), float(xy[:, 0].max())
    tower_west = x_min + 90.0
    tower_east = x_max - 90.0
    x_mid = 0.5 * (tower_west + tower_east)
    half_span = 0.5 * (tower_east - tower_west)
    z_cable_mid = 168.0
    z_cable_tower = 188.0

    def cable_z_at(x: float) -> float:
        t = (x - x_mid) / max(half_span, 1.0)
        t = max(-1.0, min(1.0, t))
        return z_cable_mid + (z_cable_tower - z_cable_mid) * (t * t)

    sides = (1.0, -1.0)
    all_pts: list[np.ndarray] = []
    all_cols: list[np.ndarray] = []
    fx = fy = cpx = cpy = None
    t0 = time.time()

    total = 0
    for p in xy:
        n_tpl = len(templates) + len(lookat_tpls)
        if p[0] <= tower_west or p[0] >= tower_east:
            n_tpl += 1
        total += n_tpl * len(sides)

    k = 0
    print(
        json.dumps(
            {
                "stations": int(len(xy)),
                "step_m": args.step_m,
                "templates": [t[0] for t in templates]
                + [t[0] for t in lookat_tpls]
                + ["tower@ends"],
                "total_shots": total,
            }
        ),
        flush=True,
    )

    def capture(pos_xy, fly_z, yaw_rad, pitch_rad, zlo, zhi, corridor, p, nrm, label, i, side):
        nonlocal k, fx, fy, cpx, cpy
        k += 1
        try:
            ax, ay, az, _, _, quat = set_pose_verified(
                client,
                vk,
                float(pos_xy[0]),
                float(pos_xy[1]),
                fly_z,
                yaw_rad,
                pitch=pitch_rad,
                tol_xy=6.0,
                tol_z=10.0,
            )
        except RuntimeError as exc:
            print(f"[{k}/{total}] skip {exc}", flush=True)
            return
        rgb, dep = client.simGetImages(
            [
                airsim.ImageRequest("0", airsim.ImageType.Scene, False, True),
                airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, True, False),
            ],
            **vk,
        )
        if dep.width == 0:
            return
        d = np.asarray(dep.image_data_float, dtype=np.float32).reshape(dep.height, dep.width)
        if fx is None:
            fx, fy, cpx, cpy = _intrinsics_from_hfov(dep.width, dep.height, 70.0)
        uu, vv = np.meshgrid(np.arange(0, dep.width, 2), np.arange(0, dep.height, 2))
        dd = d[vv, uu]
        m = (dd >= 2) & (dd <= 160) & np.isfinite(dd)
        npt = 0
        if m.any():
            cam = np.stack(
                [(uu[m] - cpx) / fx * dd[m], (vv[m] - cpy) / fy * dd[m], dd[m]], 1
            )
            pw = cam @ _rotation_from_quat_wxyz(quat) + np.array([ax, ay, az])
            keep = (pw[:, 2] >= zlo) & (pw[:, 2] < zhi)
            if corridor > 0:
                cross_dist = np.abs((pw[:, :2] - p[None, :]) @ nrm)
                keep = keep & (cross_dist <= corridor)
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
        print(
            f"[{k}/{total}] i={i}/{len(xy)} x={p[0]:.0f} side={int(side):+d} {label} +{npt}",
            flush=True,
        )

    for i, (p, t, nrm) in enumerate(zip(xy, tang, perp)):
        yaw_base = math.degrees(math.atan2(t[1], t[0]))
        for side in sides:
            yaw_deg = yaw_base - 90.0 if side > 0 else yaw_base + 90.0
            for name, cross, fly_z, pitch_deg, zlo, zhi, corridor in templates:
                pos = p + side * cross * nrm
                capture(
                    pos,
                    fly_z,
                    math.radians(yaw_deg),
                    math.radians(pitch_deg),
                    zlo,
                    zhi,
                    corridor,
                    p,
                    nrm,
                    name,
                    i,
                    side,
                )

            aim_z = cable_z_at(float(p[0]))
            for name, cross, fly_z, z_half, corridor in lookat_tpls:
                pos = p + side * cross * nrm
                pos3 = np.array([pos[0], pos[1], fly_z])
                aim = np.array([p[0], p[1], aim_z])
                yaw, pitch = look_at_yaw_pitch(pos3, aim)
                capture(
                    pos,
                    fly_z,
                    yaw,
                    -pitch,
                    aim_z - z_half,
                    aim_z + z_half,
                    corridor,
                    p,
                    nrm,
                    f"{name}@z{aim_z:.0f}",
                    i,
                    side,
                )

            if p[0] <= tower_west or p[0] >= tower_east:
                name, cross, fly_z, z_half, corridor = tower_look
                pos = p + side * cross * nrm
                pos3 = np.array([pos[0], pos[1], fly_z])
                aim = np.array([p[0], p[1], z_cable_tower])
                yaw, pitch = look_at_yaw_pitch(pos3, aim)
                capture(
                    pos,
                    fly_z,
                    yaw,
                    -pitch,
                    z_cable_tower - z_half,
                    z_cable_tower + z_half,
                    corridor,
                    p,
                    nrm,
                    name,
                    i,
                    side,
                )

        if (i + 1) % 20 == 0 and all_pts:
            pts_ck = np.concatenate(all_pts)
            cols_ck = np.concatenate(all_cols).astype(np.uint8)
            write_ply(out / "complete_points_ckpt.ply", pts_ck, cols_ck)
            print(f"checkpoint stations={i+1} pts={len(pts_ck)}", flush=True)

    if not all_pts:
        raise SystemExit("no points")
    pts = np.concatenate(all_pts)
    cols = np.concatenate(all_cols).astype(np.uint8)
    ply = out / "complete_points.ply"
    write_ply(ply, pts, cols)

    edges = np.arange(float(xy[:, 0].min()), float(xy[:, 0].max()) + 40.0, 40.0)
    bins = []
    for a, b in zip(edges[:-1], edges[1:]):
        m = (pts[:, 0] >= a) & (pts[:, 0] < b)
        bins.append(
            {
                "x0": float(a),
                "x1": float(b),
                "underside": int(((pts[:, 2] < 126) & m).sum()),
                "deck": int(((pts[:, 2] >= 124) & (pts[:, 2] < 135) & m).sum()),
                "mid": int(((pts[:, 2] >= 135) & (pts[:, 2] < 150) & m).sum()),
                "cable": int(((pts[:, 2] >= 148) & (pts[:, 2] < 165) & m).sum()),
                "crown": int(((pts[:, 2] >= 165) & (pts[:, 2] < 185) & m).sum()),
                "tower": int(((pts[:, 2] >= 185) & m).sum()),
            }
        )
    weak = [b for b in bins if b["deck"] < 3000 or b["cable"] < 400]
    report = {
        "status": "PASS" if not weak else "WEAK",
        "mode": "continuous_complete_full_bridge_v6",
        "n_points": int(len(pts)),
        "n_stations": int(len(xy)),
        "step_m": args.step_m,
        "elapsed_s": round(time.time() - t0, 1),
        "n_z_gt_165": int((pts[:, 2] > 165).sum()),
        "n_z_gt_175": int((pts[:, 2] > 175).sum()),
        "weak_bins": weak,
        "bins": bins,
        "ply": str(ply),
    }
    (out / "complete_report.json").write_text(json.dumps(report, indent=2))
    print(
        json.dumps(
            {
                k: report[k]
                for k in (
                    "status",
                    "n_points",
                    "n_z_gt_165",
                    "n_z_gt_175",
                    "elapsed_s",
                    "weak_bins",
                )
            },
            indent=2,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
