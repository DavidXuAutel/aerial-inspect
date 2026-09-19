"""AirSim depth clearance measurement and pose repair."""
from __future__ import annotations

import math
import time
from typing import Tuple

import numpy as np

Pose4 = Tuple[float, float, float, float]


def yaw_toward(from_xy: tuple[float, float], to_xy: tuple[float, float]) -> float:
    dx, dy = to_xy[0] - from_xy[0], to_xy[1] - from_xy[1]
    return float(math.atan2(dy, dx))


def frame_clearance_m(depth: np.ndarray) -> float:
    h, _w = depth.shape
    roi = depth[int(h * 0.12) :, :]
    pv = roi[(roi > 0.8) & (roi < 80.0)]
    return float(np.min(pv)) if pv.size else 999.0


def measure_pose_clearance(
    client,
    vk: dict,
    x: float,
    y: float,
    z: float,
    yaw: float,
    settle_s: float = 0.35,
) -> tuple[bool, dict]:
    import airsim

    pose = airsim.Pose(
        airsim.Vector3r(x, y, -z),
        airsim.to_quaternion(0.0, 0.0, yaw),
    )
    client.simPause(True)
    client.simSetVehiclePose(pose, True, **vk)
    time.sleep(settle_s)
    client.simPause(False)
    time.sleep(0.05)

    pos = client.simGetVehiclePose(**vk).position
    ax, ay, az = float(pos.x_val), float(pos.y_val), -float(pos.z_val)
    xy_err = math.hypot(ax - x, ay - y)
    z_err = abs(az - z)

    resp = client.simGetImages(
        [airsim.ImageRequest("front_custom", airsim.ImageType.DepthPlanar, True, False)],
        **vk,
    )[0]
    depth = np.array(resp.image_data_float, dtype=np.float32).reshape(resp.height, resp.width)
    min_clear = frame_clearance_m(depth)
    meta = {
        "xy_err": round(xy_err, 2),
        "z_err": round(z_err, 2),
        "min_clear_m": round(min_clear, 2),
    }
    return xy_err <= 1.0 and z_err <= 4.0, meta


def pose_clearance_ok(
    client,
    vk: dict,
    x: float,
    y: float,
    z: float,
    yaw: float,
    clearance_m: float,
    settle_s: float = 0.35,
) -> tuple[bool, dict]:
    ok_pose, meta = measure_pose_clearance(client, vk, x, y, z, yaw, settle_s=settle_s)
    ok = ok_pose and float(meta["min_clear_m"]) >= clearance_m
    return ok, meta


def repair_pose_clearance(
    client,
    vk: dict,
    x: float,
    y: float,
    z: float,
    yaw: float,
    clearance_m: float,
    center_xy: tuple[float, float],
    max_push_m: float = 40.0,
) -> tuple[float, float, float, float, dict, bool]:
    """Push outward / sideways until depth clearance is met."""
    ok, meta = pose_clearance_ok(client, vk, x, y, z, yaw, clearance_m, settle_s=0.3)
    if ok:
        return x, y, z, yaw, meta, True

    cx, cy = center_xy
    dx, dy = x - cx, y - cy
    r = math.hypot(dx, dy)
    best = (x, y, z, yaw, meta, False)
    best_clear = float(meta.get("min_clear_m", 0.0))

    if r > 1.0:
        ux, uy = dx / r, dy / r
        for extra in np.arange(2.0, max_push_m + 1.0, 2.0):
            nx = cx + ux * (r + float(extra))
            ny = cy + uy * (r + float(extra))
            nyaw = yaw_toward((nx, ny), center_xy)
            ok, meta = pose_clearance_ok(client, vk, nx, ny, z, nyaw, clearance_m, settle_s=0.25)
            mc = float(meta.get("min_clear_m", 0.0))
            if mc > best_clear:
                best_clear = mc
                best = (nx, ny, z, nyaw, meta, ok)
            if ok:
                return nx, ny, z, nyaw, meta, True

        tx, ty = -uy, ux
        for lat in (4.0, -4.0, 8.0, -8.0, 12.0, -12.0, 16.0, -16.0, 20.0, -20.0):
            nx = x + tx * lat
            ny = y + ty * lat
            nyaw = yaw_toward((nx, ny), center_xy)
            ok, meta = pose_clearance_ok(client, vk, nx, ny, z, nyaw, clearance_m, settle_s=0.25)
            mc = float(meta.get("min_clear_m", 0.0))
            if mc > best_clear:
                best_clear = mc
                best = (nx, ny, z, nyaw, meta, ok)
            if ok:
                return nx, ny, z, nyaw, meta, True

    bx, by, bz, byaw, bmeta, bok = best
    return bx, by, bz, byaw, bmeta, bok


def repair_segment_pose(
    client,
    vk: dict,
    x0: float,
    y0: float,
    z0: float,
    x1: float,
    y1: float,
    z1: float,
    t: float,
    clearance_m: float,
    center_xy: tuple[float, float],
) -> Pose4:
    """Sample along chord; if unsafe, walk back toward the start waypoint."""
    ix = x0 + (x1 - x0) * t
    iy = y0 + (y1 - y0) * t
    iz = z0 + (z1 - z0) * t
    yaw = yaw_toward((ix, iy), center_xy)
    ok, _meta = pose_clearance_ok(client, vk, ix, iy, iz, yaw, clearance_m, settle_s=0.25)
    if ok:
        return ix, iy, iz, yaw

    lo, hi = 0.0, t
    safe_pose = (x0, y0, z0, yaw_toward((x0, y0), center_xy))
    while hi - lo > 0.04:
        mid = (lo + hi) / 2.0
        mx = x0 + (x1 - x0) * mid
        my = y0 + (y1 - y0) * mid
        mz = z0 + (z1 - z0) * mid
        myaw = yaw_toward((mx, my), center_xy)
        ok, _ = pose_clearance_ok(client, vk, mx, my, mz, myaw, clearance_m, settle_s=0.2)
        if ok:
            safe_pose = (mx, my, mz, myaw)
            lo = mid
        else:
            hi = mid

    rx, ry, rz, ryaw, _meta, rok = repair_pose_clearance(
        client, vk, safe_pose[0], safe_pose[1], safe_pose[2], safe_pose[3],
        clearance_m, center_xy,
    )
    return rx, ry, rz, ryaw
