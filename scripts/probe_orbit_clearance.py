#!/usr/bin/env python3
"""Probe collision-safe freeform orbit waypoints in AirSim (radial + lateral search)."""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from aerial_inspect.mission.schema import MissionSpec
from aerial_inspect.survey.orbit_planner import plan_survey_from_xy
from path_clearance import pose_clearance_ok, repair_pose_clearance, yaw_toward


def _offset_xy(
    cx: float,
    cy: float,
    r: float,
    theta: float,
    lateral_m: float,
) -> tuple[float, float]:
    tx = -math.sin(theta)
    ty = math.cos(theta)
    x = cx + r * math.cos(theta) + lateral_m * tx
    y = cy + r * math.sin(theta) + lateral_m * ty
    return x, y


def _lateral_offsets(lateral_max: float, step: float = 4.0) -> list[float]:
    if lateral_max <= 0:
        return [0.0]
    offs = [0.0]
    k = 1
    while k * step <= lateral_max + 1e-6:
        offs.extend([-k * step, k * step])
        k += 1
    return offs


def _search_pose(
    client,
    vk: dict,
    cx: float,
    cy: float,
    alt: float,
    theta: float,
    r_min: float,
    r_max: float,
    lateral_max: float,
    clearance_m: float,
) -> tuple[float, float, float, float, dict]:
    best: tuple[float, float, float, float, dict] | None = None
    best_score = float("inf")
    r_hi = r_max + 100.0

    for lat in _lateral_offsets(lateral_max):
        lo, hi = r_min, r_hi
        found_r: float | None = None
        found_meta: dict = {}
        while hi - lo > 1.5:
            mid = (lo + hi) / 2.0
            x, y = _offset_xy(cx, cy, mid, theta, lat)
            yaw = yaw_toward((x, y), (cx, cy))
            ok, meta = pose_clearance_ok(client, vk, x, y, alt, yaw, clearance_m + 0.5)
            if ok:
                found_r = mid
                found_meta = meta
                hi = mid
            else:
                lo = mid
        if found_r is not None:
            score = found_r + abs(lat) * 0.35
            if score < best_score:
                best_score = score
                x, y = _offset_xy(cx, cy, found_r, theta, lat)
                best = (x, y, found_r, lat, found_meta)

    if best is not None:
        return best

    for lat in _lateral_offsets(lateral_max):
        x, y = _offset_xy(cx, cy, r_hi, theta, lat)
        yaw = yaw_toward((x, y), (cx, cy))
        rx, ry, _, ryaw, meta, ok = repair_pose_clearance(
            client, vk, x, y, alt, yaw, clearance_m, (cx, cy), max_push_m=120.0,
        )
        if ok:
            r = math.hypot(rx - cx, ry - cy)
            return rx, ry, r, lat, meta

    raise RuntimeError(
        f"no safe pose for theta={math.degrees(theta) % 360:.0f}° within r<={r_hi:.0f}m "
        f"lat<={lateral_max:.0f}m clearance>={clearance_m:.0f}m"
    )


def probe_waypoints(
    client,
    vk: dict,
    cx: float,
    cy: float,
    alt: float,
    n: int,
    r_min: float,
    r_max: float,
    lateral_max: float,
    clearance_m: float,
    start_deg: float,
    arc_deg: float = 360.0,
) -> list[dict]:
    rows: list[dict] = []
    start = math.radians(start_deg)
    arc = math.radians(arc_deg if arc_deg > 0 else 360.0)
    closed = arc_deg >= 360.0
    for i in range(n):
        if closed:
            theta = start + 2.0 * math.pi * i / n
        else:
            theta = start + arc * (i / max(n - 1, 1))
        x, y, r, lat, meta = _search_pose(
            client, vk, cx, cy, alt, theta, r_min, r_max, lateral_max, clearance_m,
        )
        yaw = yaw_toward((x, y), (cx, cy))
        ok, verify = pose_clearance_ok(client, vk, x, y, alt, yaw, clearance_m)
        if not ok:
            x, y, alt, yaw, verify, ok = repair_pose_clearance(
                client, vk, x, y, alt, yaw, clearance_m, (cx, cy), max_push_m=50.0,
            )
            r = math.hypot(x - cx, y - cy)
        if not ok:
            raise RuntimeError(
                f"waypoint {i} failed after repair: clear={verify.get('min_clear_m')}m"
            )
        rows.append(
            {
                "index": i,
                "theta_deg": round(math.degrees(theta) % 360.0, 1),
                "radius_m": round(r, 2),
                "lateral_m": round(lat, 2),
                "x": round(x, 2),
                "y": round(y, 2),
                "probe": verify,
            }
        )
        print(
            f"[{i + 1}/{n}] theta={rows[-1]['theta_deg']:.0f}° "
            f"r={r:.0f}m lat={lat:+.0f}m clear={verify.get('min_clear_m', '?')}m",
            flush=True,
        )
    return rows


def main() -> int:
    p = argparse.ArgumentParser(description="Probe collision-safe freeform orbit in AirSim")
    p.add_argument("mission_dir", help="artifacts/<mission_id>")
    p.add_argument("--config", help="mission yaml (default: read mission_spec.json)")
    p.add_argument("--host", default=os.environ.get("AIRSIM_HOST", "127.0.0.1"))
    p.add_argument("--port", type=int, default=int(os.environ.get("AIRSIM_PORT", "41451")))
    p.add_argument("--vehicle", default=os.environ.get("AIRSIM_VEHICLE", "drone_1"))
    args = p.parse_args()

    mission_dir = Path(args.mission_dir).resolve()
    if args.config:
        spec = MissionSpec.from_dict(yaml.safe_load(Path(args.config).read_text(encoding="utf-8")))
    else:
        spec = MissionSpec.from_dict(
            json.loads((mission_dir / "mission_spec.json").read_text(encoding="utf-8"))
        )
    if spec.bridge_centroid_xyz is None:
        raise SystemExit("bridge_centroid_xyz required")

    cx, cy, cz = spec.bridge_centroid_xyz
    sv = spec.survey
    n = max(8, int(sv.points_per_lap))
    r_min = float(sv.radius_m)
    r_max = float(sv.radius_max_m) if sv.radius_max_m > 0 else r_min + 18.0
    lateral_max = float(sv.lateral_max_m)
    clearance_m = float(sv.clearance_m)
    alt = float(sv.altitude_m)

    import airsim

    client = airsim.MultirotorClient(ip=args.host, port=args.port)
    client.confirmConnection()
    vk = {"vehicle_name": args.vehicle}
    client.enableApiControl(True, **vk)
    client.armDisarm(True, **vk)

    arc_deg = float(sv.orbit_arc_deg)
    rows = probe_waypoints(
        client,
        vk,
        cx,
        cy,
        alt,
        n,
        r_min,
        r_max,
        lateral_max,
        clearance_m,
        sv.orbit_start_deg,
        arc_deg,
    )
    xy_pts = [(r["x"], r["y"]) for r in rows]
    wps = plan_survey_from_xy(spec, xy_pts)
    (mission_dir / "waypoints.json").write_text(
        json.dumps([w.as_dict() for w in wps], indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    out = {
        "path_mode": "freeform",
        "path_curve": str(sv.path_curve),
        "center_xyz": [cx, cy, cz],
        "r_min_m": r_min,
        "r_max_m": r_max,
        "lateral_max_m": lateral_max,
        "clearance_m": clearance_m,
        "altitude_m": alt,
        "orbit_start_deg": sv.orbit_start_deg,
        "orbit_arc_deg": arc_deg,
        "path_closed": arc_deg >= 360.0,
        "bearings": rows,
    }
    (mission_dir / "orbit_clearance.json").write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
