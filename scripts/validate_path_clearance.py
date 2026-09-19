#!/usr/bin/env python3
"""Validate clearance along the full survey/video path in AirSim."""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from airsim_pose_utils import yaw_toward_xy
from path_clearance import pose_clearance_ok, repair_pose_clearance


def _load_center(mission_dir: Path) -> tuple[float, float]:
    spec = json.loads((mission_dir / "mission_spec.json").read_text(encoding="utf-8"))
    c = spec.get("bridge_centroid_xyz")
    if not c:
        raise SystemExit("bridge_centroid_xyz missing")
    return float(c[0]), float(c[1])


def _path_settings(mission_dir: Path) -> tuple[str, str]:
    curve = "linear"
    clearance = mission_dir / "orbit_clearance.json"
    if clearance.is_file():
        data = json.loads(clearance.read_text(encoding="utf-8"))
        curve = str(data.get("path_curve", curve))
        if data.get("path_mode") == "freeform":
            return "freeform", curve
    survey = json.loads((mission_dir / "mission_spec.json").read_text()).get("survey", {})
    curve = str(survey.get("path_curve", curve))
    if survey.get("pattern") == "clearance_orbit":
        return "freeform", curve
    return "orbit", curve


def _build_video_poses(
    wps: list[dict],
    center_xy: tuple[float, float],
    path_mode: str,
    curve: str,
    steps_between: int = 12,
    hold_frames: int = 8,
) -> list[tuple[float, float, float, float]]:
    from export_sim_survey_video import _build_poses

    return _build_poses(wps, center_xy, path_mode, curve, steps_between, hold_frames)


def _dedupe_poses(
    poses: list[tuple[float, float, float, float]],
    xy_tol: float = 0.25,
) -> list[tuple[float, float, float, float]]:
    out: list[tuple[float, float, float, float]] = []
    for pose in poses:
        if not out:
            out.append(pose)
            continue
        lx, ly, _, _ = out[-1]
        x, y, _, _ = pose
        if math.hypot(x - lx, y - ly) >= xy_tol:
            out.append(pose)
    return out


def main() -> int:
    p = argparse.ArgumentParser(description="Validate path clearance in AirSim")
    p.add_argument("mission_dir")
    p.add_argument("--host", default=os.environ.get("AIRSIM_HOST", "127.0.0.1"))
    p.add_argument("--port", type=int, default=int(os.environ.get("AIRSIM_PORT", "41451")))
    p.add_argument("--vehicle", default=os.environ.get("AIRSIM_VEHICLE", "drone_1"))
    p.add_argument("--clearance-m", type=float, default=-1.0)
    p.add_argument("--steps-between", type=int, default=12)
    p.add_argument("--hold-frames", type=int, default=8)
    p.add_argument("--out", help="write JSON report path")
    args = p.parse_args()

    mission_dir = Path(args.mission_dir).resolve()
    clearance_path = mission_dir / "orbit_clearance.json"
    clearance_m = 12.0
    if clearance_path.is_file():
        clearance_m = float(json.loads(clearance_path.read_text()).get("clearance_m", clearance_m))
    if args.clearance_m > 0:
        clearance_m = args.clearance_m

    wps = json.loads((mission_dir / "waypoints.json").read_text(encoding="utf-8"))
    center_xy = _load_center(mission_dir)
    path_mode, curve = _path_settings(mission_dir)
    poses = _build_video_poses(wps, center_xy, path_mode, curve, args.steps_between, args.hold_frames)
    samples = _dedupe_poses(poses)

    import airsim

    client = airsim.MultirotorClient(ip=args.host, port=args.port)
    client.confirmConnection()
    vk = {"vehicle_name": args.vehicle}
    client.enableApiControl(True, **vk)

    rows: list[dict] = []
    for i, (x, y, z, yaw) in enumerate(samples):
        if path_mode == "freeform":
            x, y, z, yaw, meta, ok = repair_pose_clearance(
                client, vk, x, y, z, yaw, clearance_m, center_xy,
            )
        else:
            ok, meta = pose_clearance_ok(client, vk, x, y, z, yaw, clearance_m, settle_s=0.25)
        row = {
            "index": i,
            "x": round(x, 2),
            "y": round(y, 2),
            "z": round(z, 2),
            "depth_ok": ok,
            **meta,
        }
        rows.append(row)
        flag = "ok" if ok else "FAIL"
        print(
            f"[{i + 1}/{len(samples)}] {flag} clear={meta.get('min_clear_m', '?')}m "
            f"@ ({x:.1f},{y:.1f})",
            flush=True,
        )

    fails = [r for r in rows if not r["depth_ok"]]
    report = {
        "clearance_m": clearance_m,
        "path_mode": path_mode,
        "path_curve": curve,
        "n_samples": len(rows),
        "n_depth_fail": len(fails),
        "min_clear_m": min(r["min_clear_m"] for r in rows) if rows else None,
        "depth_failures": fails[:80],
        "samples": rows,
    }
    out_path = Path(args.out) if args.out else mission_dir / "path_clearance_report.json"
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {k: report[k] for k in ("clearance_m", "path_curve", "n_samples", "n_depth_fail", "min_clear_m")},
            indent=2,
        )
    )
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
