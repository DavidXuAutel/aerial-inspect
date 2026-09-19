#!/usr/bin/env python3
"""Require flown survey endpoints to match planned waypoints (no teleport-only capture)."""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


def _last_pos(traj_path: Path) -> list[float] | None:
    if not traj_path.is_file():
        return None
    last = None
    for line in traj_path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            last = json.loads(line)
    if last is None or "pos" not in last:
        return None
    return [float(x) for x in last["pos"][:3]]


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("mission_dir")
    p.add_argument("--max-xy-error-m", type=float, default=15.0)
    p.add_argument("--max-z-error-m", type=float, default=12.0)
    p.add_argument("--min-fraction", type=float, default=0.85)
    args = p.parse_args()

    mission_dir = Path(args.mission_dir).resolve()
    wps = json.loads((mission_dir / "waypoints.json").read_text(encoding="utf-8"))
    survey = mission_dir / "sim_runs" / "survey"
    wp_dirs = sorted([d for d in survey.iterdir() if d.is_dir() and d.name.startswith("wp_")])

    rows: list[dict] = []
    ok = 0
    for i, wp_dir in enumerate(wp_dirs):
        if i >= len(wps):
            break
        planned = wps[i]
        flown = _last_pos(wp_dir / "traj" / "route00.jsonl")
        if flown is None:
            rows.append({"waypoint": wp_dir.name, "status": "no_traj"})
            continue
        px, py, pz = float(planned["x"]), float(planned["y"]), float(planned["z"])
        err_xy = math.hypot(flown[0] - px, flown[1] - py)
        err_z = abs(flown[2] - pz)
        passed = err_xy <= args.max_xy_error_m and err_z <= args.max_z_error_m
        if passed:
            ok += 1
        rows.append(
            {
                "waypoint": wp_dir.name,
                "label": planned.get("label", ""),
                "err_xy_m": round(err_xy, 2),
                "err_z_m": round(err_z, 2),
                "ok": passed,
            }
        )

    n = len(wp_dirs)
    frac = ok / n if n else 0.0
    report = {
        "status": "ok" if frac >= args.min_fraction else "fail",
        "n_waypoints": n,
        "n_pose_ok": ok,
        "fraction": round(frac, 4),
        "min_fraction": args.min_fraction,
        "max_xy_error_m": args.max_xy_error_m,
        "max_z_error_m": args.max_z_error_m,
        "waypoints": rows,
    }
    if report["status"] == "fail":
        report["reason"] = "flown pose deviates from planned survey waypoints"
    out = mission_dir / "survey_pose_report.json"
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({k: report[k] for k in ("status", "n_pose_ok", "n_waypoints", "fraction")}, indent=2))
    return 0 if report["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
