#!/usr/bin/env python3
"""Require most survey waypoints to have flown traj (no teleport-only capture)."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("mission_dir")
    p.add_argument("--min-fraction", type=float, default=0.85)
    args = p.parse_args()

    survey = Path(args.mission_dir).resolve() / "sim_runs" / "survey"
    if not survey.is_dir():
        print(json.dumps({"status": "fail", "reason": f"missing {survey}"}))
        return 1

    wp_dirs = sorted([d for d in survey.iterdir() if d.is_dir() and d.name.startswith("wp_")])
    if not wp_dirs:
        print(json.dumps({"status": "fail", "reason": "no wp_* dirs"}))
        return 1

    ok = 0
    for wp in wp_dirs:
        traj = wp / "traj" / "route00.jsonl"
        if traj.is_file() and traj.stat().st_size > 10:
            ok += 1

    frac = ok / len(wp_dirs)
    report = {
        "status": "ok" if frac >= args.min_fraction else "fail",
        "n_waypoints": len(wp_dirs),
        "n_with_traj": ok,
        "fraction": round(frac, 4),
        "min_fraction": args.min_fraction,
    }
    if report["status"] == "fail":
        report["reason"] = "survey waypoints missing flown traj — cannot claim autonomous orbit"
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
