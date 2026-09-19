#!/usr/bin/env python3
"""Hard gate: SEARCH traj must contain real detector hits, not tracker-only locks."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def validate_search_traj(
    traj_path: Path,
    *,
    min_det_hits: int = 10,
    min_det_fraction: float = 0.02,
    min_tracked_with_det: int = 10,
    min_goal_rel_dist_m: float = 40.0,
    min_x_span_m: float = 0.0,
    min_max_x: float = 0.0,
) -> dict:
    if not traj_path.is_file():
        return {"status": "fail", "reason": f"missing traj: {traj_path}"}

    n_rows = 0
    n_det_hit = 0
    n_tracked = 0
    n_tracked_det = 0
    goal_dists: list[float] = []
    xs: list[float] = []

    for line in traj_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        n_rows += 1
        pos = row.get("pos")
        if pos and len(pos) >= 1:
            xs.append(float(pos[0]))
        det_hit = bool(row.get("det_hit"))
        if det_hit:
            n_det_hit += 1
        state = str(row.get("tracker_state") or (row.get("step_info") or {}).get("tracker_state") or "")
        has_bridge_lock = bool(row.get("bridge_goal_rel") or (row.get("step_info") or {}).get("bridge_goal_rel"))
        if state.lower() in {"tracking", "occluded", "arrived"}:
            n_tracked += 1
            if det_hit:
                n_tracked_det += 1
        elif det_hit and has_bridge_lock:
            n_tracked_det += 1
            goal_rel = (
                row.get("bridge_goal_rel")
                or (row.get("step_info") or {}).get("bridge_goal_rel")
                or row.get("goal_rel")
                or (row.get("step_info") or {}).get("goal_rel")
            )
            if goal_rel and len(goal_rel) >= 4:
                goal_dists.append(float(goal_rel[3]))

    if n_rows == 0:
        return {"status": "fail", "reason": "empty SEARCH traj", "n_rows": 0}

    det_frac = n_det_hit / n_rows
    med_dist = float(sorted(goal_dists)[len(goal_dists) // 2]) if goal_dists else None
    x_min = min(xs) if xs else None
    x_max = max(xs) if xs else None
    x_span = float(x_max - x_min) if x_min is not None and x_max is not None else None

    report = {
        "status": "ok",
        "n_rows": n_rows,
        "n_det_hit": n_det_hit,
        "det_fraction": round(det_frac, 4),
        "n_tracked": n_tracked,
        "n_tracked_with_det": n_tracked_det,
        "median_goal_rel_dist_m": med_dist,
        "x_min": x_min,
        "x_max": x_max,
        "x_span_m": round(x_span, 2) if x_span is not None else None,
    }

    if n_det_hit < min_det_hits:
        report["status"] = "fail"
        report["reason"] = f"det_hit {n_det_hit} < {min_det_hits} (no real vision detections)"
        return report
    if det_frac < min_det_fraction:
        report["status"] = "fail"
        report["reason"] = f"det_fraction {det_frac:.4f} < {min_det_fraction}"
        return report
    if n_tracked_det < min_tracked_with_det:
        report["status"] = "fail"
        report["reason"] = f"tracked+det {n_tracked_det} < {min_tracked_with_det}"
        return report
    if med_dist is not None and med_dist < min_goal_rel_dist_m:
        report["status"] = "fail"
        report["reason"] = (
            f"median goal_rel dist {med_dist:.1f}m < {min_goal_rel_dist_m}m "
            "(likely near-field false lock, not bridge at range)"
        )
        return report
    if min_x_span_m > 0 and (x_span is None or x_span < min_x_span_m):
        report["status"] = "fail"
        report["reason"] = (
            f"x_span {x_span}m < {min_x_span_m}m (SEARCH did not traverse corridor)"
        )
        return report
    if min_max_x > 0 and (x_max is None or x_max < min_max_x):
        report["status"] = "fail"
        report["reason"] = (
            f"x_max {x_max} < {min_max_x} (SEARCH never reached bridge corridor east)"
        )
        return report
    return report


def main() -> int:
    p = argparse.ArgumentParser(description="Validate WAM SEARCH traj before replan")
    p.add_argument("traj_path", help="sim_runs/search/traj/route00.jsonl")
    p.add_argument("--min-det-hits", type=int, default=10)
    p.add_argument("--min-det-fraction", type=float, default=0.02)
    p.add_argument("--min-tracked-with-det", type=int, default=10)
    p.add_argument("--min-goal-rel-dist-m", type=float, default=40.0)
    p.add_argument("--min-x-span-m", type=float, default=0.0)
    p.add_argument("--min-max-x", type=float, default=0.0)
    p.add_argument("--out", help="write JSON report")
    args = p.parse_args()

    report = validate_search_traj(
        Path(args.traj_path),
        min_det_hits=args.min_det_hits,
        min_det_fraction=args.min_det_fraction,
        min_tracked_with_det=args.min_tracked_with_det,
        min_goal_rel_dist_m=args.min_goal_rel_dist_m,
        min_x_span_m=args.min_x_span_m,
        min_max_x=args.min_max_x,
    )
    out = Path(args.out) if args.out else Path(args.traj_path).resolve().parent.parent / "search_validation.json"
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
