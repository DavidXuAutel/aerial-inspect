#!/usr/bin/env python3
"""Gate approach/survey eval_result.json — no silent SR=0% passes."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def validate_eval(path: Path, *, phase: str, min_det_fraction: float = 0.15) -> dict:
    if not path.is_file():
        return {"status": "fail", "phase": phase, "reason": f"missing {path}"}
    data = json.loads(path.read_text(encoding="utf-8"))
    eps = data.get("episodes") or []
    if not eps:
        return {"status": "fail", "phase": phase, "reason": "no episodes in eval_result"}
    ep = eps[0]
    det_frac = float(ep.get("detection_frac") or 0.0)
    arrived = bool(ep.get("arrived") or ep.get("arrived_vision"))
    actual_len = float(ep.get("actual_length_m") or 0.0)
    steps = int(ep.get("steps") or 0)

    report = {
        "status": "ok",
        "phase": phase,
        "detection_frac": det_frac,
        "arrived": arrived,
        "actual_length_m": actual_len,
        "steps": steps,
        "verdict": data.get("verdict"),
    }

    if phase == "approach":
        if not arrived and actual_len < 20.0:
            report["status"] = "fail"
            report["reason"] = "approach did not arrive and moved < 20m"
            return report
        if det_frac < min_det_fraction:
            report["status"] = "fail"
            report["reason"] = f"approach det_fraction {det_frac:.3f} < {min_det_fraction}"
            return report
    elif phase == "survey_wp":
        if steps < 5:
            report["status"] = "fail"
            report["reason"] = f"survey wp too few steps ({steps})"
            return report
        if det_frac < min_det_fraction * 0.5:
            report["status"] = "fail"
            report["reason"] = f"survey wp det_fraction {det_frac:.3f} too low"
            return report
    return report


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("eval_json")
    p.add_argument("--phase", choices=("approach", "survey_wp"), required=True)
    p.add_argument("--min-det-fraction", type=float, default=0.15)
    args = p.parse_args()
    report = validate_eval(Path(args.eval_json), phase=args.phase, min_det_fraction=args.min_det_fraction)
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
