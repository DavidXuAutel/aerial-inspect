#!/usr/bin/env python3
"""CLI entry: aerial-inspect plan | export-wam | check-capture."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from aerial_inspect.adapters.wam_platform import export_wam_waypoints
from aerial_inspect.capture.session import CaptureSessionRef
from aerial_inspect.mission.orchestrator import load_mission_yaml, plan_mission
from aerial_inspect.reconstruct.pipeline import quality_check


def _cmd_plan(args: argparse.Namespace) -> int:
    spec = load_mission_yaml(Path(args.config))
    out = Path(args.output)
    summary = plan_mission(spec, out)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


def _cmd_export_wam(args: argparse.Namespace) -> int:
    out = export_wam_waypoints(Path(args.mission_dir))
    print(f"wrote {out}")
    return 0


def _cmd_check_capture(args: argparse.Namespace) -> int:
    ref = CaptureSessionRef.from_run_dir(Path(args.capture_dir))
    q = quality_check(ref.run_dir)
    print(json.dumps(q, indent=2))
    return 0 if q.get("min_frames_met") else 1


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="aerial-inspect")
    sub = p.add_subparsers(dest="cmd", required=True)

    p_plan = sub.add_parser("plan", help="Generate survey waypoints from mission YAML")
    p_plan.add_argument("config", help="configs/missions/*.yaml")
    p_plan.add_argument("-o", "--output", required=True, help="artifacts/mission_xxx")
    p_plan.set_defaults(func=_cmd_plan)

    p_exp = sub.add_parser("export-wam", help="Export wam_waypoints.json for deploy")
    p_exp.add_argument("mission_dir")
    p_exp.set_defaults(func=_cmd_export_wam)

    p_chk = sub.add_parser("check-capture", help="QC a recorder run directory")
    p_chk.add_argument("capture_dir")
    p_chk.set_defaults(func=_cmd_check_capture)

    args = p.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
