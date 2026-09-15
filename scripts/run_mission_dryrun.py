#!/usr/bin/env python3
"""Simulate end-to-end mission planning without connecting to aircraft."""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aerial_inspect.adapters.wam_platform import export_wam_phases, export_wam_waypoints
from aerial_inspect.mission.centroid import estimate_centroid_from_traj, write_detected_centroid
from aerial_inspect.mission.orchestrator import load_mission_yaml, plan_mission, replan_survey

SEARCH_FIXTURE = ROOT / "tests/fixtures/search_traj.jsonl"


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: run_mission_dryrun.py <configs/missions/*.yaml>")
        return 1
    cfg = Path(sys.argv[1])
    spec = load_mission_yaml(cfg)
    out = ROOT / "artifacts" / spec.mission_id

    print("=== 1. plan (pre-search) ===")
    summary = plan_mission(spec, out, include_survey=False)
    phases_path = export_wam_phases(out)

    print("=== 2. SEARCH (simulated via fixture traj) ===")
    detection = estimate_centroid_from_traj(SEARCH_FIXTURE, min_samples=3)
    detection["source"] = "dryrun_fixture"
    write_detected_centroid(out, detection)
    if detection["status"] != "ok":
        print(json.dumps(detection, indent=2))
        return 1

    print("=== 3. replan survey ===")
    survey_summary = replan_survey(out, tuple(detection["centroid_xyz"]), source="dryrun_fixture")
    wam_path = export_wam_waypoints(out)
    export_wam_phases(out)

    print("\n=== Mission plan ===")
    print(json.dumps({**summary, **survey_summary}, indent=2, ensure_ascii=False))
    print(f"\nDetected centroid: {detection['centroid_xyz']}")
    print(f"WAM phases: {phases_path}")
    print(f"WAM survey: {wam_path}")
    print("\nEnd-to-end phases:")
    print("  1. SEARCH   — scripts/run_wam_search.sh")
    print("  2. REPLAN   — aerial-inspect replan-survey --export-wam")
    print("  3. APPROACH — scripts/run_wam_approach.sh")
    print("  4. SURVEY   — scripts/run_wam_survey.sh")
    print("  5. OFFLINE  — scripts/run_offline_reconstruct.py <capture_run_dir>")
    print("\nFull pipeline: scripts/run_mission_pipeline.sh", cfg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
