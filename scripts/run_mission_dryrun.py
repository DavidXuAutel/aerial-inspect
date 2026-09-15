#!/usr/bin/env python3
"""Print mission FSM and survey stats without connecting to aircraft."""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aerial_inspect.adapters.wam_platform import export_wam_waypoints
from aerial_inspect.mission.orchestrator import load_mission_yaml, plan_mission


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: run_mission_dryrun.py <configs/missions/*.yaml>")
        return 1
    cfg = Path(sys.argv[1])
    spec = load_mission_yaml(cfg)
    out = ROOT / "artifacts" / spec.mission_id
    summary = plan_mission(spec, out)
    wam_path = export_wam_waypoints(out)
    print("=== Mission plan ===")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"\nWAM export: {wam_path}")
    print("\nPhases:")
    print("  1. SEARCH  — wam_vgoal_deploy + visual_prompt (vgoal repo)")
    print("  2. APPROACH — standoff toward detected bridge")
    print("  3. SURVEY   — sequential waypoints in wam_waypoints.json")
    print("  4. OFFLINE  — run_offline_reconstruct.py on capture run_*")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
