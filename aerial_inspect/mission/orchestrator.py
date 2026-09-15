"""Mission planning: spec → artifacts on disk."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict

import yaml

from aerial_inspect.mission.schema import MissionSpec
from aerial_inspect.survey.coverage import expected_frame_count, heading_overlap_ok
from aerial_inspect.survey.orbit_planner import estimate_path_length_m, plan_survey_waypoints


def load_mission_yaml(path: Path) -> MissionSpec:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return MissionSpec.from_dict(data)


def plan_mission(spec: MissionSpec, out_dir: Path) -> Dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    waypoints = plan_survey_waypoints(spec)
    wp_path = out_dir / "waypoints.json"
    wp_path.write_text(
        json.dumps([w.as_dict() for w in waypoints], indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    summary = {
        "mission_id": spec.mission_id,
        "instruction": spec.instruction,
        "phase_plan": ["search", "approach", "survey", "offline_reconstruct"],
        "n_waypoints": len(waypoints),
        "path_length_m": round(estimate_path_length_m(waypoints), 2),
        "expected_frames": expected_frame_count(waypoints),
        "heading_overlap_ok": heading_overlap_ok(spec.survey, waypoints),
        "target": {
            "visual_prompt": spec.target.visual_prompt,
            "standoff_m": spec.target.standoff_dist_m,
        },
        "survey": {
            "pattern": spec.survey.pattern,
            "radius_m": spec.survey.radius_m,
            "num_laps": spec.survey.num_laps,
        },
        "waypoints_file": str(wp_path),
    }
    (out_dir / "mission_plan.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    (out_dir / "mission_spec.json").write_text(
        json.dumps(
            {
                "mission_id": spec.mission_id,
                "instruction": spec.instruction,
                "search_area": {
                    "center_xy": list(spec.search.center_xy),
                    "radius_m": spec.search.radius_m,
                    "altitude_m": spec.search.altitude_m,
                },
                "target": spec.target.__dict__,
                "survey": spec.survey.__dict__,
                "bridge_centroid_xyz": list(spec.bridge_centroid_xyz)
                if spec.bridge_centroid_xyz
                else None,
            },
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    return summary
