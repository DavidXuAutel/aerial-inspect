"""Mission planning: spec → artifacts on disk."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml

from aerial_inspect.mission.phases import build_phase_plan
from aerial_inspect.mission.schema import MissionSpec
from aerial_inspect.survey.coverage import expected_frame_count, heading_overlap_ok
from aerial_inspect.survey.orbit_planner import estimate_path_length_m, plan_survey_waypoints


def load_mission_yaml(path: Path) -> MissionSpec:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return MissionSpec.from_dict(data)


def spec_to_dict(spec: MissionSpec) -> Dict[str, Any]:
    return {
        "mission_id": spec.mission_id,
        "instruction": spec.instruction,
        "search_area": {
            "center_xy": list(spec.search.center_xy),
            "radius_m": spec.search.radius_m,
            "altitude_m": spec.search.altitude_m,
        },
        "target": spec.target.__dict__,
        "survey": spec.survey.__dict__,
        "bridge_centroid_xyz": list(spec.bridge_centroid_xyz) if spec.bridge_centroid_xyz else None,
    }


def load_spec_from_mission_dir(mission_dir: Path) -> MissionSpec:
    path = mission_dir / "mission_spec.json"
    if not path.is_file():
        raise FileNotFoundError(f"missing {path}; run plan first")
    data = json.loads(path.read_text(encoding="utf-8"))
    return MissionSpec.from_dict(data)


def _write_spec(mission_dir: Path, spec: MissionSpec) -> None:
    (mission_dir / "mission_spec.json").write_text(
        json.dumps(spec_to_dict(spec), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


def _write_phase_plan(mission_dir: Path, spec: MissionSpec) -> Dict[str, Any]:
    phase_plan = build_phase_plan(spec)
    (mission_dir / "phase_plan.json").write_text(
        json.dumps(phase_plan, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    return phase_plan


def _write_survey_waypoints(mission_dir: Path, spec: MissionSpec) -> Tuple[Path, list]:
    waypoints = plan_survey_waypoints(spec)
    wp_path = mission_dir / "waypoints.json"
    wp_path.write_text(
        json.dumps([w.as_dict() for w in waypoints], indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    return wp_path, waypoints


def plan_mission(spec: MissionSpec, out_dir: Path, *, include_survey: bool = False) -> Dict[str, Any]:
    """Pre-search plan: phases only. Survey waypoints generated after SEARCH detection."""
    out_dir.mkdir(parents=True, exist_ok=True)
    _write_spec(out_dir, spec)
    phase_plan = _write_phase_plan(out_dir, spec)

    survey_status = "pending_detection"
    n_waypoints = 0
    wp_path: Optional[Path] = None
    waypoints = []

    if include_survey:
        if spec.bridge_centroid_xyz is None:
            raise ValueError("include_survey requires bridge_centroid_xyz (use replan-survey after SEARCH)")
        wp_path, waypoints = _write_survey_waypoints(out_dir, spec)
        survey_status = "ready"
        n_waypoints = len(waypoints)

    summary: Dict[str, Any] = {
        "mission_id": spec.mission_id,
        "instruction": spec.instruction,
        "phase_plan": ["search", "replan_survey", "approach", "survey", "offline_reconstruct"],
        "survey_status": survey_status,
        "n_waypoints": n_waypoints,
        "target": {
            "visual_prompt": spec.target.visual_prompt,
            "standoff_m": spec.target.standoff_dist_m,
        },
        "survey": {
            "pattern": spec.survey.pattern,
            "radius_m": spec.survey.radius_m,
            "num_laps": spec.survey.num_laps,
        },
        "bridge_centroid_xyz": list(spec.bridge_centroid_xyz) if spec.bridge_centroid_xyz else None,
        "approach_goal": phase_plan["approach"]["goal"],
    }
    if waypoints:
        summary["path_length_m"] = round(estimate_path_length_m(waypoints), 2)
        summary["expected_frames"] = expected_frame_count(waypoints)
        summary["heading_overlap_ok"] = heading_overlap_ok(spec.survey, waypoints)
        summary["waypoints_file"] = str(wp_path)

    (out_dir / "mission_plan.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    return summary


def replan_survey(
    mission_dir: Path,
    centroid_xyz: Tuple[float, float, float],
    *,
    source: str = "detected",
) -> Dict[str, Any]:
    """Apply bridge centroid and generate survey waypoints + refreshed approach goal."""
    spec = load_spec_from_mission_dir(mission_dir)
    spec.bridge_centroid_xyz = centroid_xyz
    _write_spec(mission_dir, spec)
    phase_plan = _write_phase_plan(mission_dir, spec)
    wp_path, waypoints = _write_survey_waypoints(mission_dir, spec)

    summary = {
        "mission_id": spec.mission_id,
        "survey_status": "ready",
        "centroid_source": source,
        "bridge_centroid_xyz": list(centroid_xyz),
        "n_waypoints": len(waypoints),
        "path_length_m": round(estimate_path_length_m(waypoints), 2),
        "expected_frames": expected_frame_count(waypoints),
        "heading_overlap_ok": heading_overlap_ok(spec.survey, waypoints),
        "waypoints_file": str(wp_path),
        "approach_goal": phase_plan["approach"]["goal"],
    }
    plan_path = mission_dir / "mission_plan.json"
    base = json.loads(plan_path.read_text(encoding="utf-8")) if plan_path.is_file() else {}
    base.update(summary)
    plan_path.write_text(json.dumps(base, indent=2, ensure_ascii=False), encoding="utf-8")
    return summary
