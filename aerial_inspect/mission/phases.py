"""SEARCH / APPROACH phase planning for WAM deploy."""
from __future__ import annotations

import math
from typing import Any, Dict, Tuple

from aerial_inspect.mission.schema import MissionSpec


def compute_approach_goal(spec: MissionSpec) -> Tuple[float, float, float]:
    """Standoff waypoint outside bridge centroid (between search area and target)."""
    if spec.bridge_centroid_xyz is None:
        cx, cy = spec.search.center_xy
        cz = spec.search.altitude_m
    else:
        cx, cy, cz = spec.bridge_centroid_xyz

    scx, scy = spec.search.center_xy
    dx = float(cx) - scx
    dy = float(cy) - scy
    dist = math.hypot(dx, dy)
    if dist < 1e-3:
        dx, dy, dist = 1.0, 0.0, 1.0
    ux, uy = dx / dist, dy / dist
    standoff = float(spec.target.standoff_dist_m)
    gx = float(cx) - ux * standoff
    gy = float(cy) - uy * standoff
    gz = max(float(cz) + float(spec.target.standoff_height_m), float(spec.survey.altitude_m))
    return (gx, gy, gz)


def build_phase_plan(spec: MissionSpec) -> Dict[str, Any]:
    """Phase metadata written to mission artifacts for shell deploy scripts."""
    gx, gy, gz = compute_approach_goal(spec)
    scx, scy = spec.search.center_xy
    return {
        "mission_id": spec.mission_id,
        "visual_prompt": spec.target.visual_prompt,
        "target_category": spec.target.category,
        "search": {
            "center_xy": [scx, scy],
            "radius_m": spec.search.radius_m,
            "altitude_m": spec.search.altitude_m,
            "max_steps": 400,
            "record_auto": True,
        },
        "approach": {
            "goal": {"x": gx, "y": gy, "z": gz},
            "standoff_dist_m": spec.target.standoff_dist_m,
            "max_steps": 200,
            "record_auto": True,
        },
        "bridge_centroid_xyz": list(spec.bridge_centroid_xyz) if spec.bridge_centroid_xyz else None,
    }
