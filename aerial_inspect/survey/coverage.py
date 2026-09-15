"""Coverage heuristics for survey missions."""
from __future__ import annotations

import math
from typing import List

from aerial_inspect.mission.schema import SurveySpec
from aerial_inspect.survey.orbit_planner import Waypoint


def heading_overlap_ok(spec: SurveySpec, waypoints: List[Waypoint]) -> bool:
    """Check adjacent orbit points have enough heading change for stereo overlap."""
    if len(waypoints) < 2:
        return False
    hfov = math.radians(spec.camera_hfov_deg)
    min_dyaw = hfov * (1.0 - float(spec.heading_overlap))
    dyaws = []
    for i in range(1, len(waypoints)):
        dy = abs(waypoints[i].yaw_rad - waypoints[i - 1].yaw_rad)
        dy = min(dy, 2 * math.pi - dy)
        dyaws.append(dy)
    return float(sum(dyaws) / len(dyaws)) <= hfov and min(dyaws) > 0.05


def expected_frame_count(waypoints: List[Waypoint], fps: float = 5.0) -> int:
    return sum(max(1, w.hold_frames) for w in waypoints)
