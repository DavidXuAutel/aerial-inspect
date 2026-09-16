"""Survey waypoint generation for bridge / structure inspection."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from aerial_inspect.mission.schema import MissionSpec, SurveySpec


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    yaw_rad: float
    label: str = ""
    hold_frames: int = 3

    def as_dict(self) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw_rad": self.yaw_rad,
            "yaw_deg": math.degrees(self.yaw_rad),
            "label": self.label,
            "hold_frames": self.hold_frames,
        }


def _yaw_toward(from_xy: np.ndarray, to_xy: np.ndarray) -> float:
    d = to_xy - from_xy
    return float(math.atan2(d[1], d[0]))


def plan_horizontal_orbit(
    center: np.ndarray,
    spec: SurveySpec,
) -> List[Waypoint]:
    """Circle around centroid at fixed altitude; yaw faces center (inward shooting)."""
    cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
    alt = float(spec.altitude_m)
    r = float(spec.radius_m)
    n = max(8, int(spec.points_per_lap))
    waypoints: List[Waypoint] = []
    for lap in range(int(spec.num_laps)):
        alt_lap = alt + lap * 3.0
        for i in range(n):
            theta = 2.0 * math.pi * i / n
            x = cx + r * math.cos(theta)
            y = cy + r * math.sin(theta)
            pos = np.array([x, y], dtype=np.float64)
            yaw = _yaw_toward(pos, np.array([cx, cy]))
            waypoints.append(
                Waypoint(
                    x=x,
                    y=y,
                    z=alt_lap,
                    yaw_rad=yaw,
                    label=f"orbit_l{lap}_i{i:02d}",
                    hold_frames=3,
                )
            )
    return waypoints


def plan_facade_arc(
    center: np.ndarray,
    spec: SurveySpec,
    approach_bearing_rad: float = 0.0,
) -> List[Waypoint]:
    """Arc on one side of the structure (e.g. downstream facade)."""
    cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
    r = float(spec.radius_m)
    alt = float(spec.altitude_m)
    arc = math.radians(float(spec.facade_arc_deg))
    n = max(6, int(spec.points_per_lap))
    waypoints: List[Waypoint] = []
    for lap in range(int(spec.num_laps)):
        alt_lap = alt + lap * 5.0
        for i in range(n):
            frac = i / max(n - 1, 1)
            theta = approach_bearing_rad - arc / 2 + arc * frac
            x = cx + r * math.cos(theta)
            y = cy + r * math.sin(theta)
            pos = np.array([x, y])
            yaw = _yaw_toward(pos, np.array([cx, cy]))
            waypoints.append(
                Waypoint(
                    x=x,
                    y=y,
                    z=alt_lap,
                    yaw_rad=yaw,
                    label=f"facade_l{lap}_i{i:02d}",
                    hold_frames=4,
                )
            )
    return waypoints


def plan_survey_waypoints(spec: MissionSpec) -> List[Waypoint]:
    if spec.bridge_centroid_xyz is None:
        cx, cy = spec.search.center_xy
        cz = spec.search.altitude_m
        center = np.array([cx, cy, cz], dtype=np.float64)
    else:
        center = np.asarray(spec.bridge_centroid_xyz, dtype=np.float64).reshape(3)

    pattern = str(spec.survey.pattern)
    if pattern == "facade_arc":
        bearing = math.radians(float(spec.survey.approach_bearing_deg))
        return plan_facade_arc(center, spec.survey, approach_bearing_rad=bearing)
    return plan_horizontal_orbit(center, spec.survey)


def estimate_path_length_m(waypoints: List[Waypoint]) -> float:
    if len(waypoints) < 2:
        return 0.0
    pts = np.array([[w.x, w.y, w.z] for w in waypoints])
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
