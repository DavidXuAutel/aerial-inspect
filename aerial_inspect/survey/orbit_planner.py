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


def plan_clearance_orbit(
    center: np.ndarray,
    spec: SurveySpec,
    radii_m: List[float],
) -> List[Waypoint]:
    """Variable-radius orbit; each waypoint uses radii_m[i] (same order as angles)."""
    cx, cy, _cz = float(center[0]), float(center[1]), float(center[2])
    alt = float(spec.altitude_m)
    n = len(radii_m)
    if n < 4:
        raise ValueError(f"need at least 4 radii, got {n}")
    start = math.radians(float(spec.orbit_start_deg))
    waypoints: List[Waypoint] = []
    for lap in range(int(spec.num_laps)):
        alt_lap = alt + lap * 3.0
        for i in range(n):
            theta = start + 2.0 * math.pi * i / n
            r = float(radii_m[i])
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
                    label=f"clear_l{lap}_i{i:02d}",
                    hold_frames=3,
                )
            )
    return waypoints


def smooth_radii(radii: List[float], window: int = 5) -> List[float]:
    if window <= 1 or len(radii) < window:
        return list(radii)
    out: List[float] = []
    half = window // 2
    for i in range(len(radii)):
        vals = []
        for j in range(i - half, i + half + 1):
            vals.append(radii[j % len(radii)])
        out.append(float(sum(vals) / len(vals)))
    return out


def smooth_xy(
    points: List[Tuple[float, float]],
    window: int = 5,
) -> List[Tuple[float, float]]:
    if window <= 1 or len(points) < window:
        return list(points)
    xs = smooth_radii([p[0] for p in points], window=window)
    ys = smooth_radii([p[1] for p in points], window=window)
    return [(x, y) for x, y in zip(xs, ys)]


def plan_xy_orbit(
    center: np.ndarray,
    spec: SurveySpec,
    xy_points: List[Tuple[float, float]],
) -> List[Waypoint]:
    """Freeform orbit: explicit (x,y) per waypoint; yaw always faces building center."""
    cx, cy = float(center[0]), float(center[1])
    alt = float(spec.altitude_m)
    n = len(xy_points)
    if n < 4:
        raise ValueError(f"need at least 4 xy points, got {n}")
    waypoints: List[Waypoint] = []
    for lap in range(int(spec.num_laps)):
        alt_lap = alt + lap * 3.0
        for i, (x, y) in enumerate(xy_points):
            pos = np.array([x, y], dtype=np.float64)
            yaw = _yaw_toward(pos, np.array([cx, cy]))
            waypoints.append(
                Waypoint(
                    x=float(x),
                    y=float(y),
                    z=alt_lap,
                    yaw_rad=yaw,
                    label=f"free_l{lap}_i{i:02d}",
                    hold_frames=3,
                )
            )
    return waypoints


def resolve_span_axis_deg(spec: MissionSpec) -> float:
    """Bridge long-axis bearing: detected > mission yaml > survey default."""
    if spec.bridge_span_axis_deg is not None:
        return float(spec.bridge_span_axis_deg)
    return float(spec.survey.span_axis_deg)


def plan_ellipse_orbit(
    center: np.ndarray,
    spec: SurveySpec,
    span_axis_deg: float,
) -> List[Waypoint]:
    """Oriented ellipse: semi-major along bridge span, semi-minor = cross-span standoff."""
    cx, cy, _cz = float(center[0]), float(center[1]), float(center[2])
    alt = float(spec.altitude_m)
    b = float(spec.radius_m)
    aspect = max(float(spec.ellipse_aspect), 1.0)
    a = b * aspect
    n = max(4, int(spec.points_per_lap))
    axis = math.radians(float(span_axis_deg))
    ca, sa = math.cos(axis), math.sin(axis)
    waypoints: List[Waypoint] = []
    for lap in range(int(spec.num_laps)):
        alt_lap = alt + lap * 3.0
        for i in range(n):
            theta = 2.0 * math.pi * i / n
            x_local = a * math.cos(theta)
            y_local = b * math.sin(theta)
            x = cx + x_local * ca - y_local * sa
            y = cy + x_local * sa + y_local * ca
            pos = np.array([x, y], dtype=np.float64)
            yaw = _yaw_toward(pos, np.array([cx, cy]))
            waypoints.append(
                Waypoint(
                    x=float(x),
                    y=float(y),
                    z=alt_lap,
                    yaw_rad=yaw,
                    label=f"ellipse_l{lap}_i{i:02d}",
                    hold_frames=3,
                )
            )
    return waypoints


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
    if pattern == "clearance_orbit":
        raise ValueError(
            "clearance_orbit requires orbit_clearance.json — run scripts/probe_orbit_clearance.py first"
        )
    if pattern == "ellipse_orbit":
        return plan_ellipse_orbit(center, spec.survey, resolve_span_axis_deg(spec))
    if pattern in ("span_facade", "span_facade_dual"):
        from aerial_inspect.survey.view_planner import plan_span_facade, plan_span_facade_dual

        corridor = tuple(spec.search.center_xy)
        if float(spec.survey.span_extent_m) > 0:
            extent = float(spec.survey.span_extent_m)
        else:
            extent = 2.0 * float(spec.survey.radius_m) * max(float(spec.survey.ellipse_aspect), 1.0)
        axis = resolve_span_axis_deg(spec)
        if pattern == "span_facade_dual":
            return plan_span_facade_dual(
                center,
                spec.survey,
                spec.target,
                axis,
                corridor_xy=corridor,
                span_extent_m=extent,
            )
        return plan_span_facade(
            center,
            spec.survey,
            spec.target,
            axis,
            corridor_xy=corridor,
            span_extent_m=extent,
        )
    return plan_horizontal_orbit(center, spec.survey)


def plan_survey_from_clearance(
    spec: MissionSpec,
    radii_m: List[float],
) -> List[Waypoint]:
    if spec.bridge_centroid_xyz is None:
        raise ValueError("bridge_centroid_xyz required for clearance orbit")
    center = np.asarray(spec.bridge_centroid_xyz, dtype=np.float64).reshape(3)
    return plan_clearance_orbit(center, spec.survey, radii_m)


def plan_survey_from_xy(
    spec: MissionSpec,
    xy_points: List[Tuple[float, float]],
) -> List[Waypoint]:
    if spec.bridge_centroid_xyz is None:
        raise ValueError("bridge_centroid_xyz required for freeform orbit")
    center = np.asarray(spec.bridge_centroid_xyz, dtype=np.float64).reshape(3)
    return plan_xy_orbit(center, spec.survey, xy_points)


def estimate_path_length_m(waypoints: List[Waypoint]) -> float:
    if len(waypoints) < 2:
        return 0.0
    pts = np.array([[w.x, w.y, w.z] for w in waypoints])
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
