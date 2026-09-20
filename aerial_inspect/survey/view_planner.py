"""Photogrammetry-aware survey viewpoints for linear structures (bridges)."""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from aerial_inspect.mission.schema import MissionSpec, SurveySpec, TargetSpec
from aerial_inspect.survey.orbit_planner import Waypoint, _yaw_toward


def ground_footprint_width_m(standoff_m: float, hfov_deg: float) -> float:
    """Horizontal ground swath width at standoff distance."""
    return 2.0 * standoff_m * math.tan(math.radians(hfov_deg) * 0.5)


def along_track_spacing_m(standoff_m: float, hfov_deg: float, heading_overlap: float) -> float:
    """Waypoint spacing along span for desired heading overlap."""
    w = ground_footprint_width_m(standoff_m, hfov_deg)
    return max(w * (1.0 - float(heading_overlap)), 10.0)


def span_unit_vectors(span_axis_deg: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """Along-span (u) and cross-span (v) unit vectors in XY."""
    axis = math.radians(float(span_axis_deg))
    u = (math.cos(axis), math.sin(axis))
    v = (-math.sin(axis), math.cos(axis))
    return u, v


def auto_facade_sign(
    centroid_xy: Tuple[float, float],
    corridor_xy: Tuple[float, float],
    span_axis_deg: float,
) -> float:
    """Fly on the SEARCH corridor side of the span (same side as area search)."""
    cx, cy = centroid_xy
    to_corridor = (float(corridor_xy[0]) - cx, float(corridor_xy[1]) - cy)
    _, v = span_unit_vectors(span_axis_deg)
    dot = to_corridor[0] * v[0] + to_corridor[1] * v[1]
    return 1.0 if dot >= 0.0 else -1.0


def estimate_span_extent_m(
    detection: Optional[dict],
    *,
    fallback_m: float,
) -> float:
    """Span coverage length from SEARCH PCA / std; fallback to mission default."""
    if detection:
        span = detection.get("span_axis") or {}
        ev = span.get("eigenvalues_xy")
        if ev and len(ev) >= 2:
            major = max(float(ev[0]), float(ev[1]))
            return max(2.0 * math.sqrt(major), 60.0)
        std = detection.get("std_m")
        if std and len(std) >= 2:
            return max(2.0 * max(float(std[0]), float(std[1])), 60.0)
    return max(float(fallback_m), 60.0)


def load_detection(mission_dir: Path) -> Optional[dict]:
    path = mission_dir / "detected_centroid.json"
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def load_centerline_xy(path: str | Path) -> Optional[np.ndarray]:
    """Load deck centerline as Nx2 array from list [[x,y],...] or {points_xy: ...}."""
    p = Path(path)
    if not p.is_file():
        return None
    raw = json.loads(p.read_text(encoding="utf-8"))
    if isinstance(raw, dict):
        pts = raw.get("points_xy") or raw.get("centerline") or raw.get("xy")
    else:
        pts = raw
    if not pts:
        return None
    arr = np.asarray(pts, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 2:
        return None
    return arr[:, :2]


def resolve_span_extent_m(
    spec: SurveySpec,
    detection: Optional[dict],
) -> float:
    """Prefer explicit mission span_extent_m (main channel); else detection; else short ellipse."""
    if float(spec.span_extent_m) > 0:
        return float(spec.span_extent_m)
    return estimate_span_extent_m(
        detection,
        fallback_m=2.0 * float(spec.radius_m) * max(float(spec.ellipse_aspect), 1.0),
    )


def _facade_pass_waypoints(
    center_xy: Tuple[float, float],
    *,
    u: Tuple[float, float],
    v: Tuple[float, float],
    extent: float,
    standoff: float,
    sign: float,
    alt: float,
    hfov_deg: float,
    heading_overlap: float,
    label_prefix: str,
) -> List[Waypoint]:
    """One side-aligned facade strip; yaw always faces the deck centerline."""
    cx, cy = center_xy
    standoff = abs(float(standoff))
    spacing = along_track_spacing_m(standoff, hfov_deg, heading_overlap)
    n_along = max(4, int(math.ceil(extent / spacing)) + 1)
    waypoints: List[Waypoint] = []
    for i in range(n_along):
        frac = i / max(n_along - 1, 1)
        s = (frac - 0.5) * extent
        px = cx + s * u[0] + sign * standoff * v[0]
        py = cy + s * u[1] + sign * standoff * v[1]
        look_x = cx + s * u[0]
        look_y = cy + s * u[1]
        yaw = _yaw_toward(np.array([px, py]), np.array([look_x, look_y]))
        waypoints.append(
            Waypoint(
                x=float(px),
                y=float(py),
                z=float(alt),
                yaw_rad=yaw,
                label=f"{label_prefix}_i{i:02d}",
                hold_frames=3,
            )
        )
    return waypoints


def plan_span_facade(
    center: np.ndarray,
    spec: SurveySpec,
    target: TargetSpec,
    span_axis_deg: float,
    *,
    corridor_xy: Tuple[float, float],
    span_extent_m: Optional[float] = None,
    facade_sign: Optional[float] = None,
) -> List[Waypoint]:
    """
    Span-aligned facade pass: camera faces deck, positions on one side at standoff.

    Viewpoints are spaced from HFOV + heading_overlap (not a fixed 24-point ellipse).
    """
    cx, cy = float(center[0]), float(center[1])
    near = float(target.approach_standoff_dist_m or target.standoff_dist_m or spec.radius_m)
    far = float(target.standoff_dist_m or spec.radius_m)
    if spec.lap_standoffs_m:
        lap_standoffs = [float(x) for x in spec.lap_standoffs_m]
    else:
        lap_standoffs = [near, far] if int(spec.num_laps) > 1 else [near]
    u, v = span_unit_vectors(span_axis_deg)
    extent = (
        float(span_extent_m)
        if span_extent_m and span_extent_m > 0
        else 2.0 * far * max(spec.ellipse_aspect, 1.0)
    )
    sign = float(facade_sign) if facade_sign is not None else auto_facade_sign((cx, cy), corridor_xy, span_axis_deg)

    waypoints: List[Waypoint] = []
    for lap in range(int(spec.num_laps)):
        standoff = lap_standoffs[lap] if lap < len(lap_standoffs) else lap_standoffs[-1]
        alt = float(spec.altitude_m) + lap * 5.0
        waypoints.extend(
            _facade_pass_waypoints(
                (cx, cy),
                u=u,
                v=v,
                extent=extent,
                standoff=standoff,
                sign=sign,
                alt=alt,
                hfov_deg=spec.camera_hfov_deg,
                heading_overlap=spec.heading_overlap,
                label_prefix=f"span_l{lap}",
            )
        )
    return waypoints


def plan_span_facade_dual(
    center: np.ndarray,
    spec: SurveySpec,
    target: TargetSpec,
    span_axis_deg: float,
    *,
    corridor_xy: Tuple[float, float],
    span_extent_m: Optional[float] = None,
) -> List[Waypoint]:
    """
    Dual-side span facade for full-bridge photogrammetry.

    Default controllable passes (num_laps=2):
      lap0: SEARCH-corridor side at approach_standoff (near)
      lap1: opposite side at standoff_dist (far)

    With num_laps>=3, insert corridor-side far pass before opposite.
    Optional end-oblique viewpoints (span tips, ~45° off facade) when num_laps>=4.
    """
    cx, cy = float(center[0]), float(center[1])
    near = float(target.approach_standoff_dist_m or target.standoff_dist_m or spec.radius_m)
    far = float(target.standoff_dist_m or spec.radius_m)
    if spec.lap_standoffs_m:
        vals = [float(x) for x in spec.lap_standoffs_m]
        near = abs(vals[0])
        far = abs(vals[1]) if len(vals) > 1 else far

    u, v = span_unit_vectors(span_axis_deg)
    extent = (
        float(span_extent_m)
        if span_extent_m and span_extent_m > 0
        else 2.0 * far * max(spec.ellipse_aspect, 1.0)
    )
    corridor_sign = auto_facade_sign((cx, cy), corridor_xy, span_axis_deg)
    opposite_sign = -corridor_sign
    base_alt = float(spec.altitude_m)
    n_laps = int(spec.num_laps)

    # (sign, standoff, alt, label_prefix)
    passes: List[Tuple[float, float, float, str]] = [
        (corridor_sign, near, base_alt, "dual_c_near"),
    ]
    if n_laps >= 3:
        passes.append((corridor_sign, far, base_alt + 5.0, "dual_c_far"))
        passes.append((opposite_sign, far, base_alt + 10.0, "dual_o_far"))
    else:
        # controllable default: near corridor + opposite far
        passes.append((opposite_sign, far, base_alt + 5.0, "dual_o_far"))

    waypoints: List[Waypoint] = []
    for sign, standoff, alt, prefix in passes:
        pass_wps = _facade_pass_waypoints(
            (cx, cy),
            u=u,
            v=v,
            extent=extent,
            standoff=standoff,
            sign=sign,
            alt=alt,
            hfov_deg=spec.camera_hfov_deg,
            heading_overlap=spec.heading_overlap,
            label_prefix=prefix,
        )
        # Opposite facade: reverse along-span order so tip transit starts at the
        # same span tip where the previous pass ended (avoids ~500m wrap).
        if waypoints and prefix.startswith("dual_o_"):
            last = waypoints[-1]
            d0 = math.hypot(pass_wps[0].x - last.x, pass_wps[0].y - last.y)
            d1 = math.hypot(pass_wps[-1].x - last.x, pass_wps[-1].y - last.y)
            if d1 + 1.0 < d0:
                pass_wps = list(reversed(pass_wps))
                for i, w in enumerate(pass_wps):
                    w.label = f"{prefix}_i{i:02d}"
        waypoints.extend(pass_wps)

    if n_laps >= 4:
        # Span-tip oblique: ~45° between along-span and cross-span for tower depth.
        tip_standoff = far * 0.85
        tip_along = 0.5 * extent
        tip_alt = base_alt + 15.0
        for tip_i, s_sign in enumerate((-1.0, 1.0)):
            for side_i, side_sign in enumerate((corridor_sign, opposite_sign)):
                px = cx + s_sign * tip_along * u[0] + side_sign * tip_standoff * v[0]
                py = cy + s_sign * tip_along * u[1] + side_sign * tip_standoff * v[1]
                look_x = cx + s_sign * tip_along * 0.6 * u[0]
                look_y = cy + s_sign * tip_along * 0.6 * u[1]
                yaw = _yaw_toward(np.array([px, py]), np.array([look_x, look_y]))
                waypoints.append(
                    Waypoint(
                        x=float(px),
                        y=float(py),
                        z=tip_alt,
                        yaw_rad=yaw,
                        label=f"dual_oblique_t{tip_i}_s{side_i}",
                        hold_frames=3,
                    )
                )
    return waypoints


def plan_survey_views(spec: MissionSpec, mission_dir: Optional[Path] = None) -> List[Waypoint]:
    """Dispatch photogrammetry planners; requires bridge_centroid_xyz."""
    from aerial_inspect.survey.orbit_planner import plan_survey_waypoints, resolve_span_axis_deg

    pattern = str(spec.survey.pattern)
    if pattern not in ("span_facade", "span_facade_dual"):
        return plan_survey_waypoints(spec)

    if spec.bridge_centroid_xyz is None:
        raise ValueError(f"{pattern} requires bridge_centroid_xyz from replan-survey")

    center = np.asarray(spec.bridge_centroid_xyz, dtype=np.float64).reshape(3)
    span_axis = resolve_span_axis_deg(spec)
    corridor = tuple(spec.search.center_xy)
    detection = load_detection(mission_dir) if mission_dir else None
    extent = resolve_span_extent_m(spec.survey, detection)
    # If a deck centerline is pinned, snap midspan + axis + extent to it (主航道).
    cl_path = str(spec.survey.centerline_path or "").strip()
    if cl_path:
        root = Path(__file__).resolve().parents[2]
        cl = load_centerline_xy(root / cl_path if not Path(cl_path).is_file() else cl_path)
        if cl is not None and len(cl) >= 2:
            mid = cl[len(cl) // 2]
            center = np.array([float(mid[0]), float(mid[1]), float(center[2])], dtype=np.float64)
            d = cl[-1] - cl[0]
            span_axis = float(math.degrees(math.atan2(d[1], d[0])))
            extent = float(np.linalg.norm(d))
    if pattern == "span_facade_dual":
        return plan_span_facade_dual(
            center,
            spec.survey,
            spec.target,
            span_axis,
            corridor_xy=corridor,
            span_extent_m=extent,
        )
    return plan_span_facade(
        center,
        spec.survey,
        spec.target,
        span_axis,
        corridor_xy=corridor,
        span_extent_m=extent,
    )
