"""Build Phase-2-style legal polylines between survey capture waypoints."""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

from aerial_inspect.survey.view_planner import span_unit_vectors

Vec3 = Tuple[float, float, float]


def _as3(p: Sequence[float]) -> Vec3:
    return (float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0)


def _dist_xy(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _cross_sign(
    xy: Sequence[float],
    centroid_xy: Tuple[float, float],
    span_axis_deg: float,
) -> float:
    _, v = span_unit_vectors(span_axis_deg)
    dx = float(xy[0]) - centroid_xy[0]
    dy = float(xy[1]) - centroid_xy[1]
    dot = dx * v[0] + dy * v[1]
    if abs(dot) < 1e-6:
        return 0.0
    return 1.0 if dot > 0.0 else -1.0


def densify_polyline(
    points: Sequence[Sequence[float]],
    *,
    spacing_m: float = 15.0,
) -> List[Vec3]:
    """Insert intermediate points so consecutive spacing ≈ spacing_m."""
    if len(points) < 2:
        return [_as3(p) for p in points]
    out: List[Vec3] = [_as3(points[0])]
    for i in range(1, len(points)):
        a = _as3(out[-1])
        b = _as3(points[i])
        seg = _dist_xy(a, b)
        n = max(1, int(math.ceil(seg / max(spacing_m, 1.0))))
        for k in range(1, n + 1):
            t = k / n
            out.append(
                (
                    a[0] + t * (b[0] - a[0]),
                    a[1] + t * (b[1] - a[1]),
                    a[2] + t * (b[2] - a[2]),
                )
            )
    return out


def build_survey_segment_polyline(
    start: Sequence[float],
    goal: Sequence[float],
    *,
    centroid_xyz: Sequence[float],
    span_axis_deg: float,
    span_extent_m: float = 160.0,
    spacing_m: float = 15.0,
    tip_margin_m: float = 40.0,
    transit_alt_boost_m: float = 20.0,
) -> List[Vec3]:
    """
    Polyline from start→goal for Phase-2 long_eval.

    Same facade side: densified straight segment (open-water / free space).
    Opposite sides: route around the nearer span tip at raised altitude, then
    densify — avoids flying through the deck with a two-point straight line.
    """
    s = _as3(start)
    g = _as3(goal)
    cx, cy = float(centroid_xyz[0]), float(centroid_xyz[1])
    u, v = span_unit_vectors(span_axis_deg)
    sign_s = _cross_sign(s, (cx, cy), span_axis_deg)
    sign_g = _cross_sign(g, (cx, cy), span_axis_deg)

    anchors: List[Vec3] = [s]
    opposite = sign_s != 0.0 and sign_g != 0.0 and sign_s != sign_g
    if opposite:
        # Tip just beyond the farther endpoint (capped at span half + margin).
        half_cap = 0.5 * float(span_extent_m) + float(tip_margin_m)
        s_along = (s[0] - cx) * u[0] + (s[1] - cy) * u[1]
        g_along = (g[0] - cx) * u[0] + (g[1] - cy) * u[1]
        half = min(half_cap, max(abs(s_along), abs(g_along)) + float(tip_margin_m))
        # Mid-span opposite still needs to clear towers/cables.
        half = max(half, min(half_cap, 0.45 * float(span_extent_m)))
        tip_candidates = (half, -half)
        tip_s = min(
            tip_candidates,
            key=lambda t: abs(s_along - t) + abs(g_along - t),
        )
        tip_xy = (cx + tip_s * u[0], cy + tip_s * u[1])
        z_hi = max(s[2], g[2]) + float(transit_alt_boost_m)
        standoff_s = abs((s[0] - cx) * v[0] + (s[1] - cy) * v[1])
        standoff_g = abs((g[0] - cx) * v[0] + (g[1] - cy) * v[1])
        tip_clear_m = max(standoff_s, standoff_g, 40.0)
        # Two-corner tip (start-side → goal-side); skip outboard dogleg to
        # keep Phase-2 tip transit under ~200m on Humen-scale spans.
        anchors.append(
            (
                tip_xy[0] + sign_s * tip_clear_m * v[0],
                tip_xy[1] + sign_s * tip_clear_m * v[1],
                z_hi,
            )
        )
        anchors.append(
            (
                tip_xy[0] + sign_g * tip_clear_m * v[0],
                tip_xy[1] + sign_g * tip_clear_m * v[1],
                z_hi,
            )
        )
    anchors.append(g)
    return densify_polyline(anchors, spacing_m=spacing_m)


def yaw_along_polyline(points: Sequence[Sequence[float]]) -> List[float]:
    yaws: List[float] = []
    for i, p in enumerate(points):
        if i + 1 < len(points):
            nxt = points[i + 1]
            yaws.append(math.atan2(float(nxt[1]) - float(p[1]), float(nxt[0]) - float(p[0])))
        else:
            yaws.append(yaws[-1] if yaws else 0.0)
    return yaws
