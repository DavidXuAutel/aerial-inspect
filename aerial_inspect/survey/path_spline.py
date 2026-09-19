"""Smooth closed-path interpolation through survey waypoints."""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

Point3 = Tuple[float, float, float]
BezierSeg = Tuple[Point3, Point3, Point3, Point3]


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def eval_cubic_bezier(p0: Point3, p1: Point3, p2: Point3, p3: Point3, t: float) -> Point3:
    u = 1.0 - t
    u2 = u * u
    u3 = u2 * u
    t2 = t * t
    t3 = t2 * t
    return (
        u3 * p0[0] + 3.0 * u2 * t * p1[0] + 3.0 * u * t2 * p2[0] + t3 * p3[0],
        u3 * p0[1] + 3.0 * u2 * t * p1[1] + 3.0 * u * t2 * p2[1] + t3 * p3[1],
        u3 * p0[2] + 3.0 * u2 * t * p1[2] + 3.0 * u * t2 * p2[2] + t3 * p3[2],
    )


def closed_cubic_bezier_segments(
    points: Sequence[Point3],
    tension: float = 1.0 / 6.0,
) -> List[BezierSeg]:
    """Catmull-Rom style control points for a closed loop through all waypoints."""
    n = len(points)
    if n < 3:
        raise ValueError("need at least 3 points for closed bezier path")
    segs: List[BezierSeg] = []
    for i in range(n):
        p0 = points[(i - 1) % n]
        p1 = points[i]
        p2 = points[(i + 1) % n]
        p3 = points[(i + 2) % n]
        c1 = (
            p1[0] + tension * (p2[0] - p0[0]),
            p1[1] + tension * (p2[1] - p0[1]),
            p1[2] + tension * (p2[2] - p0[2]),
        )
        c2 = (
            p2[0] - tension * (p3[0] - p1[0]),
            p2[1] - tension * (p3[1] - p1[1]),
            p2[2] - tension * (p3[2] - p1[2]),
        )
        segs.append((p1, c1, c2, p2))
    return segs


def sample_closed_bezier(
    points: Sequence[Point3],
    steps_between: int,
    tension: float = 1.0 / 6.0,
) -> List[Point3]:
    """Sample positions along a closed cubic-bezier loop (excludes duplicate start)."""
    if steps_between < 1:
        raise ValueError("steps_between must be >= 1")
    samples: List[Point3] = []
    for p0, p1, p2, p3 in closed_cubic_bezier_segments(points, tension=tension):
        for step in range(1, steps_between + 1):
            t = step / steps_between
            samples.append(eval_cubic_bezier(p0, p1, p2, p3, t))
    return samples


def path_length_m(points: Sequence[Point3]) -> float:
    total = 0.0
    for i in range(len(points)):
        x1, y1, z1 = points[i]
        x2, y2, z2 = points[(i + 1) % len(points)]
        total += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
    return total
