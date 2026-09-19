import math

from aerial_inspect.survey.survey_nav_path import (
    build_survey_segment_polyline,
    densify_polyline,
    yaw_along_polyline,
)


def test_densify_spacing():
    pts = densify_polyline([(0.0, 0.0, 10.0), (100.0, 0.0, 10.0)], spacing_m=20.0)
    assert len(pts) >= 6
    for a, b in zip(pts, pts[1:]):
        assert math.hypot(b[0] - a[0], b[1] - a[1]) <= 20.5


def test_same_side_is_nearly_straight():
    start = (935.0, -52.0, 85.0)
    goal = (900.0, -52.0, 85.0)
    pts = build_survey_segment_polyline(
        start,
        goal,
        centroid_xyz=(935.0, -27.0, 94.0),
        span_axis_deg=180.0,
        span_extent_m=160.0,
        spacing_m=15.0,
    )
    assert pts[0] == start
    assert abs(pts[-1][0] - goal[0]) < 1e-6
    # No big detour north of centroid for same-side.
    assert all(p[1] < -27.0 for p in pts)


def test_opposite_side_goes_around_tip():
    start = (935.0, -52.0, 85.0)  # south / corridor
    goal = (935.0, 33.0, 90.0)  # north / opposite
    pts = build_survey_segment_polyline(
        start,
        goal,
        centroid_xyz=(935.0, -27.0, 94.0),
        span_axis_deg=180.0,
        span_extent_m=160.0,
        spacing_m=20.0,
        tip_margin_m=40.0,
    )
    assert len(pts) > 8
    # Must swing in x away from centroid (around tip), not through deck center.
    xs = [p[0] for p in pts]
    assert max(abs(x - 935.0) for x in xs) > 50.0
    assert pts[-1][1] > 0.0
    # Cross from south to north only after leaving the mid-span x band.
    crossed = False
    for p in pts:
        if abs(p[0] - 935.0) > 50.0 and p[1] > -10.0:
            crossed = True
            break
    assert crossed


def test_opposite_picks_shorter_tip():
    # Start/goal both near +x end → should swing around +x tip, not −x.
    start = (1000.0, -52.0, 85.0)
    goal = (1010.0, 33.0, 90.0)
    pts = build_survey_segment_polyline(
        start,
        goal,
        centroid_xyz=(935.0, -27.0, 94.0),
        span_axis_deg=180.0,
        span_extent_m=160.0,
        spacing_m=20.0,
        tip_margin_m=40.0,
    )
    xs = [p[0] for p in pts]
    assert max(xs) > 1000.0
    assert min(xs) > 900.0  # should not detour to the far west tip


def test_yaw_along_polyline():
    yaws = yaw_along_polyline([(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.0, 10.0, 0.0)])
    assert abs(yaws[0]) < 1e-6
    assert abs(yaws[1] - math.pi / 2) < 1e-6
