import math

from aerial_inspect.mission.schema import MissionSpec, SearchArea, SurveySpec, TargetSpec
from aerial_inspect.survey.view_planner import (
    along_track_spacing_m,
    auto_facade_sign,
    estimate_span_extent_m,
    plan_span_facade,
    plan_span_facade_dual,
    span_unit_vectors,
)


def test_along_track_spacing_positive():
    s = along_track_spacing_m(60.0, 70.0, 0.7)
    assert s > 10.0


def test_span_facade_faces_deck():
    spec = SurveySpec(
        pattern="span_facade",
        radius_m=60.0,
        altitude_m=85.0,
        num_laps=1,
        camera_hfov_deg=70.0,
        heading_overlap=0.7,
        ellipse_aspect=2.0,
    )
    target = TargetSpec(standoff_dist_m=60.0, standoff_height_m=25.0)
    center = __import__("numpy").array([935.0, -27.0, 94.0])
    wps = plan_span_facade(
        center,
        spec,
        target,
        180.0,
        corridor_xy=(1100.0, -50.0),
        span_extent_m=160.0,
        facade_sign=auto_facade_sign((935.0, -27.0), (1100.0, -50.0), 180.0),
    )
    assert 4 <= len(wps) <= 24
    ys = [w.y for w in wps]
    assert min(ys) < -27.0
    u, _ = span_unit_vectors(180.0)
    for w in wps:
        s = (w.x - 935.0) / u[0] if abs(u[0]) > 1e-6 else 0.0
        look_x = 935.0 + s * u[0]
        look_y = -27.0 + s * u[1]
        bearing = math.atan2(look_y - w.y, look_x - w.x)
        assert abs(bearing - w.yaw_rad) < 0.05


def test_estimate_span_extent_from_detection():
    det = {"span_axis": {"eigenvalues_xy": [400.0, 5000.0]}}
    assert estimate_span_extent_m(det, fallback_m=100.0) > 120.0


def test_span_facade_lap_standoffs_near_then_far():
    spec = SurveySpec(
        pattern="span_facade",
        radius_m=60.0,
        altitude_m=85.0,
        num_laps=2,
        camera_hfov_deg=70.0,
        heading_overlap=0.7,
        ellipse_aspect=2.0,
        lap_standoffs_m=(25.0, 60.0),
    )
    target = TargetSpec(approach_standoff_dist_m=25.0, standoff_dist_m=60.0, standoff_height_m=25.0)
    center = __import__("numpy").array([935.0, -27.0, 94.0])
    sign = auto_facade_sign((935.0, -27.0), (1100.0, -50.0), 180.0)
    wps = plan_span_facade(
        center,
        spec,
        target,
        180.0,
        corridor_xy=(1100.0, -50.0),
        span_extent_m=160.0,
        facade_sign=sign,
    )
    lap0 = [w for w in wps if w.label.startswith("span_l0")]
    lap1 = [w for w in wps if w.label.startswith("span_l1")]
    assert lap0 and lap1
    cross0 = abs(lap0[0].y - (-27.0))
    cross1 = abs(lap1[0].y - (-27.0))
    assert cross0 < cross1
    assert abs(cross0 - 25.0) < 2.0
    assert abs(cross1 - 60.0) < 2.0


def test_span_facade_dual_covers_both_sides():
    spec = SurveySpec(
        pattern="span_facade_dual",
        radius_m=60.0,
        altitude_m=85.0,
        num_laps=2,
        camera_hfov_deg=70.0,
        heading_overlap=0.55,
        ellipse_aspect=2.0,
        lap_standoffs_m=(25.0, 60.0),
    )
    target = TargetSpec(approach_standoff_dist_m=25.0, standoff_dist_m=60.0, standoff_height_m=25.0)
    center = __import__("numpy").array([935.0, -27.0, 94.0])
    wps = plan_span_facade_dual(
        center,
        spec,
        target,
        180.0,
        corridor_xy=(1100.0, -50.0),
        span_extent_m=160.0,
    )
    near = [w for w in wps if w.label.startswith("dual_c_near")]
    opp = [w for w in wps if w.label.startswith("dual_o_far")]
    assert near and opp
    assert 8 <= len(wps) <= 28
    # Corridor side of span@180° is south (y more negative for this corridor).
    assert all(w.y < -27.0 for w in near)
    assert all(w.y > -27.0 for w in opp)
    # Opposite pass should start near the tip where near pass ended.
    assert math.hypot(opp[0].x - near[-1].x, opp[0].y - near[-1].y) < math.hypot(
        opp[-1].x - near[-1].x, opp[-1].y - near[-1].y
    )
    # Every waypoint looks toward the deck centerline (same-x look for axis=180).
    for w in wps:
        look_x = w.x  # axis 180 → look along x at cy
        look_y = -27.0
        bearing = math.atan2(look_y - w.y, look_x - w.x)
        assert abs(bearing - w.yaw_rad) < 0.08
