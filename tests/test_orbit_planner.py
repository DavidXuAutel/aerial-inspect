from aerial_inspect.mission.schema import MissionSpec, SearchArea, SurveySpec, TargetSpec
from aerial_inspect.survey.orbit_planner import (
    plan_clearance_orbit,
    plan_ellipse_orbit,
    plan_xy_orbit,
    plan_survey_waypoints,
    estimate_path_length_m,
    smooth_radii,
    smooth_xy,
)


def test_horizontal_orbit_count():
    spec = MissionSpec(
        mission_id="t",
        instruction="",
        search=SearchArea(center_xy=(0.0, 0.0)),
        target=TargetSpec(),
        survey=SurveySpec(num_laps=2, points_per_lap=12, radius_m=30.0),
        bridge_centroid_xyz=(100.0, 200.0, 40.0),
    )
    wps = plan_survey_waypoints(spec)
    assert len(wps) == 24
    assert estimate_path_length_m(wps) > 100.0
    for wp in wps:
        assert abs(wp.z - 40.0) <= 10.0 or wp.z == 43.0


def test_clearance_orbit_variable_radius():
    spec = MissionSpec(
        mission_id="t",
        instruction="",
        search=SearchArea(center_xy=(0.0, 0.0)),
        target=TargetSpec(),
        survey=SurveySpec(
            pattern="clearance_orbit",
            num_laps=1,
            points_per_lap=4,
            radius_m=20.0,
            orbit_start_deg=0.0,
        ),
        bridge_centroid_xyz=(10.0, 20.0, 15.0),
    )
    radii = [20.0, 25.0, 30.0, 22.0]
    wps = plan_clearance_orbit(
        __import__("numpy").array([10.0, 20.0, 15.0]),
        spec.survey,
        radii,
    )
    assert len(wps) == 4
    assert abs(wps[1].x - 10.0) < 1e-6
    assert abs(wps[1].y - 45.0) < 1e-6
    assert smooth_radii([20.0, 50.0, 20.0, 20.0], window=3)[1] == 30.0


def test_freeform_xy_orbit():
    spec = MissionSpec(
        mission_id="t",
        instruction="",
        search=SearchArea(center_xy=(0.0, 0.0)),
        target=TargetSpec(),
        survey=SurveySpec(
            pattern="clearance_orbit",
            num_laps=1,
            points_per_lap=4,
            orbit_start_deg=0.0,
        ),
        bridge_centroid_xyz=(10.0, 20.0, 15.0),
    )
    xy = [(30.0, 20.0), (10.0, 45.0), (-10.0, 20.0), (10.0, -5.0)]
    wps = plan_xy_orbit(__import__("numpy").array([10.0, 20.0, 15.0]), spec.survey, xy)
    assert len(wps) == 4
    assert abs(wps[0].x - 30.0) < 1e-6
    assert smooth_xy([(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)], window=3)[1] == (10.0, 0.0)


def test_ellipse_orbit_aspect():
    spec = MissionSpec(
        mission_id="t",
        instruction="",
        search=SearchArea(center_xy=(0.0, 0.0)),
        target=TargetSpec(),
        survey=SurveySpec(
            pattern="ellipse_orbit",
            num_laps=1,
            points_per_lap=4,
            radius_m=50.0,
            ellipse_aspect=2.0,
        ),
        bridge_centroid_xyz=(-950.0, -60.0, 55.0),
        bridge_span_axis_deg=0.0,
    )
    wps = plan_survey_waypoints(spec)
    assert len(wps) == 4
    xs = [w.x for w in wps]
    assert max(xs) - min(xs) > 90.0
    ys = [w.y for w in wps]
    assert max(ys) - min(ys) < 110.0


def test_ellipse_orbit_direct():
    import numpy as np

    spec = SurveySpec(num_laps=1, points_per_lap=8, radius_m=30.0, ellipse_aspect=2.5)
    wps = plan_ellipse_orbit(np.array([-950.0, -60.0, 55.0]), spec, span_axis_deg=0.0)
    assert len(wps) == 8


def test_facade_arc():
    spec = MissionSpec(
        mission_id="t",
        instruction="",
        search=SearchArea(center_xy=(0.0, 0.0)),
        target=TargetSpec(),
        survey=SurveySpec(pattern="facade_arc", num_laps=1, points_per_lap=8),
        bridge_centroid_xyz=(0.0, 0.0, 30.0),
    )
    wps = plan_survey_waypoints(spec)
    assert len(wps) == 8
