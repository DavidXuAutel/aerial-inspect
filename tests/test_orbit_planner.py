from aerial_inspect.mission.schema import MissionSpec, SearchArea, SurveySpec, TargetSpec
from aerial_inspect.survey.orbit_planner import plan_survey_waypoints, estimate_path_length_m


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
