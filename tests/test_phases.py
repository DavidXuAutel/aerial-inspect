import math

from aerial_inspect.mission.phases import build_phase_plan, compute_approach_goal
from aerial_inspect.mission.schema import MissionSpec, SearchArea, SurveySpec, TargetSpec


def test_approach_goal_standoff_from_centroid():
    spec = MissionSpec(
        mission_id="t",
        instruction="",
        search=SearchArea(center_xy=(-1020.0, -220.0)),
        target=TargetSpec(standoff_dist_m=30.0, standoff_height_m=3.0),
        survey=SurveySpec(altitude_m=45.0),
        bridge_centroid_xyz=(-1010.0, -215.0, 42.0),
    )
    gx, gy, gz = compute_approach_goal(spec)
    dist = math.hypot(gx - (-1010.0), gy - (-215.0))
    assert abs(dist - 30.0) < 0.5
    assert gz >= 45.0


def test_phase_plan_has_search_and_approach():
    spec = MissionSpec(
        mission_id="bridge_river_001",
        instruction="find bridge",
        search=SearchArea(center_xy=(0.0, 0.0), radius_m=100.0, altitude_m=40.0),
        target=TargetSpec(visual_prompt="bridge"),
        survey=SurveySpec(),
        bridge_centroid_xyz=(50.0, 0.0, 35.0),
    )
    plan = build_phase_plan(spec)
    assert plan["visual_prompt"] == "bridge"
    assert plan["search"]["max_steps"] == 400
    assert "goal" in plan["approach"]
    assert plan["bridge_centroid_xyz"] == [50.0, 0.0, 35.0]
