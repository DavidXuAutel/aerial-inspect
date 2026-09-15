import json
from pathlib import Path

from aerial_inspect.mission.centroid import estimate_centroid_from_traj
from aerial_inspect.mission.orchestrator import load_mission_yaml, plan_mission, replan_survey


FIXTURE = Path(__file__).parent / "fixtures" / "search_traj.jsonl"
FIXTURE_EVAL = Path(__file__).parent / "fixtures" / "search_traj_eval.jsonl"


def test_estimate_centroid_from_fixture():
    result = estimate_centroid_from_traj(FIXTURE, min_samples=3)
    assert result["status"] == "ok"
    x, y, z = result["centroid_xyz"]
    assert abs(x - (-1010.0)) < 1.0
    assert abs(y - (-215.0)) < 1.0
    assert abs(z - 42.0) < 1.0


def test_estimate_centroid_from_eval_traj_format():
    result = estimate_centroid_from_traj(FIXTURE_EVAL, min_samples=3)
    assert result["status"] == "ok"
    x, y, z = result["centroid_xyz"]
    assert abs(x - (-1010.0)) < 1.0
    assert abs(y - (-215.0)) < 1.0


def test_replan_survey_after_detection(tmp_path):
    root = Path(__file__).resolve().parents[1]
    spec = load_mission_yaml(root / "configs/missions/bridge_default.yaml")
    mission_dir = tmp_path / spec.mission_id
    plan_mission(spec, mission_dir, include_survey=False)
    assert not (mission_dir / "waypoints.json").is_file()

    result = estimate_centroid_from_traj(FIXTURE, min_samples=3)
    summary = replan_survey(mission_dir, tuple(result["centroid_xyz"]), source="test_fixture")
    assert summary["survey_status"] == "ready"
    assert summary["n_waypoints"] == 48
    assert (mission_dir / "waypoints.json").is_file()
    spec_after = json.loads((mission_dir / "mission_spec.json").read_text())
    assert spec_after["bridge_centroid_xyz"] is not None
