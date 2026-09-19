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


def test_nearest_lock_recorded(tmp_path: Path):
    traj = tmp_path / "mixed_dist.jsonl"
    rows = [
        {
            "step": 0,
            "pos": [900.0, -50.0, 95.0],
            "yaw": 0.0,
            "det_hit": True,
            "bridge_goal_rel": [5.0, 0.0, 0.0, 80.0],
            "using_fallback": False,
        },
        {
            "step": 1,
            "pos": [894.0, -48.0, 95.0],
            "yaw": 0.0,
            "det_hit": True,
            "bridge_goal_rel": [2.0, 0.0, 0.0, 2.0],
            "using_fallback": False,
        },
    ]
    traj.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")
    result = estimate_centroid_from_traj(traj, min_samples=1, min_goal_rel_dist_m=40.0)
    assert result["status"] == "ok"
    assert result["nearest_lock"]["goal_rel_dist_m"] == 2.0
    assert result["nearest_lock"]["drone_xyz"][0] == 894.0


def test_centroid_from_bridge_goal_rel_searching(tmp_path: Path):
    traj = tmp_path / "area_priority.jsonl"
    rows = [
        {
            "step": i,
            "pos": [900.0 + i * 2.0, -50.0, 95.0],
            "yaw": 0.785398,
            "tracker_state": "searching",
            "det_hit": True,
            "bridge_goal_rel": [10.0, 0.0, -5.0, 80.0],
            "using_fallback": False,
        }
        for i in range(12)
    ]
    traj.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")
    result = estimate_centroid_from_traj(
        traj, min_samples=10, require_det_hit=True, min_goal_rel_dist_m=40.0
    )
    assert result["status"] == "ok"
    assert result["n_samples"] == 12


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
