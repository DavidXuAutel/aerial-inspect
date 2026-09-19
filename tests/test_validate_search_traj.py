import json
from pathlib import Path

from scripts.validate_search_traj import validate_search_traj


def test_validate_rejects_tracker_without_det(tmp_path: Path):
    traj = tmp_path / "traj.jsonl"
    rows = [
        {"step": i, "pos": [0, 0, 0], "tracker_state": "tracking", "det_hit": False, "goal_rel": [1, 0, 0, 80.0]}
        for i in range(20)
    ]
    traj.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")
    report = validate_search_traj(traj)
    assert report["status"] == "fail"


def test_validate_rejects_short_corridor(tmp_path: Path):
    traj = tmp_path / "traj.jsonl"
    rows = [
        {
            "step": i,
            "pos": [775.0 + i * 0.5, -50, 95],
            "tracker_state": "tracking",
            "det_hit": True,
            "bridge_goal_rel": [1, 0, 0, 120.0],
        }
        for i in range(20)
    ]
    traj.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")
    report = validate_search_traj(traj, min_x_span_m=250.0)
    assert report["status"] == "fail"


def test_validate_accepts_det_hits(tmp_path: Path):
    traj = tmp_path / "traj.jsonl"
    rows = [
        {
            "step": i,
            "pos": [1000 + i, -50, 95],
            "tracker_state": "tracking",
            "det_hit": True,
            "goal_rel": [1, 0, 0, 120.0],
        }
        for i in range(20)
    ]
    traj.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")
    report = validate_search_traj(traj)
    assert report["status"] == "ok"
    assert report["n_det_hit"] == 20


def test_centroid_require_det_hit(tmp_path: Path):
    from aerial_inspect.mission.centroid import estimate_centroid_from_traj

    traj = tmp_path / "t.jsonl"
    rows = [
        {
            "step": 1,
            "pos": [-1020.0, -220.0, 45.0],
            "yaw": 0.0,
            "tracker_state": "tracking",
            "det_hit": True,
            "goal_rel": [10.0, 5.0, -3.0, 80.0],
            "using_fallback": False,
        }
        for _ in range(5)
    ]
    traj.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")
    ok = estimate_centroid_from_traj(traj, min_samples=3, require_det_hit=True, min_goal_rel_dist_m=40.0)
    assert ok["status"] == "ok"
    no_det = estimate_centroid_from_traj(traj, min_samples=3, require_det_hit=True)
    traj.write_text(
        "\n".join(
            json.dumps({**rows[0], "det_hit": False}) for _ in range(5)
        ),
        encoding="utf-8",
    )
    fail = estimate_centroid_from_traj(traj, min_samples=3, require_det_hit=True)
    assert fail["status"] == "failed"
