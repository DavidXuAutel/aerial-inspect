import json
import math

from aerial_inspect.mission.centroid import estimate_span_axis_from_traj


def test_span_axis_pca(tmp_path):
    traj = tmp_path / "traj.jsonl"
    rows = []
    for i in range(10):
        rows.append(
            {
                "pos": [-1000.0 + i * 5.0, -60.0, 85.0],
                "yaw": 0.0,
                "step_info": {
                    "tracker_state": "tracking",
                    "goal_rel": [30.0, 0.0, -5.0],
                    "using_fallback": False,
                },
            }
        )
    traj.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")
    result = estimate_span_axis_from_traj(traj, min_samples=5)
    assert result["status"] == "ok"
    axis = float(result["span_axis_deg"])
    assert abs(axis) < 20.0 or abs(abs(axis) - 180.0) < 20.0
