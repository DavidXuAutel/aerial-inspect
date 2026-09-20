"""Main-channel survey must cover the full Humen span, not only the west tower."""
from pathlib import Path

import numpy as np
import yaml

from aerial_inspect.mission.schema import MissionSpec
from aerial_inspect.survey.orbit_planner import plan_survey_waypoints
from aerial_inspect.survey.view_planner import plan_survey_views, resolve_span_extent_m


ROOT = Path(__file__).resolve().parents[1]
YAML = ROOT / "configs/missions/bridge_humen.yaml"
CL = ROOT / "configs/sim/humen_centerline.json"


def test_humen_yaml_pins_main_channel():
    data = yaml.safe_load(YAML.read_text(encoding="utf-8"))
    assert data["bridge_span_axis_deg"] < 0  # deck runs SE (~-31°), not old +26°
    assert float(data["survey"]["span_extent_m"]) >= 800.0
    cx, cy, _ = data["bridge_centroid_xyz"]
    assert 2600.0 <= cx <= 2900.0
    assert cy < -100.0  # midspan, not west-tower pin near y≈-20


def test_humen_waypoints_cover_full_main_span():
    spec = MissionSpec.from_dict(yaml.safe_load(YAML.read_text(encoding="utf-8")))
    assert resolve_span_extent_m(spec.survey, None) >= 800.0
    wps = plan_survey_views(spec, mission_dir=None)
    assert len(wps) >= 40
    xs = np.array([w.x for w in wps])
    ys = np.array([w.y for w in wps])
    # Full main channel roughly x∈[2360,3080], y∈[-6,-435]
    assert xs.min() <= 2450.0
    assert xs.max() >= 2950.0
    assert ys.min() <= -350.0
    assert ys.max() >= -80.0
    # Must not collapse to the old west-only box x∈[2260,2545]
    assert xs.max() - xs.min() >= 600.0


def test_humen_centerline_file_exists_and_length():
    assert CL.is_file()
    import json

    pts = np.asarray(json.loads(CL.read_text(encoding="utf-8")), dtype=np.float64)
    assert len(pts) >= 50
    assert float(np.linalg.norm(pts[-1] - pts[0])) >= 800.0


def test_plan_survey_waypoints_uses_span_extent():
    spec = MissionSpec.from_dict(yaml.safe_load(YAML.read_text(encoding="utf-8")))
    wps = plan_survey_waypoints(spec)
    xs = [w.x for w in wps]
    assert max(xs) - min(xs) >= 600.0
