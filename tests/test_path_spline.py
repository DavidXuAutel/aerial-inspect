from aerial_inspect.survey.path_spline import (
    closed_cubic_bezier_segments,
    eval_cubic_bezier,
    sample_closed_bezier,
)


def test_bezier_endpoints():
    pts = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.0, 10.0, 0.0)]
    segs = closed_cubic_bezier_segments(pts)
    p0, c1, c2, p3 = segs[0]
    assert p0 == pts[0]
    assert p3 == pts[1]
    assert eval_cubic_bezier(p0, c1, c2, p3, 0.0) == p0
    assert eval_cubic_bezier(p0, c1, c2, p3, 1.0) == p3


def test_closed_bezier_sample_count():
    pts = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.0, 10.0, 0.0), (0.0, 10.0, 0.0)]
    samples = sample_closed_bezier(pts, steps_between=4)
    assert len(samples) == len(pts) * 4
