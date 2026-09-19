import math

import numpy as np

from scripts.colmap_known_pose import _intrinsics_from_hfov, traj_row_to_world_to_cam


def test_intrinsics_hfov80():
    fx, fy, cx, cy = _intrinsics_from_hfov(1920, 1080, 80.0)
    assert 1100 < fx < 1200
    assert abs(cx - 960) < 1e-6
    assert abs(cy - 540) < 1e-6


def test_world_to_cam_center_on_axis():
    row = {"pos": [-820.0, -76.0, 24.0], "yaw_rad": math.pi / 2, "pitch_rad": 0.0}
    r, t = traj_row_to_world_to_cam(row)
    c = -r.T @ t
    assert abs(c[0] + 820.0) < 1.0


def test_pitch_looks_down():
    row = {"pos": [-820.0, -76.0, 24.0], "yaw_rad": 0.0, "pitch_rad": -0.12}
    r, _t = traj_row_to_world_to_cam(row)
    forward = r[2]
    assert forward[2] < 0.0


def test_prefers_quat_over_yaw_pitch():
    row = {
        "pos": [0.0, 0.0, 10.0],
        "yaw_rad": 0.0,
        "pitch_rad": 0.0,
        "quat_wxyz": [0.9238795, 0.0, 0.3826834, 0.0],
    }
    r_quat, _ = traj_row_to_world_to_cam(row)
    row_no_quat = {"pos": row["pos"], "yaw_rad": 0.0, "pitch_rad": 0.0}
    r_yaw, _ = traj_row_to_world_to_cam(row_no_quat)
    assert not np.allclose(r_quat, r_yaw)


def test_cam_pose_overrides_vehicle():
    row = {
        "pos": [0.0, 0.0, 10.0],
        "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
        "cam_pos": [1.0, 2.0, 3.0],
        "cam_quat_wxyz": [0.9238795, 0.0, 0.3826834, 0.0],
    }
    _r, t = traj_row_to_world_to_cam(row)
    c = -_r.T @ t
    assert abs(c[0] - 1.0) < 1e-6
    assert abs(c[2] - 3.0) < 1e-6
