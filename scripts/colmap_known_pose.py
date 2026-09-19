"""Export AirSim traj.jsonl as COLMAP text model for point_triangulator."""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict

import numpy as np


def _image_size(images_dir: Path) -> tuple[int, int]:
    from PIL import Image

    first = sorted(images_dir.glob("*.jpg"))[0]
    w, h = Image.open(first).size
    return w, h


def _intrinsics_from_hfov(width: int, height: int, hfov_deg: float) -> tuple[float, float, float, float]:
    hfov = math.radians(hfov_deg)
    fx = width / (2.0 * math.tan(hfov / 2.0))
    vfov = 2.0 * math.atan(math.tan(hfov / 2.0) * height / width)
    fy = height / (2.0 * math.tan(vfov / 2.0))
    return fx, fy, width / 2.0, height / 2.0


def _rotation_from_yaw_pitch(yaw_rad: float, pitch_rad: float) -> np.ndarray:
    """Camera axes (right, down, forward) from horizontal yaw + pitch (Z-up world)."""
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    forward = np.array([cp * cy, cp * sy, sp], dtype=np.float64)
    world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    right = np.cross(forward, world_up)
    rn = float(np.linalg.norm(right))
    if rn < 1e-6:
        right = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    else:
        right /= rn
    down = np.cross(forward, right)
    down /= max(float(np.linalg.norm(down)), 1e-9)
    return np.stack([right, down, forward], axis=0)


def _rotation_from_quat_wxyz(quat: list[float]) -> np.ndarray:
    """AirSim body (X fwd, Y right, Z down) -> camera (right, down, forward) rows."""
    qw, qx, qy, qz = [float(x) for x in quat]
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    rot = np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=np.float64,
    )
    forward = rot @ np.array([1.0, 0.0, 0.0])
    right = rot @ np.array([0.0, 1.0, 0.0])
    down = rot @ np.array([0.0, 0.0, 1.0])
    return np.stack([right, down, forward], axis=0)


def rotate_camera_pitch(r: np.ndarray, pitch_rad: float) -> np.ndarray:
    """Extra pitch (rad) about camera right axis (+ = look down)."""
    if abs(pitch_rad) < 1e-9:
        return r
    right = r[0]
    forward = r[2]
    down = r[1]
    c, s = math.cos(pitch_rad), math.sin(pitch_rad)
    new_forward = c * forward + s * down
    new_down = -s * forward + c * down
    return np.stack([right, new_down, new_forward], axis=0)


def traj_row_to_world_to_cam(
    row: Dict[str, Any],
    *,
    pitch_offset_rad: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    """AirSim survey pose (X north, Y east, Z up) -> R,t with X_cam = R @ X_world + t."""
    if row.get("cam_pos") and row.get("cam_quat_wxyz"):
        cx, cy, cz = [float(x) for x in row["cam_pos"][:3]]
        r = _rotation_from_quat_wxyz(row["cam_quat_wxyz"])
    elif row.get("quat_wxyz"):
        pos = row["pos"]
        cx, cy, cz = float(pos[0]), float(pos[1]), float(pos[2])
        r = _rotation_from_quat_wxyz(row["quat_wxyz"])
    else:
        pos = row["pos"]
        cx, cy, cz = float(pos[0]), float(pos[1]), float(pos[2])
        mount = math.radians(float(row.get("camera_mount_pitch_deg", 0.0)))
        yaw = float(row.get("yaw_rad", 0.0))
        pitch = float(row.get("pitch_rad", 0.0)) + mount
        r = _rotation_from_yaw_pitch(yaw, pitch)

    if pitch_offset_rad:
        r = rotate_camera_pitch(r, pitch_offset_rad)

    c = np.array([cx, cy, cz], dtype=np.float64)
    t = -r @ c
    return r, t


def _quat_wxyz_from_rot(r: np.ndarray) -> tuple[float, float, float, float]:
    m = r.astype(np.float64)
    tr = float(m[0, 0] + m[1, 1] + m[2, 2])
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    return float(qw), float(qx), float(qy), float(qz)


def load_traj_poses(traj_path: Path) -> dict[str, dict]:
    poses: dict[str, dict] = {}
    for line in traj_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        poses[str(row["frame"])] = row
    return poses


def write_colmap_text_model(
    traj_path: Path,
    images_dir: Path,
    out_dir: Path,
    hfov_deg: float = 80.0,
) -> int:
    """Write cameras.txt / images.txt / points3D.txt from known poses."""
    poses = load_traj_poses(traj_path)
    width, height = _image_size(images_dir)
    fx, fy, cx, cy = _intrinsics_from_hfov(width, height, hfov_deg)
    out_dir.mkdir(parents=True, exist_ok=True)

    cameras_path = out_dir / "cameras.txt"
    images_path = out_dir / "images.txt"
    points_path = out_dir / "points3D.txt"

    with cameras_path.open("w", encoding="utf-8") as f:
        f.write("# Camera list with one line of data per camera:\n")
        f.write("# CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
        f.write(f"1 PINHOLE {width} {height} {fx} {fy} {cx} {cy}\n")

    image_lines: list[str] = []
    for image_id, name in enumerate(sorted(poses.keys()), start=1):
        row = poses[name]
        r, t = traj_row_to_world_to_cam(row)
        qw, qx, qy, qz = _quat_wxyz_from_rot(r.T)
        tx, ty, tz = float(t[0]), float(t[1]), float(t[2])
        image_lines.append(
            f"{image_id} {qw} {qx} {qy} {qz} {tx} {ty} {tz} 1 {name}\n"
        )
        image_lines.append("\n")

    images_path.write_text("".join(image_lines), encoding="utf-8")
    points_path.write_text("", encoding="utf-8")
    return len(poses)
