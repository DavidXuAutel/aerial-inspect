"""Reliable AirSim pose teleport helpers for survey capture."""
from __future__ import annotations

import math
import time
from typing import Any, Sequence

import numpy as np


def yaw_toward_xy(
    from_xy: tuple[float, float],
    to_xy: tuple[float, float],
) -> float:
    dx, dy = to_xy[0] - from_xy[0], to_xy[1] - from_xy[1]
    return float(math.atan2(dy, dx))


def resolve_deck_look_at(spec: dict) -> list[float]:
    """Bridge deck aim point from SEARCH centroid (xy) and deck-level z."""
    import os

    centroid = spec.get("bridge_centroid_xyz") or spec.get("bridge_centroid")
    if not centroid or len(centroid) < 3:
        raise ValueError("bridge_centroid_xyz required")
    target = spec.get("target") or {}
    standoff_h = float(
        os.environ.get(
            "SURVEY_STANDOFF_HEIGHT_M",
            target.get("standoff_height_m", 25.0),
        )
    )
    return [
        float(centroid[0]),
        float(centroid[1]),
        float(centroid[2]) - standoff_h,
    ]


def facade_look_at_for_pos(
    pos: Sequence[float],
    spec: dict,
) -> list[float]:
    """Aim at the span centerline at the same along-track station (not a single centroid).

    Dual-facade mid-span cameras that look at one centroid point see empty water;
    projecting onto the deck axis keeps the bridge in frame.
    """
    centroid = spec.get("bridge_centroid_xyz") or spec.get("bridge_centroid")
    if not centroid or len(centroid) < 3:
        raise ValueError("bridge_centroid_xyz required")
    cx, cy = float(centroid[0]), float(centroid[1])
    deck = resolve_deck_look_at(spec)
    deck_z = float(deck[2])
    survey = spec.get("survey") or {}
    span_deg = float(
        spec.get("bridge_span_axis_deg")
        if spec.get("bridge_span_axis_deg") is not None
        else survey.get("span_axis_deg", 0.0)
    )
    rad = math.radians(span_deg)
    ux, uy = math.cos(rad), math.sin(rad)
    s = (float(pos[0]) - cx) * ux + (float(pos[1]) - cy) * uy
    return [cx + s * ux, cy + s * uy, deck_z]


def survey_capture_yaw_pitch(
    pos: Sequence[float],
    look_at: Sequence[float],
) -> tuple[float, float]:
    """Per-waypoint aim: 3D look-at when camera mount pitch is 0 (Humen), else yaw-only."""
    import os

    mount_deg = float(os.environ.get("SURVEY_CAMERA_MOUNT_PITCH", "0"))
    mode = os.environ.get("SURVEY_AIM_MODE", "deck_lookat" if abs(mount_deg) < 0.5 else "yaw_fixed")
    if mode == "deck_lookat":
        return look_at_yaw_pitch(pos, look_at)
    return survey_yaw_fixed_pitch(pos, look_at[:2] if len(look_at) >= 2 else look_at)


def survey_yaw_fixed_pitch(
    pos: Sequence[float],
    target_xy: Sequence[float],
    *,
    pitch_deg: float | None = None,
) -> tuple[float, float]:
    """Horizontal yaw toward target; fixed body pitch (camera mount handles depression)."""
    import os

    px, py = float(pos[0]), float(pos[1])
    tx, ty = float(target_xy[0]), float(target_xy[1])
    dx, dy = tx - px, ty - py
    yaw = float(math.atan2(dy, dx)) if abs(dx) + abs(dy) > 1e-6 else 0.0
    if pitch_deg is None:
        pitch_deg = float(os.environ.get("SURVEY_BODY_PITCH", os.environ.get("BRIDGE_CAMERA_PITCH", "-20")))
    return yaw, math.radians(pitch_deg)


def look_at_yaw_pitch(
    pos: Sequence[float],
    target: Sequence[float],
) -> tuple[float, float]:
    """Horizontal yaw + pitch (rad) so body forward points at ``target`` (Z-up world)."""
    px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
    tx, ty, tz = float(target[0]), float(target[1]), float(target[2])
    dx, dy, dz = tx - px, ty - py, tz - pz
    horiz = math.hypot(dx, dy)
    yaw = float(math.atan2(dy, dx)) if horiz > 1e-6 else 0.0
    pitch = float(math.atan2(dz, horiz)) if horiz > 1e-6 else float(-math.pi / 2 if dz < 0 else math.pi / 2)
    return yaw, pitch


def rotation_body_to_world(yaw: float, pitch: float) -> np.ndarray:
    """Body forward/right/down -> world (X north, Y east, Z up). Pitch<0 looks down."""
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
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
    return np.stack([forward, right, down], axis=1)


def quat_wxyz_from_yaw_pitch(yaw: float, pitch: float) -> tuple[float, float, float, float]:
    """Quaternion (w,x,y,z) matching AirSim ``to_quaternion(pitch, roll, yaw)`` with roll=0."""
    import airsim

    q = airsim.to_quaternion(float(pitch), 0.0, float(yaw))
    return (
        float(q.w_val),
        float(q.x_val),
        float(q.y_val),
        float(q.z_val),
    )


def read_pose_xyz(client: Any, vk: dict[str, str]) -> tuple[float, float, float]:
    pos = client.simGetVehiclePose(**vk).position
    return float(pos.x_val), float(pos.y_val), -float(pos.z_val)


def read_camera_pose_zup(
    client: Any,
    camera_name: str,
    vk: dict[str, str],
) -> tuple[list[float], list[float]]:
    """Camera optical center pose in survey coords (X north, Y east, Z up)."""
    info = client.simGetCameraInfo(camera_name, **vk)
    pos = info.pose.position
    q = info.pose.orientation
    cam_pos = [float(pos.x_val), float(pos.y_val), -float(pos.z_val)]
    cam_quat = [float(q.w_val), float(q.x_val), float(q.y_val), float(q.z_val)]
    return cam_pos, cam_quat


def set_pose_verified(
    client: Any,
    vk: dict[str, str],
    x: float,
    y: float,
    z: float,
    yaw: float,
    *,
    pitch: float = 0.0,
    tol_xy: float = 1.5,
    tol_z: float = 6.0,
    retries: int = 8,
    settle_s: float = 0.45,
) -> tuple[float, float, float, float, float, tuple[float, float, float, float]]:
    """Teleport and verify XY/Z before capture. Returns actual pos, yaw, pitch, quat."""
    import airsim

    pose = airsim.Pose(
        airsim.Vector3r(x, y, -z),
        airsim.to_quaternion(float(pitch), 0.0, float(yaw)),
    )
    last = (float("nan"), float("nan"), float("nan"))
    last_err = float("inf")
    for attempt in range(retries):
        if hasattr(client, "simPause"):
            client.simPause(True)
        client.simSetVehiclePose(pose, True, **vk)
        time.sleep(settle_s)
        if hasattr(client, "simPause"):
            client.simPause(False)
        time.sleep(0.1)
        ax, ay, az = read_pose_xyz(client, vk)
        last = (ax, ay, az)
        err_xy = math.hypot(ax - x, ay - y)
        err_z = abs(az - z)
        last_err = err_xy
        if err_xy <= tol_xy and err_z <= tol_z:
            qw, qx, qy, qz = quat_wxyz_from_yaw_pitch(yaw, pitch)
            return ax, ay, az, yaw, pitch, (qw, qx, qy, qz)
    raise RuntimeError(
        f"pose not reached target=({x:.1f},{y:.1f},{z:.1f}) "
        f"last=({last[0]:.1f},{last[1]:.1f},{last[2]:.1f}) err_xy={last_err:.2f}m"
    )


class SimPauseSession:
    """Keep simulation paused for the whole capture loop."""

    def __init__(self, client: Any) -> None:
        self._client = client
        self._active = False

    def __enter__(self) -> SimPauseSession:
        if hasattr(self._client, "simPause"):
            self._client.simPause(True)
            self._active = True
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._active:
            self._client.simPause(False)
