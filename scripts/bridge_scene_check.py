#!/usr/bin/env python3
"""Teleport to bridge search coords on local AirSim and save a verification frame."""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import sys
import time
from pathlib import Path

import yaml


def _spawn_defaults(scene_config: Path) -> tuple[float, float, float, float, str, str]:
    if not scene_config.is_file():
        return -1020.0, -220.0, 45.0, 0.0, "front_custom", "drone_1"
    data = yaml.safe_load(scene_config.read_text(encoding="utf-8")) or {}
    xyz = data.get("spawn_xyz") or [-1020.0, -220.0, 45.0]
    yaw = float(data.get("spawn_yaw_deg", 0.0))
    camera = str(data.get("camera", os.environ.get("AIRSIM_CAMERA", "front_custom")))
    vehicle = str(data.get("vehicle", os.environ.get("AIRSIM_VEHICLE", "drone_1")))
    return float(xyz[0]), float(xyz[1]), float(xyz[2]), yaw, camera, vehicle


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    scene_cfg = Path(
        os.environ.get("AIRSIM_SCENE_CONFIG", "configs/sim/bridge_scene.yaml")
    )
    if not scene_cfg.is_absolute():
        scene_cfg = repo / scene_cfg
    dx, dy, dz, dyaw, default_cam, default_vehicle = _spawn_defaults(scene_cfg)
    scene_data = (
        yaml.safe_load(scene_cfg.read_text(encoding="utf-8")) if scene_cfg.is_file() else {}
    ) or {}
    default_port = int(scene_data.get("airsim_port", os.environ.get("AIRSIM_PORT", "41451")))

    p = argparse.ArgumentParser(description="Capture bridge scene check frame from AirSim")
    p.add_argument("--scene-config", default=str(scene_cfg))
    p.add_argument("--host", default=os.environ.get("AIRSIM_HOST", "127.0.0.1"))
    p.add_argument("--port", type=int, default=default_port)
    p.add_argument("--x", type=float, default=dx)
    p.add_argument("--y", type=float, default=dy)
    p.add_argument("--z", type=float, default=dz)
    p.add_argument("--yaw-deg", type=float, default=dyaw)
    p.add_argument("--camera", default=default_cam)
    p.add_argument("--vehicle", default=default_vehicle)
    p.add_argument("--out", default="artifacts/bridge_scene_check.jpg")
    p.add_argument("--meta-out", default="artifacts/bridge_scene_check.json")
    args = p.parse_args()

    import airsim  # lazy: only on sim host

    client = airsim.MultirotorClient(ip=args.host, port=args.port)
    client.confirmConnection()
    vk = {"vehicle_name": args.vehicle}
    client.enableApiControl(True, **vk)
    client.armDisarm(True, **vk)

    yaw = math.radians(args.yaw_deg)
    pose = airsim.Pose(
        airsim.Vector3r(float(args.x), float(args.y), -float(args.z)),
        airsim.to_quaternion(0.0, 0.0, yaw),
    )
    if hasattr(client, "simPause"):
        client.simPause(True)
    client.simSetVehiclePose(pose, True, **vk)
    time.sleep(0.5)
    if hasattr(client, "simPause"):
        client.simPause(False)

    pos = client.simGetVehiclePose(**vk).position
    meta: dict = {
        "scene_config": str(args.scene_config),
        "requested_xyz": [args.x, args.y, args.z],
        "actual_ned": [float(pos.x_val), float(pos.y_val), float(pos.z_val)],
        "host": f"{args.host}:{args.port}",
        "camera": args.camera,
    }

    resp = None
    for cam in (args.camera, "front_custom", "0", "front_center"):
        req = airsim.ImageRequest(cam, airsim.ImageType.Scene, False, True)
        try:
            resp = client.simGetImages([req], **vk)[0]
            if resp.width > 0 and resp.height > 0:
                meta["camera_used"] = cam
                break
        except Exception as exc:  # noqa: BLE001
            meta.setdefault("camera_errors", []).append(f"{cam}: {exc!r}")
            resp = None

    if resp is None or resp.width <= 0:
        print(json.dumps(meta, indent=2))
        return 1

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    raw = bytes(resp.image_data_uint8)
    if not raw:
        raw = base64.b64decode(resp.image_data_uint8)
    out.write_bytes(raw)

    meta.update({"image": str(out), "width": int(resp.width), "height": int(resp.height), "bytes": out.stat().st_size})
    Path(args.meta_out).write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
