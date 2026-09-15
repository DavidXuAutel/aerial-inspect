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


def main() -> int:
    p = argparse.ArgumentParser(description="Capture bridge scene check frame from AirSim")
    p.add_argument("--host", default=os.environ.get("AIRSIM_HOST", "127.0.0.1"))
    p.add_argument("--port", type=int, default=int(os.environ.get("AIRSIM_PORT", "41451")))
    p.add_argument("--x", type=float, default=-1020.0)
    p.add_argument("--y", type=float, default=-220.0)
    p.add_argument("--z", type=float, default=45.0)
    p.add_argument("--yaw-deg", type=float, default=0.0)
    p.add_argument("--camera", default=os.environ.get("AIRSIM_CAMERA", "front_custom"))
    p.add_argument("--vehicle", default=os.environ.get("AIRSIM_VEHICLE", "drone_1"))
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
        "requested_xyz": [args.x, args.y, args.z],
        "actual_ned": [float(pos.x_val), float(pos.y_val), float(pos.z_val)],
        "host": f"{args.host}:{args.port}",
        "camera": args.camera,
    }

    resp = None
    for cam in (args.camera, "front_custom", "0", "front_center"):
        req = airsim.ImageRequest(cam, airsim.ImageType.Scene, False, False)
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
    if resp.pixels_as_uint8:
        out.write_bytes(resp.pixels_as_uint8)
    else:
        out.write_bytes(base64.b64decode(resp.image_data_uint8))

    meta.update({"image": str(out), "width": int(resp.width), "height": int(resp.height), "bytes": out.stat().st_size})
    Path(args.meta_out).write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
