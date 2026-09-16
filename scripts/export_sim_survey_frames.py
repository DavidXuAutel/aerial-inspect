#!/usr/bin/env python3
"""Export JPEG frames at survey waypoints for offline COLMAP (AirSim on 84)."""
from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser(description="Export survey waypoint frames from AirSim")
    p.add_argument("mission_dir", help="artifacts/<mission_id>")
    p.add_argument("--out", help="capture output dir (default <mission_dir>/capture_survey)")
    p.add_argument("--host", default=os.environ.get("AIRSIM_HOST", "127.0.0.1"))
    p.add_argument("--port", type=int, default=int(os.environ.get("AIRSIM_PORT", "41451")))
    p.add_argument("--camera", default=os.environ.get("AIRSIM_CAMERA", "front_custom"))
    p.add_argument("--vehicle", default=os.environ.get("AIRSIM_VEHICLE", "drone_1"))
    p.add_argument("--max-waypoints", type=int, default=0, help="0 = all")
    args = p.parse_args()

    mission_dir = Path(args.mission_dir).resolve()
    wps = json.loads((mission_dir / "waypoints.json").read_text(encoding="utf-8"))
    if args.max_waypoints > 0:
        wps = wps[: args.max_waypoints]

    out = Path(args.out).resolve() if args.out else mission_dir / "capture_survey"
    frames = out / "frames"
    frames.mkdir(parents=True, exist_ok=True)
    traj_path = out / "traj.jsonl"

    import airsim  # lazy

    client = airsim.MultirotorClient(ip=args.host, port=args.port)
    client.confirmConnection()
    vk = {"vehicle_name": args.vehicle}
    client.enableApiControl(True, **vk)
    client.armDisarm(True, **vk)

    rows: list[dict] = []
    for i, wp in enumerate(wps):
        x, y, z = float(wp["x"]), float(wp["y"]), float(wp["z"])
        yaw = float(wp.get("yaw_rad", 0.0))
        pose = airsim.Pose(
            airsim.Vector3r(x, y, -z),
            airsim.to_quaternion(0.0, 0.0, yaw),
        )
        if hasattr(client, "simPause"):
            client.simPause(True)
        client.simSetVehiclePose(pose, True, **vk)
        time.sleep(0.35)
        if hasattr(client, "simPause"):
            client.simPause(False)

        req = airsim.ImageRequest(args.camera, airsim.ImageType.Scene, False, True)
        resp = client.simGetImages([req], **vk)[0]
        raw = bytes(resp.image_data_uint8)
        if not raw:
            print(f"skip empty frame wp={i}")
            continue
        name = f"frame_{i:04d}.jpg"
        (frames / name).write_bytes(raw)

        pos = client.simGetVehiclePose(**vk).position
        rows.append(
            {
                "frame": name,
                "waypoint": wp.get("label", f"wp_{i:03d}"),
                "pos": [float(pos.x_val), float(pos.y_val), -float(pos.z_val)],
                "yaw_rad": yaw,
            }
        )
        print(f"[{i + 1}/{len(wps)}] {name} @ [{x:.1f},{y:.1f},{z:.1f}]")

    with traj_path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")

    meta = {
        "n_frames": len(rows),
        "mission_dir": str(mission_dir),
        "frames_dir": str(frames),
    }
    (out / "manifest.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(json.dumps(meta, indent=2))
    return 0 if rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
