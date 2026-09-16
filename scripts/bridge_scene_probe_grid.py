#!/usr/bin/env python3
"""Grid-probe AirSim scene for bridge-like views; saves JPEGs + manifest."""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import time
from pathlib import Path

import yaml


def _load_scene(path: Path) -> dict:
    if not path.is_file():
        return {}
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def _frange(lo: float, hi: float, step: float) -> list[float]:
    out: list[float] = []
    x = lo
    while x <= hi + 1e-6:
        out.append(round(x, 3))
        x += step
    return out


def main() -> int:
    p = argparse.ArgumentParser(description="Grid probe for bridges in AirSim scene")
    p.add_argument(
        "--scene-config",
        default=os.environ.get(
            "AIRSIM_SCENE_CONFIG",
            "configs/sim/guangzhou_scene.yaml",
        ),
    )
    p.add_argument("--host", default=os.environ.get("AIRSIM_HOST", "127.0.0.1"))
    p.add_argument("--port", type=int, default=int(os.environ.get("AIRSIM_PORT", "41451")))
    p.add_argument("--camera", default=os.environ.get("AIRSIM_CAMERA", "front_custom"))
    p.add_argument("--vehicle", default=os.environ.get("AIRSIM_VEHICLE", "drone_1"))
    p.add_argument("--out-dir", default="artifacts/bridge_probe_gz")
    p.add_argument("--max-shots", type=int, default=0, help="0 = all grid points")
    args = p.parse_args()

    scene = _load_scene(Path(args.scene_config))
    grid = scene.get("probe_grid") or {}
    xs = _frange(float(grid.get("x_min", 200)), float(grid.get("x_max", 2200)), float(grid.get("x_step", 200)))
    ys = _frange(float(grid.get("y_min", -800)), float(grid.get("y_max", 400)), float(grid.get("y_step", 200)))
    z = float(grid.get("z", scene.get("spawn_xyz", [0, 0, 40])[2]))
    yaws = [float(y) for y in grid.get("yaws_deg", [0.0, 90.0, 180.0, 270.0])]

    import airsim  # lazy

    client = airsim.MultirotorClient(ip=args.host, port=args.port)
    client.confirmConnection()
    vk = {"vehicle_name": args.vehicle}
    client.enableApiControl(True, **vk)
    client.armDisarm(True, **vk)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest: list[dict] = []
    shot = 0

    for xi, x in enumerate(xs):
        for yi, y in enumerate(ys):
            for yaw_deg in yaws:
                if args.max_shots and shot >= args.max_shots:
                    break
                yaw = math.radians(yaw_deg)
                pose = airsim.Pose(
                    airsim.Vector3r(float(x), float(y), -float(z)),
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
                if resp.width <= 0:
                    continue
                raw = bytes(resp.image_data_uint8) or base64.b64decode(resp.image_data_uint8)
                name = f"probe_{xi:02d}_{yi:02d}_yaw{int(yaw_deg):03d}.jpg"
                path = out_dir / name
                path.write_bytes(raw)
                manifest.append(
                    {
                        "file": name,
                        "xyz": [x, y, z],
                        "yaw_deg": yaw_deg,
                        "bytes": path.stat().st_size,
                    }
                )
                shot += 1
            if args.max_shots and shot >= args.max_shots:
                break
        if args.max_shots and shot >= args.max_shots:
            break

    meta = {
        "scene_config": str(args.scene_config),
        "scene_id": scene.get("scene_id"),
        "host": f"{args.host}:{args.port}",
        "shots": len(manifest),
        "points": manifest,
    }
    (out_dir / "manifest.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(json.dumps({"out_dir": str(out_dir), "shots": len(manifest)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
