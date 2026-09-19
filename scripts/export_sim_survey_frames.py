#!/usr/bin/env python3
"""Export JPEG frames at survey waypoints for offline COLMAP (AirSim on 84)."""
from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from airsim_pose_utils import (
    quat_wxyz_from_yaw_pitch,
    read_camera_pose_zup,
    read_pose_xyz,
    resolve_deck_look_at,
    set_pose_verified,
    survey_capture_yaw_pitch,
)


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
    spec_path = mission_dir / "mission_spec.json"
    if not spec_path.is_file():
        raise SystemExit(f"missing {spec_path}")
    spec = json.loads(spec_path.read_text(encoding="utf-8"))
    centroid = spec.get("bridge_centroid_xyz")
    if not centroid or len(centroid) < 3:
        raise SystemExit("bridge_centroid_xyz required — orbit center must be the building, not the drone")
    look_at = resolve_deck_look_at(spec)
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
        yaw, pitch = survey_capture_yaw_pitch((x, y, z), look_at)
        try:
            ax, ay, az, yaw, pitch, quat = set_pose_verified(
                client, vk, x, y, z, yaw, pitch=pitch,
            )
        except RuntimeError as exc:
            print(f"WARN wp={i}: {exc} — capturing at last pose", flush=True)
            ax, ay, az = read_pose_xyz(client, vk)
            qw, qx, qy, qz = quat_wxyz_from_yaw_pitch(yaw, pitch)
            quat = (qw, qx, qy, qz)

        req = airsim.ImageRequest(args.camera, airsim.ImageType.Scene, False, True)
        resp = client.simGetImages([req], **vk)[0]
        raw = bytes(resp.image_data_uint8)
        if not raw:
            print(f"skip empty frame wp={i}")
            continue
        name = f"frame_{i:04d}.jpg"
        (frames / name).write_bytes(raw)

        row = {
            "frame": name,
            "waypoint": wp.get("label", f"wp_{i:03d}"),
            "pos": [ax, ay, az],
            "yaw_rad": yaw,
            "pitch_rad": pitch,
            "quat_wxyz": list(quat),
            "look_at": look_at,
        }
        try:
            cam_pos, cam_quat = read_camera_pose_zup(client, args.camera, vk)
            row["cam_pos"] = cam_pos
            row["cam_quat_wxyz"] = cam_quat
        except Exception as exc:  # noqa: BLE001
            print(f"warn: simGetCameraInfo failed wp={i}: {exc}", flush=True)
        rows.append(row)
        pitch_deg = __import__("math").degrees(pitch)
        print(f"[{i + 1}/{len(wps)}] {name} @ [{ax:.1f},{ay:.1f},{az:.1f}] pitch={pitch_deg:.1f}°", flush=True)

    with traj_path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")

    meta = {
        "n_frames": len(rows),
        "mission_dir": str(mission_dir),
        "frames_dir": str(frames),
        "look_at": look_at,
    }
    (out / "manifest.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(json.dumps(meta, indent=2))
    return 0 if rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
