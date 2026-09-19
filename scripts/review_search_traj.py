#!/usr/bin/env python3
"""Human-review helper: subsample SEARCH traj poses and QC bridge visibility."""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from qc_capture_bridge import _bridge_detected  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description="Review SEARCH traj frames in AirSim")
    p.add_argument("traj_path")
    p.add_argument("--out", default="artifacts/bridge_humen_001/search_review_v2")
    p.add_argument("--stride", type=int, default=20)
    p.add_argument("--max-frames", type=int, default=40)
    args = p.parse_args()

    traj = Path(args.traj_path)
    rows = [json.loads(l) for l in traj.read_text(encoding="utf-8").splitlines() if l.strip()]
    pick = list(range(0, len(rows), max(1, args.stride)))[: args.max_frames]
    for i, r in enumerate(rows):
        if r.get("det_hit"):
            pick.extend(range(max(0, i - 1), min(len(rows), i + 2)))
    pick = sorted(set(pick))[: args.max_frames]

    import airsim
    import cv2

    client = airsim.MultirotorClient(
        ip=os.environ.get("AIRSIM_HOST", "127.0.0.1"),
        port=int(os.environ.get("AIRSIM_PORT", "41463")),
    )
    client.confirmConnection()
    vk = {"vehicle_name": os.environ.get("AIRSIM_VEHICLE", "SimpleFlight")}
    cam = os.environ.get("AIRSIM_CAMERA", "0")
    out = Path(args.out) / "frames"
    out.mkdir(parents=True, exist_ok=True)

    hits: list[dict] = []
    prev = None
    for i in pick:
        r = rows[i]
        pos = r["pos"]
        yaw = float(r.get("yaw", 0.785398))
        if prev is not None:
            dx, dy = pos[0] - prev[0], pos[1] - prev[1]
            if abs(dx) + abs(dy) > 0.5:
                yaw = math.atan2(dy, dx)
        prev = pos
        pitch = math.radians(float(os.environ.get("BRIDGE_CAMERA_PITCH", "-20")))
        pose = airsim.Pose(
            airsim.Vector3r(float(pos[0]), float(pos[1]), -float(pos[2])),
            airsim.to_quaternion(pitch, 0.0, yaw),
        )
        client.simSetVehiclePose(pose, True, **vk)
        time.sleep(0.15)
        resp = client.simGetImages(
            [airsim.ImageRequest(cam, airsim.ImageType.Scene, False, False)], **vk
        )[0]
        rgb = np.frombuffer(resp.image_data_uint8, dtype=np.uint8).reshape(resp.height, resp.width, 3)
        ok, conf = _bridge_detected(rgb)
        name = f"step_{i:04d}_det{int(bool(r.get('det_hit')))}.jpg"
        cv2.imwrite(str(out / name), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        hits.append(
            {
                "step": i,
                "file": name,
                "traj_det_hit": bool(r.get("det_hit")),
                "qc_bridge": ok,
                "qc_conf": round(conf, 4),
                "pos": pos,
            }
        )

    n_qc = sum(1 for h in hits if h["qc_bridge"])
    report = {
        "n_frames": len(hits),
        "n_qc_bridge": n_qc,
        "fraction": round(n_qc / max(len(hits), 1), 4),
        "frames": hits,
    }
    Path(args.out, "review_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
