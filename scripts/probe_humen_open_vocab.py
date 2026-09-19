#!/usr/bin/env python3
"""Sweep prompts/classes at corridor poses — inform Humen SEARCH detector choice."""
from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path

import numpy as np


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--host", default=os.environ.get("AIRSIM_HOST", "127.0.0.1"))
    p.add_argument("--port", type=int, default=int(os.environ.get("AIRSIM_PORT", "41463")))
    p.add_argument("--camera", default=os.environ.get("AIRSIM_CAMERA", "0"))
    p.add_argument("--vehicle", default=os.environ.get("AIRSIM_VEHICLE", "SimpleFlight"))
    p.add_argument("--out", default="artifacts/bridge_humen_001/open_vocab_probe.json")
    args = p.parse_args()

    import airsim
    from ultralytics import YOLO

    client = airsim.MultirotorClient(ip=args.host, port=args.port)
    client.confirmConnection()
    vk = {"vehicle_name": args.vehicle}
    model_path = os.environ.get("SIM_YOLO_MODEL", "yolov8s-worldv2.pt")
    if not Path(model_path).is_file():
        model_path = str(Path.home() / "Projects/aerial-wam-v2/yolov8s-worldv2.pt")
    model = YOLO(model_path)

    prompts = [
        "bridge",
        "suspension bridge",
        "cable-stayed bridge",
        "viaduct",
        "overpass",
        "tower",
        "ship",
        "cargo ship",
        "suspension bridge tower cable",
    ]
    poses = [
        (1100, -50, 95, -22, 25),
        (1200, -50, 95, -20, 45),
        (1350, -50, 95, -22, 25),
        (1500, -35, 95, -18, 30),
        (1250, -50, 85, -15, 40),
    ]
    confs = [0.01, 0.03, 0.05, 0.08]

    results: list[dict] = []
    for prompt in prompts:
        model.set_classes(prompt.split())
        for x, y, z, pitch, yaw in poses:
            pose = airsim.Pose(
                airsim.Vector3r(float(x), float(y), -float(z)),
                airsim.to_quaternion(math.radians(pitch), 0.0, math.radians(yaw)),
            )
            client.simSetVehiclePose(pose, True, **vk)
            time.sleep(0.35)
            resp = client.simGetImages(
                [airsim.ImageRequest(args.camera, airsim.ImageType.Scene, False, False)], **vk
            )[0]
            rgb = np.frombuffer(resp.image_data_uint8, dtype=np.uint8).reshape(resp.height, resp.width, 3)
            best = 0.0
            n_best = 0
            cls_best = ""
            for conf in confs:
                res = model.predict(rgb, conf=conf, imgsz=1280, verbose=False)[0]
                boxes = res.boxes
                if boxes is None or len(boxes) == 0:
                    continue
                idx = int(boxes.conf.argmax())
                c = float(boxes.conf[idx])
                if c > best:
                    best = c
                    n_best = len(boxes)
                    cls_best = prompt.split()[min(int(boxes.cls[idx]), len(prompt.split()) - 1)]
            row = {
                "prompt": prompt,
                "x": x,
                "y": y,
                "z": z,
                "pitch": pitch,
                "yaw": yaw,
                "best_conf": round(best, 4),
                "n_boxes": n_best,
            }
            results.append(row)
            if best >= 0.05:
                print(f"HIT prompt={prompt[:24]:24s} pos=({x},{y}) conf={best:.3f} n={n_best}", flush=True)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    summary = {
        "n_trials": len(results),
        "n_hits_005": sum(1 for r in results if r["best_conf"] >= 0.05),
        "n_hits_010": sum(1 for r in results if r["best_conf"] >= 0.10),
        "best": max(results, key=lambda r: r["best_conf"]),
        "results": results,
    }
    out.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps({k: summary[k] for k in ("n_trials", "n_hits_005", "n_hits_010", "best")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
