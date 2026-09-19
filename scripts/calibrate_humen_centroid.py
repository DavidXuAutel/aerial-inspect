#!/usr/bin/env python3
"""Locate Humen bridge deck by scanning E-W positions (sim GT when vision SEARCH fails)."""
from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path

import numpy as np


def _yolo_bridge_score(rgb: np.ndarray) -> tuple[float, float]:
    """Return (confidence, bbox_area) from YOLO-World if available."""
    try:
        from ultralytics import YOLO
    except ImportError:
        return 0.0, 0.0
    model = getattr(_yolo_bridge_score, "_model", None)
    if model is None:
        model = YOLO("yolov8s-worldv2.pt")
        model.set_classes(["bridge", "suspension bridge", "viaduct"])
        _yolo_bridge_score._model = model  # type: ignore[attr-defined]
    res = model.predict(rgb, conf=0.04, imgsz=1280, verbose=False)[0]
    boxes = res.boxes
    if boxes is None or len(boxes) == 0:
        return 0.0, 0.0
    idx = int(boxes.conf.argmax())
    xyxy = boxes.xyxy[idx].tolist()
    area = (xyxy[2] - xyxy[0]) * (xyxy[3] - xyxy[1])
    return float(boxes.conf[idx]), float(area)


def _score_frame(rgb: np.ndarray) -> float:
    """Prefer YOLO bridge hits; else horizontal structure in lower-mid frame (deck/cables)."""
    conf, area = _yolo_bridge_score(rgb)
    if conf >= 0.08 and area > 400.0:
        return conf * 5000.0 + area
    h, w = rgb.shape[:2]
    # Bridge deck / cables: strong horizontal edges in center-lower band.
    band = rgb[int(h * 0.35) : int(h * 0.72), int(w * 0.2) : int(w * 0.8)]
    gray = band.mean(axis=2)
    gx = np.abs(np.diff(gray, axis=1)).mean()
    gy = np.abs(np.diff(gray, axis=0)).mean()
    horiz = float(gx - 0.35 * gy)
    # Penalize uniform forest (green upper half, low contrast).
    upper = rgb[: int(h * 0.55), :, :]
    g_dom = float(upper[:, :, 1].mean() - 0.5 * (upper[:, :, 0].mean() + upper[:, :, 2].mean()))
    forest_pen = max(0.0, g_dom - 12.0) * 3.0
    sky_water = float(gray.std())
    return horiz * 4.0 + sky_water * 1.5 - forest_pen


def main() -> int:
    p = argparse.ArgumentParser(description="Scan X axis to locate Humen bridge for orbit centroid")
    p.add_argument("mission_dir", help="artifacts/bridge_humen_001")
    p.add_argument("--host", default=os.environ.get("AIRSIM_HOST", "127.0.0.1"))
    p.add_argument("--port", type=int, default=int(os.environ.get("AIRSIM_PORT", "41463")))
    p.add_argument("--camera", default=os.environ.get("AIRSIM_CAMERA", "0"))
    p.add_argument("--vehicle", default=os.environ.get("AIRSIM_VEHICLE", "SimpleFlight"))
    p.add_argument("--y", type=float, default=-60.0)
    p.add_argument("--z", type=float, default=95.0)
    p.add_argument(
        "--x-min",
        type=float,
        default=600.0,
        help="East corridor only — west of x~500 is forest, not the bridge",
    )
    p.add_argument("--x-max", type=float, default=2200.0)
    p.add_argument("--x-step", type=float, default=150.0)
    p.add_argument("--yaw-deg", type=float, default=25.0)
    p.add_argument("--pitch-deg", type=float, default=-22.0)
    p.add_argument("--deck-z", type=float, default=55.0)
    args = p.parse_args()

    import airsim

    client = airsim.MultirotorClient(ip=args.host, port=args.port)
    client.confirmConnection()
    vk = {"vehicle_name": args.vehicle}

    xs = np.arange(args.x_min, args.x_max + 1e-6, args.x_step)
    results: list[dict] = []
    best_score = -1e9
    best_x = float(xs[0])

    for x in xs:
        pose = airsim.Pose(
            airsim.Vector3r(float(x), float(args.y), -float(args.z)),
            airsim.to_quaternion(math.radians(args.pitch_deg), 0.0, math.radians(args.yaw_deg)),
        )
        client.simSetVehiclePose(pose, True, **vk)
        time.sleep(0.35)
        resp = client.simGetImages(
            [airsim.ImageRequest(args.camera, airsim.ImageType.Scene, False, False)], **vk
        )[0]
        rgb = np.frombuffer(resp.image_data_uint8, dtype=np.uint8).reshape(resp.height, resp.width, 3)
        score = _score_frame(rgb)
        results.append({"x": float(x), "score": round(score, 3)})
        if score > best_score:
            best_score = score
            best_x = float(x)
        print(f"x={x:7.0f} score={score:.2f}", flush=True)

    # refine ±step
    fine_x = best_x
    for x in np.linspace(best_x - args.x_step, best_x + args.x_step, 7):
        pose = airsim.Pose(
            airsim.Vector3r(float(x), float(args.y), -float(args.z)),
            airsim.to_quaternion(math.radians(args.pitch_deg), 0.0, math.radians(args.yaw_deg)),
        )
        client.simSetVehiclePose(pose, True, **vk)
        time.sleep(0.3)
        resp = client.simGetImages(
            [airsim.ImageRequest(args.camera, airsim.ImageType.Scene, False, False)], **vk
        )[0]
        rgb = np.frombuffer(resp.image_data_uint8, dtype=np.uint8).reshape(resp.height, resp.width, 3)
        score = _score_frame(rgb)
        if score > best_score:
            best_score = score
            fine_x = float(x)

    # stand off ~50m west of best viewing point along corridor
    centroid = [round(fine_x + 50.0, 1), round(args.y, 1), round(args.deck_z, 1)]
    detection = {
        "status": "ok",
        "centroid_xyz": centroid,
        "source": "humen_geometric_scan",
        "best_score": round(best_score, 3),
        "scan": results,
        "refined_x": fine_x,
    }

    mission_dir = Path(args.mission_dir).resolve()
    mission_dir.mkdir(parents=True, exist_ok=True)
    out = mission_dir / "detected_centroid.json"
    out.write_text(json.dumps(detection, indent=2), encoding="utf-8")
    print(json.dumps(detection, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
