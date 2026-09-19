#!/usr/bin/env python3
"""QC gate: require bridge visible in enough capture frames before COLMAP."""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

# Generic open-vocab (legacy building demos).
GENERIC_QC_CLASSES = [
    "tower",
    "suspension bridge tower cable",
    "suspension bridge",
    "cable-stayed bridge",
    "bridge",
    "viaduct",
]
GENERIC_QC_MIN_CONF = 0.08

# Humen corridor: per-class thresholds from open_vocab_probe / humen_corridor_detector.
HUMEN_QC_CLASSES = list(GENERIC_QC_CLASSES)
# Survey orbit is farther / higher than SEARCH corridor — slightly lower tower gate.
HUMEN_CLASS_MIN_CONF = [0.04, 0.06, 0.05, 0.05, 0.04, 0.04]


def _bbox_passes_spatial(bbox: np.ndarray, h: int, w: int, cls_name: str) -> bool:
    cy = 0.5 * (float(bbox[1]) + float(bbox[3]))
    if cls_name == "tower":
        return cy < 0.72 * h
    return True


def _yolo_model(classes: list[str]):
    from ultralytics import YOLO

    key = tuple(classes)
    cache = getattr(_yolo_model, "_cache", {})
    if key not in cache:
        model = YOLO("yolov8s-worldv2.pt")
        model.set_classes(list(classes))
        cache[key] = model
        _yolo_model._cache = cache  # type: ignore[attr-defined]
    return cache[key]


def _bridge_detected_generic(rgb: np.ndarray) -> tuple[bool, float, str]:
    model = _yolo_model(GENERIC_QC_CLASSES)
    res = model.predict(rgb, conf=0.02, imgsz=1280, verbose=False)[0]
    boxes = res.boxes
    if boxes is None or len(boxes) == 0:
        return False, 0.0, "generic"
    conf = float(boxes.conf.max())
    return conf >= GENERIC_QC_MIN_CONF, conf, "generic"


def _bridge_detected_humen(rgb: np.ndarray) -> tuple[bool, float, str]:
    arr = np.asarray(rgb, dtype=np.uint8)
    h, w = arr.shape[:2]
    model = _yolo_model(HUMEN_QC_CLASSES)
    res = model.predict(arr, conf=0.02, imgsz=1280, verbose=False)[0]
    boxes = res.boxes
    if boxes is None or len(boxes) == 0:
        return False, 0.0, "humen_corridor"
    best_conf = 0.0
    for i in range(len(boxes)):
        cls_id = int(boxes.cls[i])
        cls_name = HUMEN_QC_CLASSES[cls_id] if cls_id < len(HUMEN_QC_CLASSES) else str(cls_id)
        conf = float(boxes.conf[i])
        min_c = HUMEN_CLASS_MIN_CONF[cls_id] if cls_id < len(HUMEN_CLASS_MIN_CONF) else 0.04
        if conf < min_c:
            continue
        xyxy = boxes.xyxy[i].cpu().numpy().astype(np.float32)
        if not _bbox_passes_spatial(xyxy, h, w, cls_name):
            continue
        best_conf = max(best_conf, conf)
    return best_conf > 0.0, best_conf, "humen_corridor"


def _bridge_detected(rgb: np.ndarray, detector: str) -> tuple[bool, float, str]:
    if detector == "humen_corridor":
        return _bridge_detected_humen(rgb)
    return _bridge_detected_generic(rgb)


def main() -> int:
    p = argparse.ArgumentParser(description="QC capture frames for bridge visibility")
    p.add_argument("capture_dir", help="artifacts/.../capture_survey")
    p.add_argument(
        "--min-fraction",
        type=float,
        default=float(os.environ.get("AERIAL_QC_MIN_FRACTION", "0.45")),
        help="min fraction with bridge",
    )
    p.add_argument("--min-frames", type=int, default=12)
    p.add_argument(
        "--detector",
        default=os.environ.get("QC_DETECTOR", "humen_corridor"),
        choices=("humen_corridor", "generic"),
    )
    p.add_argument("--out", help="write qc_report.json here")
    args = p.parse_args()

    cap = Path(args.capture_dir).resolve()
    frames_dir = cap / "frames"
    if not frames_dir.is_dir():
        print(f"ERROR: missing {frames_dir}", file=sys.stderr)
        return 2

    try:
        import cv2
    except ImportError:
        print("ERROR: opencv-python required for qc_capture_bridge", file=sys.stderr)
        return 2

    frames = sorted(frames_dir.glob("frame_*.jpg")) + sorted(frames_dir.glob("frame_*.png"))
    if len(frames) < args.min_frames:
        print(f"ERROR: only {len(frames)} frames (need {args.min_frames})", file=sys.stderr)
        return 2

    hits: list[dict] = []
    for fp in frames:
        rgb = cv2.cvtColor(cv2.imread(str(fp)), cv2.COLOR_BGR2RGB)
        ok, conf, det_name = _bridge_detected(rgb, args.detector)
        hits.append({"frame": fp.name, "bridge": ok, "conf": round(conf, 4), "detector": det_name})

    n_ok = sum(1 for h in hits if h["bridge"])
    frac = n_ok / len(frames)
    report = {
        "status": "ok" if frac >= args.min_fraction else "fail",
        "detector": args.detector,
        "n_frames": len(frames),
        "n_bridge": n_ok,
        "fraction": round(frac, 4),
        "min_fraction": args.min_fraction,
        "frames": hits,
    }
    out_path = Path(args.out).resolve() if args.out else cap / "qc_bridge_report.json"
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({k: report[k] for k in ("status", "detector", "n_frames", "n_bridge", "fraction")}, indent=2))
    return 0 if report["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
