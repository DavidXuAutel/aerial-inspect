#!/usr/bin/env python3
"""Export a smooth survey fly-through MP4 from planned waypoints (AirSim on 84)."""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from pathlib import Path

import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(ROOT))
from airsim_pose_utils import set_pose_verified, yaw_toward_xy
from path_clearance import repair_pose_clearance
from aerial_inspect.survey.path_spline import closed_cubic_bezier_segments, eval_cubic_bezier


def _encode_video(video_frames: Path, out_mp4: Path, fps: int) -> None:
    frames = sorted(video_frames.glob("vid_*.jpg"))
    if not frames:
        raise SystemExit(f"no frames under {video_frames}")

    import shutil

    if shutil.which("ffmpeg"):
        proc = subprocess.run(
            [
                "ffmpeg", "-y",
                "-framerate", str(fps),
                "-i", str(video_frames / "vid_%05d.jpg"),
                "-c:v", "libx264", "-pix_fmt", "yuv420p",
                str(out_mp4),
            ],
            capture_output=True,
            text=True,
        )
        if proc.returncode == 0:
            return
        print(proc.stderr, flush=True)

    import cv2

    first = cv2.imread(str(frames[0]))
    if first is None:
        raise SystemExit(f"failed to read {frames[0]}")
    h, w = first.shape[:2]
    writer = cv2.VideoWriter(
        str(out_mp4),
        cv2.VideoWriter_fourcc(*"mp4v"),
        float(fps),
        (w, h),
    )
    if not writer.isOpened():
        raise SystemExit(f"VideoWriter failed for {out_mp4}")
    for frame_path in frames:
        img = cv2.imread(str(frame_path))
        if img is None:
            continue
        if img.shape[0] != h or img.shape[1] != w:
            img = cv2.resize(img, (w, h))
        writer.write(img)
    writer.release()
    print(f"encoded with OpenCV -> {out_mp4}", flush=True)


def _interp_on_orbit(
    x: float,
    y: float,
    z: float,
    x2: float,
    y2: float,
    z2: float,
    t: float,
    center_xy: tuple[float, float],
) -> tuple[float, float, float, float]:
    """Move along the circular arc (planet path), always face the building center (sun)."""
    cx, cy = center_xy
    r1 = math.hypot(x - cx, y - cy)
    r2 = math.hypot(x2 - cx, y2 - cy)
    r = r1 + (r2 - r1) * t
    a1 = math.atan2(y - cy, x - cx)
    a2 = math.atan2(y2 - cy, x2 - cx)
    da = a2 - a1
    while da > math.pi:
        da -= 2.0 * math.pi
    while da < -math.pi:
        da += 2.0 * math.pi
    a = a1 + da * t
    ix = cx + r * math.cos(a)
    iy = cy + r * math.sin(a)
    iz = z + (z2 - z) * t
    return ix, iy, iz, yaw_toward_xy((ix, iy), center_xy)


def _interp_linear_xy(
    x: float,
    y: float,
    z: float,
    x2: float,
    y2: float,
    z2: float,
    t: float,
    center_xy: tuple[float, float],
) -> tuple[float, float, float, float]:
    """Straight-line path between waypoints; camera still faces building center."""
    ix = x + (x2 - x) * t
    iy = y + (y2 - y) * t
    iz = z + (z2 - z) * t
    return ix, iy, iz, yaw_toward_xy((ix, iy), center_xy)


def _clearance_m(mission_dir: Path) -> float:
    p = mission_dir / "orbit_clearance.json"
    if p.is_file():
        return float(json.loads(p.read_text(encoding="utf-8")).get("clearance_m", 12.0))
    spec = mission_dir / "mission_spec.json"
    if spec.is_file():
        return float(json.loads(spec.read_text(encoding="utf-8")).get("survey", {}).get("clearance_m", 12.0))
    return 12.0


def _path_closed(mission_dir: Path) -> bool:
    clearance = mission_dir / "orbit_clearance.json"
    if clearance.is_file():
        data = json.loads(clearance.read_text(encoding="utf-8"))
        if "path_closed" in data:
            return bool(data["path_closed"])
        return float(data.get("orbit_arc_deg", 360.0)) >= 360.0
    spec_path = mission_dir / "mission_spec.json"
    if spec_path.is_file():
        sv = json.loads(spec_path.read_text(encoding="utf-8")).get("survey", {})
        return float(sv.get("orbit_arc_deg", 360.0)) >= 360.0
    return True


def _path_settings(mission_dir: Path) -> tuple[str, str]:
    """Return (path_mode, curve) where path_mode is orbit|freeform and curve is bezier|linear."""
    curve = "linear"
    clearance = mission_dir / "orbit_clearance.json"
    if clearance.is_file():
        data = json.loads(clearance.read_text(encoding="utf-8"))
        curve = str(data.get("path_curve", curve))
        if data.get("path_mode") == "freeform":
            return "freeform", curve
    spec_path = mission_dir / "mission_spec.json"
    if spec_path.is_file():
        survey = json.loads(spec_path.read_text(encoding="utf-8")).get("survey", {})
        curve = str(survey.get("path_curve", curve))
        if survey.get("pattern") == "clearance_orbit":
            return "freeform", curve
    return "orbit", curve


def _build_poses(
    wps: list[dict],
    center_xy: tuple[float, float],
    path_mode: str,
    curve: str,
    steps_between: int,
    hold_frames: int,
    path_closed: bool = True,
) -> list[tuple[float, float, float, float]]:
    n_wps = len(wps)
    poses: list[tuple[float, float, float, float]] = []
    n_segments = n_wps if path_closed else max(n_wps - 1, 0)

    if path_mode == "freeform" and curve == "bezier" and n_wps >= 3 and path_closed:
        pts = [(float(w["x"]), float(w["y"]), float(w["z"])) for w in wps]
        segments = closed_cubic_bezier_segments(pts)
        for i, (p0, c1, c2, p3) in enumerate(segments):
            x, y, z = p0
            yaw = yaw_toward_xy((x, y), center_xy)
            for _ in range(max(1, hold_frames)):
                poses.append((x, y, z, yaw))
            for step in range(1, steps_between + 1):
                t = step / steps_between
                ix, iy, iz = eval_cubic_bezier(p0, c1, c2, p3, t)
                poses.append((ix, iy, iz, yaw_toward_xy((ix, iy), center_xy)))
        return poses

    interp = _interp_linear_xy if path_mode == "freeform" else _interp_on_orbit
    for i in range(n_wps):
        wp = wps[i]
        x, y, z = float(wp["x"]), float(wp["y"]), float(wp["z"])
        yaw = yaw_toward_xy((x, y), center_xy)
        for _ in range(max(1, hold_frames)):
            poses.append((x, y, z, yaw))
        if i >= n_segments:
            continue
        nxt = wps[(i + 1) % n_wps] if path_closed else wps[i + 1]
        x2, y2, z2 = float(nxt["x"]), float(nxt["y"]), float(nxt["z"])
        for step in range(1, steps_between + 1):
            t = step / steps_between
            poses.append(interp(x, y, z, x2, y2, z2, t, center_xy))
    return poses


def _orbit_center(mission_dir: Path, wps: list[dict]) -> tuple[float, float, float]:
    """Building centroid — never use waypoint mean (that is the drone path, not the target)."""
    meta_path = mission_dir / "mission_spec.json"
    if meta_path.is_file():
        spec = json.loads(meta_path.read_text(encoding="utf-8"))
        c = spec.get("bridge_centroid_xyz")
        if c and len(c) >= 3:
            return float(c[0]), float(c[1]), float(c[2])
        c = spec.get("search_area", {}).get("center_xy")
        if c and len(c) >= 2:
            z = float(spec.get("search_area", {}).get("altitude_m", wps[0]["z"]))
            return float(c[0]), float(c[1]), z
    raise SystemExit(f"missing bridge_centroid_xyz in {meta_path}")


def main() -> int:
    p = argparse.ArgumentParser(description="Export survey path video from AirSim")
    p.add_argument("mission_dir", help="artifacts/<mission_id>")
    p.add_argument("--out", help="output mp4 (default <mission_dir>/capture_survey/survey_flythrough.mp4)")
    p.add_argument("--host", default=os.environ.get("AIRSIM_HOST", "127.0.0.1"))
    p.add_argument("--port", type=int, default=int(os.environ.get("AIRSIM_PORT", "41451")))
    p.add_argument("--camera", default=os.environ.get("AIRSIM_CAMERA", "front_custom"))
    p.add_argument("--vehicle", default=os.environ.get("AIRSIM_VEHICLE", "drone_1"))
    p.add_argument("--steps-between", type=int, default=12, help="interpolated frames between waypoints")
    p.add_argument("--hold-frames", type=int, default=8, help="frames to hold at each waypoint")
    p.add_argument("--fps", type=int, default=10)
    p.add_argument("--encode-only", action="store_true", help="skip capture, encode existing video_frames/")
    args = p.parse_args()

    mission_dir = Path(args.mission_dir).resolve()
    wps = json.loads((mission_dir / "waypoints.json").read_text(encoding="utf-8"))
    capture = mission_dir / "capture_survey"
    video_frames = capture / "video_frames"
    video_frames.mkdir(parents=True, exist_ok=True)
    out_mp4 = Path(args.out).resolve() if args.out else capture / "survey_flythrough.mp4"

    import airsim  # lazy

    client = airsim.MultirotorClient(ip=args.host, port=args.port)
    client.confirmConnection()
    vk = {"vehicle_name": args.vehicle}
    client.enableApiControl(True, **vk)
    client.armDisarm(True, **vk)

    cx, cy, _cz = _orbit_center(mission_dir, wps)
    center_xy = (cx, cy)
    path_mode, curve = _path_settings(mission_dir)
    path_closed = _path_closed(mission_dir)
    print(
        f"orbit target (building center): ({cx:.1f}, {cy:.1f}), "
        f"path={path_mode}, curve={curve}, closed={path_closed}",
        flush=True,
    )
    poses = _build_poses(
        wps,
        center_xy,
        path_mode,
        curve,
        args.steps_between,
        args.hold_frames,
        path_closed,
    )

    idx = len(list(video_frames.glob("vid_*.jpg")))
    if args.encode_only:
        if idx == 0:
            print("no frames to encode")
            return 1
        _encode_video(video_frames, out_mp4, args.fps)
        meta = {
            "n_frames": idx,
            "fps": args.fps,
            "duration_s": round(idx / args.fps, 2),
            "video": str(out_mp4),
            "frames_dir": str(video_frames),
            "encode_only": True,
        }
        (capture / "video_manifest.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
        print(json.dumps(meta, indent=2))
        return 0

    clearance_m = _clearance_m(mission_dir)
    n_repair_fail = 0
    idx = 0
    for pose_i, (x, y, z, yaw) in enumerate(poses):
        if path_mode == "freeform":
            x, y, z, yaw, meta, ok = repair_pose_clearance(
                client, vk, x, y, z, yaw, clearance_m, center_xy,
            )
            if not ok:
                n_repair_fail += 1
                print(
                    f"WARN pose {pose_i}: unrepaired clear={meta.get('min_clear_m')}m",
                    flush=True,
                )
        try:
            set_pose_verified(client, vk, x, y, z, yaw, settle_s=0.25)
        except RuntimeError as exc:
            print(f"FAIL video pose: {exc}", flush=True)
            return 1

        req = airsim.ImageRequest(args.camera, airsim.ImageType.Scene, False, True)
        resp = client.simGetImages([req], **vk)[0]
        raw = bytes(resp.image_data_uint8)
        if not raw:
            continue
        name = f"vid_{idx:05d}.jpg"
        (video_frames / name).write_bytes(raw)
        idx += 1
        if idx % 20 == 0:
            print(f"captured {idx}/{len(poses)}", flush=True)

    if idx == 0 and not list(video_frames.glob("vid_*.jpg")):
        print("no frames captured")
        return 1

    _encode_video(video_frames, out_mp4, args.fps)

    meta = {
        "n_frames": idx,
        "fps": args.fps,
        "duration_s": round(idx / args.fps, 2),
        "video": str(out_mp4),
        "frames_dir": str(video_frames),
        "path_mode": path_mode,
        "path_curve": curve,
        "n_repair_fail": n_repair_fail,
    }
    (capture / "video_manifest.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
