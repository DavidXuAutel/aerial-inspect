#!/usr/bin/env python3
"""Export COLMAP frames from WAM sim-survey runs.

Default: capture at **flown** endpoint (requires validate_survey_poses pass).
Set SURVEY_EXPORT_ALLOW_PLANNED=1 only for debug (not mainline).
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from airsim_pose_utils import (
    facade_look_at_for_pos,
    read_camera_pose_zup,
    resolve_deck_look_at,
    set_pose_verified,
    survey_capture_yaw_pitch,
)


def _last_traj_row(traj_path: Path) -> dict | None:
    if not traj_path.is_file():
        return None
    last = None
    for line in traj_path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            last = json.loads(line)
    return last


def _load_planned_waypoints(mission_dir: Path) -> list[dict]:
    wp_path = mission_dir / "waypoints.json"
    if not wp_path.is_file():
        return []
    return json.loads(wp_path.read_text(encoding="utf-8"))


def _resolve_survey_traj(wp_dir: Path) -> Path | None:
    for candidate in (
        wp_dir / "traj" / "route00.jsonl",
        wp_dir / "traj.jsonl",
        wp_dir / "traj" / "route00" / "traj.jsonl",
    ):
        if candidate.is_file():
            return candidate
    matches = sorted(wp_dir.glob("**/route*.jsonl"))
    return matches[0] if matches else None


def main() -> int:
    p = argparse.ArgumentParser(description="Export capture_survey from sim_runs/survey")
    p.add_argument("mission_dir", help="artifacts/<mission_id>")
    p.add_argument("--out", help="output capture dir (default <mission>/capture_survey)")
    p.add_argument("--host", default=os.environ.get("AIRSIM_HOST", "127.0.0.1"))
    p.add_argument("--port", type=int, default=int(os.environ.get("AIRSIM_PORT", "41451")))
    p.add_argument("--camera", default=os.environ.get("AIRSIM_CAMERA", "front_custom"))
    p.add_argument("--vehicle", default=os.environ.get("AIRSIM_VEHICLE", "drone_1"))
    args = p.parse_args()

    mission_dir = Path(args.mission_dir).resolve()
    spec = json.loads((mission_dir / "mission_spec.json").read_text(encoding="utf-8"))
    centroid = spec.get("bridge_centroid_xyz")
    if not centroid or len(centroid) < 3:
        raise SystemExit("bridge_centroid_xyz missing — run sim-pipeline + replan-survey first")
    look_at_default = resolve_deck_look_at(spec)

    detected = mission_dir / "detected_centroid.json"
    if detected.is_file():
        det = json.loads(detected.read_text(encoding="utf-8"))
        if det.get("source") == "manual":
            print("WARN: detected_centroid.json is manual — prefer sim-search detection", flush=True)

    survey_root = mission_dir / "sim_runs" / "survey"
    if not survey_root.is_dir():
        raise SystemExit(f"missing {survey_root} — run: aerial-inspect sim-survey")

    wp_dirs = sorted([d for d in survey_root.iterdir() if d.is_dir() and d.name.startswith("wp_")])
    if not wp_dirs:
        raise SystemExit(f"no wp_* dirs under {survey_root}")

    out = Path(args.out).resolve() if args.out else mission_dir / "capture_survey"
    frames = out / "frames"
    # Fresh frames only — avoid mixing with prior exports.
    if frames.is_dir():
        for old in frames.glob("*.jpg"):
            old.unlink()
    frames.mkdir(parents=True, exist_ok=True)
    traj_path = out / "traj.jsonl"

    import airsim

    client = airsim.MultirotorClient(ip=args.host, port=args.port)
    client.confirmConnection()
    vk = {"vehicle_name": args.vehicle}
    client.enableApiControl(True, **vk)
    client.armDisarm(True, **vk)

    planned = _load_planned_waypoints(mission_dir)
    rows: list[dict] = []
    for i, wp_dir in enumerate(wp_dirs):
        traj_file = _resolve_survey_traj(wp_dir)
        if traj_file is None:
            print(f"skip {wp_dir.name}: no traj", flush=True)
            continue
        last = _last_traj_row(traj_file)
        if last is None or "pos" not in last:
            print(f"skip {wp_dir.name}: empty traj", flush=True)
            continue
        fx, fy, fz = [float(v) for v in last["pos"][:3]]
        pose_source = "flown_traj"
        x, y, z = fx, fy, fz
        if i < len(planned):
            px, py, pz = float(planned[i]["x"]), float(planned[i]["y"]), float(planned[i]["z"])
            err_xy = math.hypot(fx - px, fy - py)
            err_z = abs(fz - pz)
            max_xy = float(os.environ.get("SURVEY_EXPORT_MAX_XY_ERROR_M", "15"))
            max_z = float(os.environ.get("SURVEY_EXPORT_MAX_Z_ERROR_M", "12"))
            if os.environ.get("SURVEY_EXPORT_ALLOW_PLANNED", "0") == "1":
                x, y, z = px, py, pz
                pose_source = "waypoints.json"
            elif os.environ.get("SURVEY_EXPORT_REQUIRE_POSE", "1") == "1" and (
                err_xy > max_xy or err_z > max_z
            ):
                print(
                    f"ERROR {wp_dir.name}: flown vs planned err_xy={err_xy:.1f} err_z={err_z:.1f} — skip",
                    file=sys.stderr,
                    flush=True,
                )
                continue
        # Per-station aim on span centerline (fixes empty mid-span water frames).
        look_at = facade_look_at_for_pos((x, y, z), spec)
        yaw, pitch = survey_capture_yaw_pitch((x, y, z), look_at)
        # Prefer planned facade yaw when available (already faces centerline).
        if i < len(planned) and planned[i].get("yaw_rad") is not None:
            if os.environ.get("SURVEY_USE_WP_YAW", "1") not in ("0", "false", "False"):
                yaw = float(planned[i]["yaw_rad"])
        try:
            ax, ay, az, yaw, pitch, quat = set_pose_verified(
                client, vk, x, y, z, yaw, pitch=pitch,
            )
        except RuntimeError as exc:
            print(f"WARN {wp_dir.name}: {exc} — capturing at last pose", flush=True)
            from airsim_pose_utils import quat_wxyz_from_yaw_pitch, read_pose_xyz

            ax, ay, az = read_pose_xyz(client, vk)
            qw, qx, qy, qz = quat_wxyz_from_yaw_pitch(yaw, pitch)
            quat = (qw, qx, qy, qz)

        req = airsim.ImageRequest(args.camera, airsim.ImageType.Scene, False, True)
        resp = client.simGetImages([req], **vk)[0]
        raw = bytes(resp.image_data_uint8)
        if not raw:
            print(f"skip empty frame {wp_dir.name}")
            continue
        name = f"frame_{i:04d}.jpg"
        (frames / name).write_bytes(raw)

        row = {
            "frame": name,
            "waypoint": planned[i].get("label", wp_dir.name) if i < len(planned) else wp_dir.name,
            "pos": [ax, ay, az],
            "yaw_rad": yaw,
            "pitch_rad": pitch,
            "quat_wxyz": list(quat),
            "look_at": look_at,
            "pose_source": pose_source,
            "source_traj": str(traj_file),
        }
        try:
            cam_pos, cam_quat = read_camera_pose_zup(client, args.camera, vk)
            row["cam_pos"] = cam_pos
            row["cam_quat_wxyz"] = cam_quat
        except Exception:  # noqa: BLE001
            pass
        rows.append(row)
        print(f"[{i + 1}/{len(wp_dirs)}] {name} @ [{ax:.1f},{ay:.1f},{az:.1f}] look=[{look_at[0]:.1f},{look_at[1]:.1f},{look_at[2]:.1f}]", flush=True)

    if not rows:
        raise SystemExit("no frames exported from sim survey")

    with traj_path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")

    meta = {
        "n_frames": len(rows),
        "mission_dir": str(mission_dir),
        "frames_dir": str(frames),
        "look_at_default": look_at_default,
        "aim": "facade_span_centerline",
        "source": "sim_survey_flown_traj",
    }
    (out / "manifest.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
