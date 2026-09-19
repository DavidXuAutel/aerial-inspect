#!/usr/bin/env python3
"""Encode survey_flythrough.mp4 from existing capture_survey frames (no AirSim)."""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from pathlib import Path


def _encode(frames: list[Path], out_mp4: Path, fps: int) -> None:
    if not frames:
        raise SystemExit("no frames to encode")
    tmp = out_mp4.parent / "_video_stitch"
    tmp.mkdir(parents=True, exist_ok=True)
    for i, src in enumerate(frames):
        dst = tmp / f"vid_{i:05d}.jpg"
        if dst.exists():
            dst.unlink()
        try:
            dst.symlink_to(src.resolve())
        except OSError:
            shutil.copy2(src, dst)
    if shutil.which("ffmpeg"):
        proc = subprocess.run(
            [
                "ffmpeg",
                "-y",
                "-framerate",
                str(fps),
                "-i",
                str(tmp / "vid_%05d.jpg"),
                "-c:v",
                "libx264",
                "-pix_fmt",
                "yuv420p",
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
    for fp in frames:
        img = cv2.imread(str(fp))
        if img is None:
            continue
        if img.shape[:2] != (h, w):
            img = cv2.resize(img, (w, h))
        writer.write(img)
    writer.release()


def main() -> int:
    p = argparse.ArgumentParser(description="Stitch capture frames into MP4")
    p.add_argument("mission_dir", help="artifacts/<mission_id>")
    p.add_argument("--capture", help="capture dir (default <mission>/capture_survey)")
    p.add_argument("--fps", type=int, default=6, help="playback fps (48 frames @6fps ≈ 8s)")
    p.add_argument("--out", help="output mp4 path")
    args = p.parse_args()

    mission_dir = Path(args.mission_dir).resolve()
    capture = Path(args.capture).resolve() if args.capture else mission_dir / "capture_survey"
    frames_dir = capture / "frames"
    traj_path = capture / "traj.jsonl"
    if not frames_dir.is_dir():
        raise SystemExit(f"missing {frames_dir}")
    if not traj_path.is_file():
        raise SystemExit(f"missing {traj_path}")

    order: list[str] = []
    for line in traj_path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            order.append(json.loads(line)["frame"])

    paths = [frames_dir / name for name in order]
    missing = [p for p in paths if not p.is_file()]
    if missing:
        raise SystemExit(f"missing {len(missing)} frames, e.g. {missing[0]}")

    out_mp4 = Path(args.out).resolve() if args.out else capture / "survey_flythrough.mp4"
    _encode(paths, out_mp4, args.fps)

    meta = {
        "n_frames": len(paths),
        "fps": args.fps,
        "duration_s": round(len(paths) / args.fps, 2),
        "video": str(out_mp4),
        "source": "capture_frames",
        "traj": str(traj_path),
    }
    (capture / "video_manifest.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
