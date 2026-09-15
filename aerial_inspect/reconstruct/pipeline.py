"""Offline reconstruction pipeline (COLMAP wrapper)."""
from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path
from typing import Any, Dict, List, Optional


def _which(cmd: str) -> Optional[str]:
    return shutil.which(cmd)


def prepare_colmap_workspace(capture_dir: Path, workspace: Path) -> Path:
    """Symlink/copy frames into COLMAP-friendly layout."""
    images = workspace / "images"
    images.mkdir(parents=True, exist_ok=True)
    src_frames = capture_dir / "frames"
    if not src_frames.is_dir():
        raise FileNotFoundError(f"no frames/ under {capture_dir}")
    for jpg in sorted(src_frames.glob("*.jpg")):
        dst = images / jpg.name
        if not dst.exists():
            try:
                dst.symlink_to(jpg.resolve())
            except OSError:
                shutil.copy2(jpg, dst)
    (workspace / "capture_ref.json").write_text(
        json.dumps({"source": str(capture_dir)}, indent=2),
        encoding="utf-8",
    )
    return images


def run_colmap_sfm(images_dir: Path, workspace: Path) -> Dict[str, Any]:
    """Run COLMAP feature_extract + exhaustive matcher + mapper if installed."""
    db = workspace / "database.db"
    sparse = workspace / "sparse"
    sparse.mkdir(parents=True, exist_ok=True)
    log: List[str] = []

    if _which("colmap") is None:
        return {
            "status": "skipped",
            "reason": "colmap not installed",
            "workspace": str(workspace),
        }

    def run(args: List[str]) -> None:
        log.append(" ".join(args))
        subprocess.run(args, check=True)

    run([
        "colmap", "feature_extractor",
        "--database_path", str(db),
        "--image_path", str(images_dir),
        "--ImageReader.single_camera", "1",
    ])
    run(["colmap", "exhaustive_matcher", "--database_path", str(db)])
    run([
        "colmap", "mapper",
        "--database_path", str(db),
        "--image_path", str(images_dir),
        "--output_path", str(sparse),
    ])
    report = {
        "status": "ok",
        "workspace": str(workspace),
        "sparse_dir": str(sparse),
        "commands": log,
    }
    (workspace / "reconstruct_report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    return report


def quality_check(capture_dir: Path) -> Dict[str, Any]:
    n_frames = len(list((capture_dir / "frames").glob("*.jpg"))) if (capture_dir / "frames").is_dir() else 0
    traj_lines = 0
    traj = capture_dir / "traj.jsonl"
    if traj.is_file():
        traj_lines = sum(1 for _ in traj.open(encoding="utf-8"))
    return {
        "n_frames": n_frames,
        "n_traj_rows": traj_lines,
        "pose_ok": traj_lines >= n_frames * 0.8 if n_frames else False,
        "min_frames_met": n_frames >= 60,
    }
