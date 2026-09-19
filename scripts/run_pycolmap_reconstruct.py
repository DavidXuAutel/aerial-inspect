#!/usr/bin/env python3
"""Run COLMAP via pycolmap: known-pose triangulation (sim) or blind SfM fallback."""
from __future__ import annotations

import argparse
import json
import shutil
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _export_ply_and_preview(sparse_model_dir: Path, out_dir: Path) -> dict:
    import numpy as np

    try:
        import pycolmap
    except ImportError as exc:
        raise SystemExit(f"pycolmap not installed: {exc}") from exc

    rec = pycolmap.Reconstruction(str(sparse_model_dir))
    pts = []
    colors = []
    for pid in rec.points3D:
        p = rec.points3D[pid]
        pts.append(p.xyz)
        colors.append(p.color)
    if not pts:
        return {"ply": None, "preview": None, "n_points": 0}

    pts_arr = np.asarray(pts, dtype=np.float64)
    colors_arr = np.asarray(colors, dtype=np.uint8)
    ply_path = out_dir / "sparse_points.ply"
    with ply_path.open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts_arr)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for (x, y, z), (r, g, b) in zip(pts_arr, colors_arr):
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n")

    preview_path = out_dir / "sparse_preview.png"
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")
        step = max(1, len(pts_arr) // 8000)
        ax.scatter(
            pts_arr[::step, 0],
            pts_arr[::step, 1],
            pts_arr[::step, 2],
            c=colors_arr[::step] / 255.0,
            s=1,
            linewidths=0,
        )
        ax.set_title(f"COLMAP sparse ({len(pts_arr)} points)")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        fig.tight_layout()
        fig.savefig(preview_path, dpi=150)
        plt.close(fig)
    except Exception as exc:  # noqa: BLE001
        preview_path = None
        print(f"preview skipped: {exc}")

    return {
        "ply": str(ply_path),
        "preview": str(preview_path) if preview_path else None,
        "n_points": len(pts_arr),
    }


def _hfov_from_capture(capture: Path) -> float:
    mission_dir = capture.parent
    spec_path = mission_dir / "mission_spec.json"
    if spec_path.is_file():
        hfov = float(json.loads(spec_path.read_text()).get("survey", {}).get("camera_hfov_deg", 80.0))
        return hfov
    return 80.0


def _prepare_images(frames: Path, images: Path) -> None:
    images.mkdir(parents=True, exist_ok=True)
    for jpg in sorted(frames.glob("*.jpg")):
        dst = images / jpg.name
        if dst.exists():
            dst.unlink()
        try:
            dst.symlink_to(jpg.resolve())
        except OSError:
            shutil.copy2(jpg, dst)
        # Prefer real files: next survey round deletes capture_survey and breaks symlinks.
        if dst.is_symlink():
            try:
                real = jpg.resolve()
                dst.unlink()
                shutil.copy2(real, dst)
            except OSError:
                pass


def _match_features(db: Path, mode: str, overlap: int) -> None:
    import pycolmap

    # Known-pose triangulation filters adjacent pairs in software; exhaustive
    # matching yields far more pair inliers than match_sequential alone.
    if mode in ("hybrid", "known_pose"):
        print("match_exhaustive (known-pose; triangulation uses sequential pairs)...", flush=True)
        pycolmap.match_exhaustive(database_path=str(db))
        return

    if mode == "sequential":
        try:
            opts = pycolmap.SequentialPairingOptions()
            opts.overlap = int(overlap)
            opts.loop_detection = False
            pycolmap.match_sequential(database_path=str(db), pairing_options=opts)
            return
        except (AttributeError, TypeError) as exc:
            print(f"match_sequential unavailable ({exc}); falling back to exhaustive", flush=True)
    print("match_exhaustive...", flush=True)
    pycolmap.match_exhaustive(database_path=str(db))


def _run_known_pose(
    capture: Path,
    workspace: Path,
    images: Path,
    hfov_deg: float,
    *,
    match_mode: str = "sequential",
    sequential_overlap: int = 10,
    lap_stride: int = 0,
    max_reproj_px: float = 6.0,
    min_tri_angle_deg: float = 0.3,
    max_dist_from_centroid_m: float = 1200.0,
) -> dict:
    import sys

    sys.path.insert(0, str(ROOT / "scripts"))
    import pycolmap
    from colmap_known_pose import _image_size, _intrinsics_from_hfov, load_traj_poses
    from triangulate_from_traj import triangulate_matches, write_ply, write_preview

    traj = capture / "traj.jsonl"
    if not traj.is_file():
        raise SystemExit(f"missing {traj} for known-pose mode")

    db = workspace / "database.db"
    sparse = workspace / "sparse"
    sparse.mkdir(parents=True, exist_ok=True)
    if db.exists():
        db.unlink()
    for child in sparse.iterdir():
        if child.is_dir():
            shutil.rmtree(child)
        else:
            child.unlink()

    width, height = _image_size(images)
    fx, fy, cx, cy = _intrinsics_from_hfov(width, height, hfov_deg)
    reader = pycolmap.ImageReaderOptions()
    reader.camera_model = "PINHOLE"
    reader.camera_params = f"{fx},{fy},{cx},{cy}"

    t0 = time.time()
    print("extract_features...", flush=True)
    pycolmap.extract_features(
        database_path=str(db),
        image_path=str(images),
        reader_options=reader,
    )
    print(f"match ({match_mode}, overlap={sequential_overlap})...", flush=True)
    _match_features(db, match_mode, sequential_overlap)

    centroid_xyz = None
    spec_path = capture.parent / "mission_spec.json"
    if spec_path.is_file():
        centroid_xyz = json.loads(spec_path.read_text()).get("bridge_centroid_xyz")

    print("triangulate_matches (known AirSim poses)...", flush=True)
    pts, cols = triangulate_matches(
        db,
        images,
        traj,
        hfov_deg=hfov_deg,
        max_reproj_px=max_reproj_px,
        min_tri_angle_deg=min_tri_angle_deg,
        sequential_overlap=sequential_overlap,
        lap_stride=lap_stride,
        max_dist_from_centroid_m=max_dist_from_centroid_m,
        centroid_xyz=centroid_xyz,
    )
    tri_stats = getattr(triangulate_matches, "last_stats", {})
    if lap_stride <= 0 and tri_stats.get("lap_stride"):
        lap_stride = int(tri_stats["lap_stride"])
    ply_path = workspace / "sparse_points.ply"
    preview_path = workspace / "sparse_preview.png"
    write_ply(pts, cols, ply_path)
    write_preview(pts, cols, preview_path)

    report = {
        "status": "ok",
        "mode": "known_pose",
        "n_models": 1,
        "workspace": str(workspace),
        "sparse_dir": str(sparse),
        "elapsed_s": round(time.time() - t0, 1),
        "pycolmap": pycolmap.COLMAP_version,
        "n_input_images": len(list((capture / "frames").glob("*.jpg"))),
        "num_registered_images": len(load_traj_poses(traj)),
        "num_points3D": int(len(pts)),
        "hfov_deg": hfov_deg,
        "max_reproj_px": max_reproj_px,
        "min_tri_angle_deg": min_tri_angle_deg,
        "sequential_overlap": sequential_overlap,
        "lap_stride": lap_stride,
        "triangulation": tri_stats,
        "traj": str(traj),
        "exports": {
            "ply": str(ply_path),
            "preview": str(preview_path),
            "n_points": int(len(pts)),
        },
    }
    return report


def _run_hybrid(
    capture: Path,
    workspace: Path,
    images: Path,
    hfov_deg: float,
    sequential_overlap: int,
    lap_stride: int,
    max_reproj_px: float,
    min_tri_angle_deg: float,
    max_dist_from_centroid_m: float = 1200.0,
) -> dict:
    """Known-pose triangulation with exhaustive matching (default product path)."""
    report = _run_known_pose(
        capture,
        workspace,
        images,
        hfov_deg,
        match_mode="hybrid",
        sequential_overlap=sequential_overlap,
        lap_stride=lap_stride,
        max_reproj_px=max_reproj_px,
        min_tri_angle_deg=min_tri_angle_deg,
        max_dist_from_centroid_m=max_dist_from_centroid_m,
    )
    report["mode"] = "hybrid"
    return report


def _run_blind_sfm(
    capture: Path,
    workspace: Path,
    images: Path,
    *,
    sequential_overlap: int = 6,
) -> dict:
    import pycolmap

    frames = capture / "frames"
    db = workspace / "database.db"
    sparse = workspace / "sparse"
    sparse.mkdir(parents=True, exist_ok=True)
    if db.exists():
        db.unlink()
    for child in sparse.iterdir():
        if child.is_dir():
            shutil.rmtree(child)
        else:
            child.unlink()

    t0 = time.time()
    print("extract_features...", flush=True)
    pycolmap.extract_features(database_path=str(db), image_path=str(images))
    print(f"match (sequential, overlap={sequential_overlap})...", flush=True)
    _match_features(db, "sequential", sequential_overlap)
    print("incremental_mapping (blind SfM)...", flush=True)
    maps = pycolmap.incremental_mapping(
        database_path=str(db),
        image_path=str(images),
        output_path=str(sparse),
    )

    report: dict = {
        "status": "ok" if maps else "empty",
        "mode": "blind_sfm",
        "n_models": len(maps) if maps else 0,
        "workspace": str(workspace),
        "sparse_dir": str(sparse),
        "elapsed_s": round(time.time() - t0, 1),
        "pycolmap": pycolmap.COLMAP_version,
        "n_input_images": len(list(frames.glob("*.jpg"))),
    }
    if maps:
        m0 = maps[0]
        report["num_registered_images"] = m0.num_images()
        report["num_points3D"] = m0.num_points3D()
        model0 = sparse / "0"
        if model0.is_dir():
            report["exports"] = _export_ply_and_preview(model0, workspace)
    return report


def main() -> int:
    p = argparse.ArgumentParser(description="COLMAP reconstruct via pycolmap")
    p.add_argument("capture_dir", help="capture dir with frames/")
    p.add_argument("--workspace", help="output workspace (default artifacts/models/<mission_id>)")
    p.add_argument(
        "--mode",
        choices=("auto", "hybrid", "known_pose", "blind_sfm"),
        default="auto",
        help="auto: hybrid if traj.jsonl present else blind_sfm",
    )
    p.add_argument("--hfov-deg", type=float, default=0.0, help="override camera HFOV")
    p.add_argument("--sequential-overlap", type=int, default=10, help="adjacent-frame match window")
    p.add_argument(
        "--lap-stride",
        type=int,
        default=0,
        help="also triangulate frame pairs i,i+stride (0=auto N/2 for even lap counts)",
    )
    p.add_argument("--max-reproj-px", type=float, default=6.0, help="triangulation reproj gate (px)")
    p.add_argument("--min-tri-angle-deg", type=float, default=0.3, help="min triangulation angle")
    p.add_argument(
        "--max-dist-from-centroid-m",
        type=float,
        default=1200.0,
        help="drop triangulated points farther than this from bridge centroid (0=off)",
    )
    args = p.parse_args()

    capture = Path(args.capture_dir).resolve()
    frames = capture / "frames"
    if not frames.is_dir():
        raise SystemExit(f"missing frames under {capture}")

    if args.workspace:
        workspace = Path(args.workspace).resolve()
    else:
        workspace = capture.parent.parent / "models" / capture.parent.name
    images = workspace / "images"
    _prepare_images(frames, images)

    mode = args.mode
    if mode == "auto":
        mode = "hybrid" if (capture / "traj.jsonl").is_file() else "blind_sfm"

    hfov = args.hfov_deg if args.hfov_deg > 0 else _hfov_from_capture(capture)
    overlap = int(args.sequential_overlap)
    lap_stride = int(args.lap_stride)
    reproj = float(args.max_reproj_px)
    tri_ang = float(args.min_tri_angle_deg)
    max_dist = float(args.max_dist_from_centroid_m)

    if mode == "hybrid":
        report = _run_hybrid(
            capture, workspace, images, hfov, overlap, lap_stride, reproj, tri_ang, max_dist
        )
    elif mode == "known_pose":
        report = _run_known_pose(
            capture,
            workspace,
            images,
            hfov,
            sequential_overlap=overlap,
            lap_stride=lap_stride,
            max_reproj_px=reproj,
            min_tri_angle_deg=tri_ang,
            max_dist_from_centroid_m=max_dist,
        )
    else:
        report = _run_blind_sfm(capture, workspace, images, sequential_overlap=overlap)

    (workspace / "reconstruct_report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    return 0 if report.get("status") == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
