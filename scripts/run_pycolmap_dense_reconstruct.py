#!/usr/bin/env python3
"""Dense MVS (patch-match stereo + fusion + Poisson mesh) from known AirSim poses.

Unlike the sparse known-pose triangulation in run_pycolmap_reconstruct.py (which
only keeps SIFT-feature points that survive two-view triangulation — sparse and
easily dominated by background/reflection outliers), this runs COLMAP's dense
pipeline against the *known* camera poses from traj.jsonl:

  1. Write a COLMAP sparse model (cameras+images, no points needed) from the
     known AirSim poses (reuses colmap_known_pose.write_colmap_text_model).
  2. undistort_images()      -> per-camera undistorted images + stereo config.
  3. patch_match_stereo()    -> per-pixel depth+normal maps (requires CUDA).
  4. stereo_fusion()         -> fused dense point cloud (fused.ply).
  5. poisson_meshing()       -> watertight mesh from the fused cloud (optional).

This produces orders of magnitude more points than sparse SfM and should make
the bridge deck/towers/cables visually recognizable in point-cloud previews.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def _build_known_pose_sparse_model(traj_path: Path, images_dir: Path, hfov_deg: float, text_dir: Path, bin_dir: Path) -> Path:
    from colmap_known_pose import write_colmap_text_model

    import pycolmap

    if text_dir.exists():
        shutil.rmtree(text_dir)
    n = write_colmap_text_model(traj_path, images_dir, text_dir, hfov_deg=hfov_deg)
    print(f"wrote known-pose text model: {n} images", flush=True)

    if bin_dir.exists():
        shutil.rmtree(bin_dir)
    bin_dir.mkdir(parents=True, exist_ok=True)
    rec = pycolmap.Reconstruction()
    rec.read_text(str(text_dir))
    rec.write_binary(str(bin_dir))
    return bin_dir


def main() -> int:
    p = argparse.ArgumentParser(description="Dense MVS from known AirSim poses (pycolmap)")
    p.add_argument("capture_dir", help="capture dir with frames/ + traj.jsonl")
    p.add_argument("--workspace", help="workspace (default artifacts/models/<mission_id>)")
    p.add_argument("--hfov-deg", type=float, default=0.0)
    p.add_argument("--max-image-size", type=int, default=1600)
    p.add_argument("--gpu-index", default="0")
    p.add_argument("--skip-mesh", action="store_true")
    args = p.parse_args()

    import pycolmap

    from run_pycolmap_reconstruct import _hfov_from_capture, _prepare_images

    capture = Path(args.capture_dir).resolve()
    frames = capture / "frames"
    traj = capture / "traj.jsonl"
    if not frames.is_dir():
        raise SystemExit(f"missing {frames}")
    if not traj.is_file():
        raise SystemExit(f"missing {traj} (dense mode requires known poses)")

    workspace = Path(args.workspace).resolve() if args.workspace else (capture.parent.parent / "models" / capture.parent.name)
    images_dir = workspace / "images"
    if not any(images_dir.glob("*.jpg")):
        _prepare_images(frames, images_dir)

    hfov = args.hfov_deg if args.hfov_deg > 0 else _hfov_from_capture(capture)

    text_dir = workspace / "sparse_known_pose_txt"
    bin_dir = workspace / "sparse_known_pose_bin"
    bin_model = _build_known_pose_sparse_model(traj, images_dir, hfov, text_dir, bin_dir)

    dense_dir = workspace / "dense"
    if dense_dir.exists():
        shutil.rmtree(dense_dir)
    dense_dir.mkdir(parents=True)

    t0 = time.time()
    print("undistort_images...", flush=True)
    undistort_opts = pycolmap.UndistortCameraOptions()
    undistort_opts.max_image_size = int(args.max_image_size)
    pycolmap.undistort_images(
        output_path=str(dense_dir),
        input_path=str(bin_model),
        image_path=str(images_dir),
        undistort_options=undistort_opts,
    )
    t_undistort = time.time() - t0

    print("patch_match_stereo (CUDA)...", flush=True)
    t1 = time.time()
    pm_opts = pycolmap.PatchMatchOptions()
    pm_opts.gpu_index = str(args.gpu_index)
    pm_opts.max_image_size = int(args.max_image_size)
    pycolmap.patch_match_stereo(workspace_path=str(dense_dir), options=pm_opts)
    t_pms = time.time() - t1

    fused_path = dense_dir / "fused.ply"
    print("stereo_fusion...", flush=True)
    t2 = time.time()
    fusion_opts = pycolmap.StereoFusionOptions()
    fusion_opts.max_image_size = int(args.max_image_size)
    fused_rec = pycolmap.stereo_fusion(
        output_path=str(fused_path),
        workspace_path=str(dense_dir),
        options=fusion_opts,
    )
    t_fusion = time.time() - t2

    report: dict = {
        "status": "ok",
        "n_input_images": len(list(frames.glob("*.jpg"))),
        "hfov_deg": hfov,
        "max_image_size": int(args.max_image_size),
        "elapsed_undistort_s": round(t_undistort, 1),
        "elapsed_patch_match_stereo_s": round(t_pms, 1),
        "elapsed_stereo_fusion_s": round(t_fusion, 1),
        "elapsed_total_s": round(time.time() - t0, 1),
        "fused_ply": str(fused_path),
        "n_dense_points": int(fused_rec.num_points3D()) if fused_rec is not None else None,
    }

    if not args.skip_mesh and fused_path.is_file():
        mesh_path = dense_dir / "meshed_poisson.ply"
        print("poisson_meshing...", flush=True)
        t3 = time.time()
        try:
            pycolmap.poisson_meshing(input_path=str(fused_path), output_path=str(mesh_path))
            report["mesh_ply"] = str(mesh_path)
            report["elapsed_poisson_s"] = round(time.time() - t3, 1)
        except Exception as exc:  # noqa: BLE001
            report["mesh_error"] = str(exc)

    (workspace / "dense_reconstruct_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
