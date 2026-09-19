"""Triangulate SIFT matches using known AirSim poses (bypasses COLMAP mapper)."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from colmap_known_pose import (
    _image_size,
    _intrinsics_from_hfov,
    load_traj_poses,
    traj_row_to_world_to_cam,
)


def _projection_matrix(k: np.ndarray, r: np.ndarray, t: np.ndarray) -> np.ndarray:
    return k @ np.hstack([r, t.reshape(3, 1)])


def _reproj_err(xyz: np.ndarray, k: np.ndarray, r: np.ndarray, t: np.ndarray, uv: np.ndarray) -> float:
    xc = r @ xyz + t
    if xc[2] <= 1e-3:
        return 1e9
    proj = k @ xc
    proj = proj[:2] / proj[2]
    return float(np.linalg.norm(proj - uv))


def _frame_index(name: str) -> int:
    stem = Path(name).stem
    if "_" in stem:
        return int(stem.split("_")[-1])
    return 0


def _infer_lap_stride(n_frames: int) -> int:
    """Dual-lap surveys: also match frame i with i+N/2 (same bearing, different altitude)."""
    if n_frames >= 16 and n_frames % 2 == 0:
        return n_frames // 2
    return 0


def _pair_ok(
    name1: str,
    name2: str,
    n_frames: int,
    overlap: int,
    lap_stride: int,
) -> bool:
    """Pair frames within each lap (with wrap) + cross-lap same-bearing + opposite ellipse."""
    i1, i2 = _frame_index(name1), _frame_index(name2)
    if i1 == i2:
        return False

    if lap_stride > 0:
        lap1, local1 = divmod(i1, lap_stride)
        lap2, local2 = divmod(i2, lap_stride)
        # same bearing, different altitude (double lap)
        if local1 == local2 and lap1 != lap2:
            return True
        if lap1 == lap2:
            d_local = abs(local2 - local1)
            d_wrap = min(d_local, lap_stride - d_local)
            if d_wrap <= overlap:
                return True
            # opposite side of ellipse — strong baseline for along-span views
            if lap_stride >= 8 and d_wrap == lap_stride // 2:
                return True
        return False

    d = abs(i2 - i1)
    d_wrap = min(d, n_frames - d)
    return d_wrap <= overlap


def _estimate_pitch_offset_rad(
    poses: dict[str, dict],
    k: np.ndarray,
    db: object,
    images: dict,
    keypoints: dict,
    n_frames: int,
    overlap: int,
    lap_stride: int,
    max_reproj_px: float,
    max_samples: int = 400,
) -> float:
    """Grid-search small camera pitch bias to maximize reproj inliers (sim extrinsic error)."""
    import pycolmap

    pair_ids = db.read_all_matches()[0]
    samples: list[tuple] = []
    for pair_id in pair_ids:
        image_id2 = int(pair_id % 2147483648)
        image_id1 = int(pair_id // 2147483648)
        if image_id1 not in keypoints or image_id2 not in keypoints:
            continue
        name1 = images[image_id1].name
        name2 = images[image_id2].name
        if name1 not in poses or name2 not in poses:
            continue
        if not _pair_ok(name1, name2, n_frames, overlap, lap_stride):
            continue
        matches = np.asarray(db.read_matches(image_id1, image_id2), dtype=np.int64)
        if matches.size == 0:
            continue
        kp1 = keypoints[image_id1]
        kp2 = keypoints[image_id2]
        for idx1, idx2 in matches[: min(20, len(matches))]:
            samples.append((name1, name2, kp1[int(idx1)], kp2[int(idx2)]))
        if len(samples) >= max_samples:
            break

    if len(samples) < 30:
        return 0.0

    best_off, best_n = 0.0, -1
    for off_deg in np.linspace(-6.0, 6.0, 25):
        off = math.radians(float(off_deg))
        cam_cache: dict[str, tuple] = {}
        n_ok = 0
        for name1, name2, uv1, uv2 in samples:
            if name1 not in cam_cache:
                r, t = traj_row_to_world_to_cam(poses[name1], pitch_offset_rad=off)
                cam_cache[name1] = (k, r, t)
            if name2 not in cam_cache:
                r, t = traj_row_to_world_to_cam(poses[name2], pitch_offset_rad=off)
                cam_cache[name2] = (k, r, t)
            k1, r1, t1 = cam_cache[name1]
            k2, r2, t2 = cam_cache[name2]
            p1 = _projection_matrix(k1, r1, t1)
            p2 = _projection_matrix(k2, r2, t2)
            x_h = cv2_triangulate(p1, p2, uv1, uv2)
            if x_h is None:
                continue
            if _reproj_err(x_h, k1, r1, t1, uv1) <= max_reproj_px and _reproj_err(
                x_h, k2, r2, t2, uv2
            ) <= max_reproj_px:
                n_ok += 1
        if n_ok > best_n:
            best_n = n_ok
            best_off = off

    triangulate_matches.last_pitch_offset_deg = math.degrees(best_off)  # type: ignore[attr-defined]
    return best_off


def triangulate_matches(
    database_path: Path,
    images_dir: Path,
    traj_path: Path,
    hfov_deg: float = 80.0,
    max_reproj_px: float = 6.0,
    min_tri_angle_deg: float = 0.3,
    sequential_overlap: int = 10,
    lap_stride: int = 0,
    auto_pitch: bool = True,
    max_dist_from_centroid_m: float = 0.0,
    centroid_xyz: list[float] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    import pycolmap

    poses = load_traj_poses(traj_path)
    width, height = _image_size(images_dir)
    fx, fy, cx, cy = _intrinsics_from_hfov(width, height, hfov_deg)
    k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

    n_frames = len(poses)
    if lap_stride <= 0:
        lap_stride = _infer_lap_stride(n_frames)

    db = pycolmap.Database.open(str(database_path))
    images = {img.image_id: img for img in db.read_all_images()}

    keypoints: dict[int, np.ndarray] = {}
    for image_id, img in images.items():
        if img.name not in poses:
            continue
        kp = db.read_keypoints(image_id)
        keypoints[image_id] = np.asarray(kp, dtype=np.float64)[:, :2]

    pitch_off = 0.0
    if auto_pitch:
        pitch_off = _estimate_pitch_offset_rad(
            poses,
            k,
            db,
            images,
            keypoints,
            n_frames,
            sequential_overlap,
            lap_stride,
            max_reproj_px,
        )

    cam_by_name: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
    for name, row in poses.items():
        r, t = traj_row_to_world_to_cam(row, pitch_offset_rad=pitch_off)
        cam_by_name[name] = (k, r, t)

    points: list[np.ndarray] = []
    colors: list[np.ndarray] = []
    stats = {
        "pairs_ok": 0,
        "matches_raw": 0,
        "pass_reproj": 0,
        "pass_angle": 0,
        "lap_stride": lap_stride,
        "pitch_offset_deg": round(math.degrees(pitch_off), 2),
    }

    from PIL import Image

    rgb_cache: dict[int, np.ndarray] = {}

    def _color_at(image_id: int, uv: np.ndarray) -> np.ndarray:
        if image_id not in rgb_cache:
            name = images[image_id].name
            rgb_cache[image_id] = np.array(Image.open(images_dir / name).convert("RGB"))
        img = rgb_cache[image_id]
        u = int(np.clip(round(uv[0]), 0, img.shape[1] - 1))
        v = int(np.clip(round(uv[1]), 0, img.shape[0] - 1))
        return img[v, u].astype(np.uint8)

    pair_ids = db.read_all_matches()[0]
    for pair_id in pair_ids:
        image_id2 = int(pair_id % 2147483648)
        image_id1 = int(pair_id // 2147483648)
        if image_id1 not in keypoints or image_id2 not in keypoints:
            continue
        name1 = images[image_id1].name
        name2 = images[image_id2].name
        if name1 not in cam_by_name or name2 not in cam_by_name:
            continue
        if not _pair_ok(name1, name2, n_frames, sequential_overlap, lap_stride):
            continue
        stats["pairs_ok"] += 1
        k1, r1, t1 = cam_by_name[name1]
        k2, r2, t2 = cam_by_name[name2]
        p1 = _projection_matrix(k1, r1, t1)
        p2 = _projection_matrix(k2, r2, t2)
        matches = np.asarray(db.read_matches(image_id1, image_id2), dtype=np.int64)
        if matches.size == 0:
            continue
        kp1 = keypoints[image_id1]
        kp2 = keypoints[image_id2]
        stats["matches_raw"] += int(len(matches))
        for idx1, idx2 in matches:
            uv1 = kp1[int(idx1)]
            uv2 = kp2[int(idx2)]
            x_h = cv2_triangulate(p1, p2, uv1, uv2)
            if x_h is None:
                continue
            err1 = _reproj_err(x_h, k1, r1, t1, uv1)
            err2 = _reproj_err(x_h, k2, r2, t2, uv2)
            if err1 > max_reproj_px or err2 > max_reproj_px:
                continue
            stats["pass_reproj"] += 1
            ang = _triangulation_angle(
                x_h,
                -r1.T @ t1,
                -r2.T @ t2,
            )
            if ang < min_tri_angle_deg:
                continue
            stats["pass_angle"] += 1
            points.append(x_h)
            colors.append(_color_at(image_id1, uv1))

    if max_dist_from_centroid_m > 0 and centroid_xyz and points:
        c = np.asarray(centroid_xyz[:3], dtype=np.float64)
        keep = [
            i
            for i, p in enumerate(points)
            if float(np.linalg.norm(np.asarray(p) - c)) <= max_dist_from_centroid_m
        ]
        stats["outlier_filter_m"] = max_dist_from_centroid_m
        stats["points_before_filter"] = len(points)
        points = [points[i] for i in keep]
        colors = [colors[i] for i in keep]
        stats["points_after_filter"] = len(points)

    triangulate_matches.last_stats = stats  # type: ignore[attr-defined]
    if not points:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.uint8)
    return np.asarray(points), np.asarray(colors)


def cv2_triangulate(p1: np.ndarray, p2: np.ndarray, uv1: np.ndarray, uv2: np.ndarray) -> np.ndarray | None:
    import cv2

    x_h = cv2.triangulatePoints(p1, p2, uv1.reshape(2, 1), uv2.reshape(2, 1)).reshape(4)
    if abs(x_h[3]) < 1e-9:
        return None
    xyz = (x_h[:3] / x_h[3]).astype(np.float64)
    if not np.all(np.isfinite(xyz)):
        return None
    return xyz


def _triangulation_angle(x: np.ndarray, c1: np.ndarray, c2: np.ndarray) -> float:
    v1 = c1 - x
    v2 = c2 - x
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 0.0
    cos_a = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_a)))


def write_ply(points: np.ndarray, colors: np.ndarray, path: Path) -> None:
    with path.open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for (x, y, z), (r, g, b) in zip(points, colors):
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n")


def write_preview(points: np.ndarray, colors: np.ndarray, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    step = max(1, len(points) // 8000)
    ax.scatter(
        points[::step, 0],
        points[::step, 1],
        points[::step, 2],
        c=colors[::step] / 255.0,
        s=1,
        linewidths=0,
    )
    ax.set_title(f"known-pose triangulation ({len(points)} points)")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
