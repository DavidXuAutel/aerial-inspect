"""Estimate bridge centroid from WAM SEARCH capture (traj.jsonl + vgoal tracker)."""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

TRACKING_STATES = frozenset({"tracking", "occluded", "arrived"})


def body_to_world_delta(yaw: float, body: Sequence[float]) -> np.ndarray:
    d_fwd, d_left, d_up = float(body[0]), float(body[1]), float(body[2])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * d_fwd - s * d_left, s * d_fwd + c * d_left, d_up], dtype=np.float64)


def _row_fields(row: Dict[str, Any]) -> Dict[str, Any]:
    """Normalize deploy (step_info) vs AirSim eval (top-level) traj rows."""
    info = row.get("step_info") or {}
    bridge_goal_rel = row.get("bridge_goal_rel") or info.get("bridge_goal_rel")
    goal_rel = bridge_goal_rel
    if goal_rel is None:
        goal_rel = info.get("goal_rel", row.get("goal_rel"))
    return {
        "tracker_state": info.get("tracker_state", row.get("tracker_state")),
        "goal_rel": goal_rel,
        "bridge_goal_rel": bridge_goal_rel,
        "using_fallback": bool(info.get("using_fallback", row.get("using_fallback", False))),
        "det_hit": bool(info.get("det_hit", row.get("det_hit", False))),
    }


def _row_counts_as_detection_sample(fields: Dict[str, Any]) -> bool:
    """Tracked lock, or area-priority SEARCH with bridge_goal_rel + det_hit."""
    state = str(fields.get("tracker_state") or "").lower()
    if state in TRACKING_STATES:
        return True
    return bool(fields.get("det_hit") and fields.get("bridge_goal_rel"))


def world_point_from_row(row: Dict[str, Any]) -> Optional[np.ndarray]:
    pos = row.get("pos")
    fields = _row_fields(row)
    goal_rel = fields.get("goal_rel")
    if pos is None or not goal_rel or len(goal_rel) < 3:
        return None
    yaw = float(row.get("yaw", 0.0))
    return np.asarray(pos, dtype=np.float64) + body_to_world_delta(yaw, goal_rel)


def standoff_goal_to_centroid(
    goal_world: np.ndarray,
    drone_pos: np.ndarray,
    standoff_m: float,
) -> np.ndarray:
    """Extrapolate from standoff waypoint (goal_rel target) to structure center."""
    vec = goal_world - drone_pos
    dist = float(np.linalg.norm(vec))
    if dist < 1e-3:
        return goal_world
    return goal_world + (vec / dist) * float(standoff_m)


def _nearest_det_lock(rows: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Closest det_hit lock (any range) — used for approach seed / survey start."""
    best_dist = float("inf")
    best: Optional[Dict[str, Any]] = None
    for row in rows:
        fields = _row_fields(row)
        if not fields["det_hit"] or fields["using_fallback"]:
            continue
        goal_rel = fields.get("goal_rel")
        if not goal_rel or len(goal_rel) < 4:
            continue
        dist_m = float(goal_rel[3])
        if dist_m >= best_dist:
            continue
        goal_world = world_point_from_row(row)
        if goal_world is None or row.get("pos") is None:
            continue
        best_dist = dist_m
        best = {
            "goal_rel_dist_m": dist_m,
            "drone_xyz": [float(v) for v in row["pos"][:3]],
            "target_xyz": [float(v) for v in goal_world[:3]],
            "step": int(row.get("step", -1)),
        }
    return best


def estimate_centroid_from_traj(
    traj_path: Path,
    *,
    extrapolate_standoff_m: Optional[float] = None,
    min_samples: int = 3,
    require_det_hit: bool = False,
    min_goal_rel_dist_m: float = 0.0,
) -> Dict[str, Any]:
    """Parse traj.jsonl; median of detected target world positions from locked tracks."""
    if not traj_path.is_file():
        raise FileNotFoundError(f"missing traj: {traj_path}")

    centroids: List[np.ndarray] = []
    parsed_rows: List[Dict[str, Any]] = []
    n_rows = 0
    n_tracked = 0
    n_det_hit = 0
    for line in traj_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        n_rows += 1
        row = json.loads(line)
        parsed_rows.append(row)
        fields = _row_fields(row)
        if require_det_hit and not fields["det_hit"]:
            continue
        if fields["using_fallback"]:
            continue
        if not _row_counts_as_detection_sample(fields):
            continue
        goal_rel = fields.get("goal_rel")
        if min_goal_rel_dist_m > 0 and goal_rel and len(goal_rel) >= 4:
            if float(goal_rel[3]) < min_goal_rel_dist_m:
                continue
        goal_world = world_point_from_row(row)
        if goal_world is None:
            continue
        n_tracked += 1
        if bool(row.get("det_hit")):
            n_det_hit += 1
        if extrapolate_standoff_m is not None and extrapolate_standoff_m > 0:
            drone = np.asarray(row["pos"], dtype=np.float64)
            centroids.append(standoff_goal_to_centroid(goal_world, drone, extrapolate_standoff_m))
        else:
            centroids.append(goal_world)

    if len(centroids) < min_samples:
        return {
            "status": "failed",
            "reason": f"insufficient tracked frames ({len(centroids)} < {min_samples})",
            "n_rows": n_rows,
            "n_tracked": n_tracked,
            "n_det_hit": n_det_hit,
            "require_det_hit": require_det_hit,
            "centroid_xyz": None,
        }

    stack = np.stack(centroids, axis=0)
    med = np.median(stack, axis=0)
    result: Dict[str, Any] = {
        "status": "ok",
        "centroid_xyz": [float(med[0]), float(med[1]), float(med[2])],
        "n_rows": n_rows,
        "n_tracked": n_tracked,
        "n_det_hit": n_det_hit,
        "require_det_hit": require_det_hit,
        "n_samples": len(centroids),
        "std_m": [float(stack[:, i].std()) for i in range(3)],
        "source_traj": str(traj_path),
    }
    nearest = _nearest_det_lock(parsed_rows)
    if nearest is not None:
        result["nearest_lock"] = nearest
    return result


def estimate_span_axis_from_traj(
    traj_path: Path,
    *,
    min_samples: int = 5,
) -> Dict[str, Any]:
    """PCA on tracked target XY positions → bridge long-axis bearing (deg, 0=north)."""
    if not traj_path.is_file():
        raise FileNotFoundError(f"missing traj: {traj_path}")

    points: List[np.ndarray] = []
    for line in traj_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        row = json.loads(line)
        fields = _row_fields(row)
        if fields["using_fallback"]:
            continue
        if not _row_counts_as_detection_sample(fields):
            continue
        goal_world = world_point_from_row(row)
        if goal_world is None:
            continue
        points.append(goal_world[:2])

    if len(points) < min_samples:
        return {
            "status": "failed",
            "reason": f"insufficient tracked XY samples ({len(points)} < {min_samples})",
            "span_axis_deg": None,
            "n_samples": len(points),
        }

    pts = np.stack(points, axis=0)
    centered = pts - np.mean(pts, axis=0)
    cov = centered.T @ centered / max(len(points), 1)
    eigvals, eigvecs = np.linalg.eigh(cov)
    principal = eigvecs[:, int(np.argmax(eigvals))]
    axis_deg = float(math.degrees(math.atan2(principal[1], principal[0])))
    return {
        "status": "ok",
        "span_axis_deg": axis_deg,
        "n_samples": len(points),
        "eigenvalues_xy": [float(eigvals[0]), float(eigvals[1])],
        "source_traj": str(traj_path),
    }


def write_detected_centroid(mission_dir: Path, result: Dict[str, Any]) -> Path:
    out = mission_dir / "detected_centroid.json"
    out.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    return out
