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
    return {
        "tracker_state": info.get("tracker_state", row.get("tracker_state")),
        "goal_rel": info.get("goal_rel", row.get("goal_rel")),
        "using_fallback": bool(info.get("using_fallback", row.get("using_fallback", False))),
    }


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


def estimate_centroid_from_traj(
    traj_path: Path,
    *,
    extrapolate_standoff_m: Optional[float] = None,
    min_samples: int = 3,
) -> Dict[str, Any]:
    """Parse traj.jsonl; median of detected target world positions from locked tracks."""
    if not traj_path.is_file():
        raise FileNotFoundError(f"missing traj: {traj_path}")

    centroids: List[np.ndarray] = []
    n_rows = 0
    n_tracked = 0
    for line in traj_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        n_rows += 1
        row = json.loads(line)
        fields = _row_fields(row)
        if fields["using_fallback"]:
            continue
        state = str(fields.get("tracker_state") or "").lower()
        if state not in TRACKING_STATES:
            continue
        goal_world = world_point_from_row(row)
        if goal_world is None:
            continue
        n_tracked += 1
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
            "centroid_xyz": None,
        }

    stack = np.stack(centroids, axis=0)
    med = np.median(stack, axis=0)
    return {
        "status": "ok",
        "centroid_xyz": [float(med[0]), float(med[1]), float(med[2])],
        "n_rows": n_rows,
        "n_tracked": n_tracked,
        "n_samples": len(centroids),
        "std_m": [float(stack[:, i].std()) for i in range(3)],
        "source_traj": str(traj_path),
    }


def write_detected_centroid(mission_dir: Path, result: Dict[str, Any]) -> Path:
    out = mission_dir / "detected_centroid.json"
    out.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    return out
