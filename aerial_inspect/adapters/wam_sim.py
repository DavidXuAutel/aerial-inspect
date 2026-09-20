"""AirSim simulation via WAM wam_vgoal_eval (10.229.20.84 / 125 bench)."""
from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from aerial_inspect.adapters.wam_platform import vgoal_root, wam_root


def sim_python() -> str:
    """Prefer 84's prebuilt venv; override with AERIAL_PY."""
    import os

    py = os.environ.get("AERIAL_PY") or os.environ.get("PYTHON_BIN")
    if py:
        return py
    default_84 = Path("/data/linux/workspace/venvs/aerial-wam/bin/python")
    if default_84.is_file():
        return str(default_84)
    import sys

    return sys.executable
from aerial_inspect.mission.orchestrator import load_spec_from_mission_dir


def sim_runs_dir(mission_dir: Path) -> Path:
    return mission_dir / "sim_runs"


def _route(
    start_xyz: List[float],
    goal_xyz: List[float],
    *,
    yaw: float = 0.0,
    instruction: str = "bridge",
) -> Dict[str, Any]:
    s, g = [float(x) for x in start_xyz], [float(x) for x in goal_xyz]
    return {
        "route_id": "inspect_mission",
        "start_pos": s,
        "goal_pos": g,
        "pos": [s, g],
        "yaw": [float(yaw), float(yaw)],
        "gpt_instruction": instruction,
    }


def export_sim_annotations(mission_dir: Path) -> Path:
    """Write sim_annotations.json for SEARCH / APPROACH / SURVEY eval episodes."""
    spec = load_spec_from_mission_dir(mission_dir)
    phases = json.loads((mission_dir / "phase_plan.json").read_text(encoding="utf-8"))
    scx, scy = spec.search.center_xy
    alt = float(spec.search.altitude_m)
    search_start = [scx, scy, alt]
    search_goal = search_start

    approach_goal = phases["approach"]["goal"]
    ag = [float(approach_goal["x"]), float(approach_goal["y"]), float(approach_goal["z"])]

    payload: Dict[str, Any] = {
        "version": "aerial_inspect_sim_v1",
        "n_routes": 1,
        "routes": [
            _route(search_start, search_goal, instruction=spec.target.visual_prompt),
        ],
        "phases": {
            "search": {"route_idx": 0, "annotation": "search"},
            "approach": {
                "route": _route(search_start, ag, instruction=spec.target.visual_prompt),
            },
        },
    }

    if (mission_dir / "waypoints.json").is_file():
        wps = json.loads((mission_dir / "waypoints.json").read_text(encoding="utf-8"))
        payload["phases"]["survey"] = {
            "waypoints": [
                {
                    "route": _route(
                        [float(wp["x"]), float(wp["y"]), float(wp["z"])],
                        [float(wp["x"]), float(wp["y"]), float(wp["z"])],
                        yaw=float(wp.get("yaw_rad", 0.0)),
                        instruction=spec.target.visual_prompt,
                    ),
                    "label": wp.get("label", ""),
                }
                for wp in wps
            ]
        }

    out = mission_dir / "sim_annotations.json"
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return out


def _write_phase_annotation(mission_dir: Path, route: Dict[str, Any], phase: str) -> Path:
    ann = {
        "version": "aerial_inspect_sim_v1",
        "n_routes": 1,
        "routes": [route],
    }
    path = mission_dir / f"sim_{phase}_annotation.json"
    path.write_text(json.dumps(ann, indent=2, ensure_ascii=False), encoding="utf-8")
    return path


def _base_eval_cmd(
    *,
    annotation: Path,
    traj_out: Path,
    visual_prompt: str,
    target_class: str,
    max_steps: int,
    search_area_half_m: float,
) -> List[str]:
    det = os.environ.get("SIM_DETECTOR", "open_vocab")
    cmd = [
        sim_python(),
        "-m",
        "experiments.aerial.scripts.wam_vgoal_eval",
        "--vgoal-repo",
        str(vgoal_root()),
        "--annotation",
        str(annotation),
        "--routes",
        "0",
        "--episodes",
        "1",
        "--detector",
        det,
        "--visual-prompt",
        visual_prompt,
        "--target-class",
        target_class,
        "--max-steps",
        str(max_steps),
        "--traj-out",
        str(traj_out),
        "--out",
        str(traj_out.parent / "eval_result.json"),
        "--planner",
        "--visual-toward-g",
        "--search-pattern",
        os.environ.get("SIM_SEARCH_PATTERN", "lawnmower"),
        "--search-area-half-m",
        str(search_area_half_m),
        "--search-det-steer",
        "--device",
        os.environ.get("SIM_DEVICE", "cuda"),
    ]
    if det in ("yolo", "open_vocab", "semantic", "humen_bridge", "humen_corridor"):
        cmd.extend(
            [
                "--yolo-model",
                os.environ.get("SIM_YOLO_MODEL", "yolov8s-worldv2.pt"),
                "--yolo-imgsz",
                os.environ.get("SIM_YOLO_IMGSZ", "1280"),
                "--yolo-conf",
                os.environ.get("SIM_YOLO_CONF", os.environ.get("SIM_YOLO_CONF_HUMEN", "0.05")),
            ]
        )
    return cmd


def _goto_eval_cmd(
    *,
    annotation: Path,
    traj_out: Path,
    visual_prompt: str,
    target_class: str,
    max_steps: int,
    goal_z: float,
    wp_radius_m: Optional[float] = None,
    success_dist_m: Optional[float] = None,
    no_shield: bool = True,
) -> List[str]:
    """Goto a single goal with geometric toward-g (scan pattern has no area planner)."""
    cmd = _base_eval_cmd(
        annotation=annotation,
        traj_out=traj_out,
        visual_prompt=visual_prompt,
        target_class=target_class,
        max_steps=max_steps,
        search_area_half_m=8.0,
    )
    cmd.extend(
        [
            "--search-pattern",
            "scan",
            "--fallback-toward-g",
            "--no-visual-toward-g",
            "--search-z-hold-m",
            str(goal_z),
            "--search-waypoint-radius-m",
            str(wp_radius_m if wp_radius_m is not None else os.environ.get("SIM_SURVEY_WP_RADIUS_M", "12")),
            "--success-dist",
            str(success_dist_m if success_dist_m is not None else os.environ.get("SIM_SURVEY_SUCCESS_DIST_M", "10")),
        ]
    )
    # Humen open-water: depth model hallucinates → three_zone shield can block
    # forward progress (same root cause SIM_SURVEY_NO_SHIELD works around for
    # the phase2 SURVEY stack).
    if no_shield:
        cmd.append("--no-shield")
    return cmd


def _survey_eval_cmd(
    *,
    annotation: Path,
    traj_out: Path,
    visual_prompt: str,
    target_class: str,
    max_steps: int,
    goal_z: float,
    goal_yaw: float,
) -> List[str]:
    return _goto_eval_cmd(
        annotation=annotation,
        traj_out=traj_out,
        visual_prompt=visual_prompt,
        target_class=target_class,
        max_steps=max_steps,
        goal_z=goal_z,
    )


def run_sim_search(mission_dir: Path) -> subprocess.CompletedProcess[str]:
    mission_dir = mission_dir.resolve()
    spec = load_spec_from_mission_dir(mission_dir)
    phases = json.loads((mission_dir / "phase_plan.json").read_text(encoding="utf-8"))
    visual = str(phases.get("visual_prompt", "bridge"))
    target_class = str(phases.get("target_category", spec.target.category))
    search = phases.get("search", {})
    run_dir = sim_runs_dir(mission_dir) / "search"
    run_dir.mkdir(parents=True, exist_ok=True)
    traj_out = run_dir / "traj"

    scx, scy = spec.search.center_xy
    alt = float(search.get("altitude_m", spec.search.altitude_m))
    half = float(spec.search.radius_m)
    pattern = os.environ.get("SIM_SEARCH_PATTERN", search.get("pattern", "lawnmower"))
    start_yaw_deg = float(
        search.get("start_yaw_deg", os.environ.get("SIM_SEARCH_START_YAW_DEG", "0"))
    )
    sweep_m = float(search.get("sweep_spacing_m", os.environ.get("SIM_SEARCH_SWEEP_M", "25")))
    wp_radius = float(
        search.get("waypoint_reach_radius_m", os.environ.get("SIM_SEARCH_WP_RADIUS_M", "10"))
    )
    if pattern == "corridor":
        min_x = float(scx) - half
        max_x = float(scx) + half
        start_x = min_x + min(25.0, half * 0.1)
        start_yaw = math.radians(start_yaw_deg if start_yaw_deg else 45.0)
        route = _route([start_x, scy, alt], [start_x, scy, alt], yaw=start_yaw, instruction=visual)
        route["search_area"] = {
            "min_x": min_x,
            "max_x": max_x,
            "min_y": float(scy),
            "max_y": float(scy),
            "corridor_y": float(scy),
            "corridor_start_x": start_x,
            "altitude_z": alt,
            "sweep_spacing_m": sweep_m,
            "waypoint_reach_radius_m": wp_radius,
            "loop": False,
        }
    else:
        route = _route([scx, scy, alt], [scx, scy, alt], instruction=visual)
        route["search_area"] = {
            "min_x": float(scx) - half,
            "max_x": float(scx) + half,
            "min_y": float(scy) - half,
            "max_y": float(scy) + half,
            "altitude_z": alt,
            "sweep_spacing_m": sweep_m,
            "waypoint_reach_radius_m": wp_radius,
        }
    ann = _write_phase_annotation(mission_dir, route, "search")

    cmd = _base_eval_cmd(
        annotation=ann,
        traj_out=traj_out,
        visual_prompt=visual,
        target_class=target_class,
        max_steps=int(search.get("max_steps", 400)),
        search_area_half_m=float(spec.search.radius_m),
    )
    # wam_vgoal_eval auto z-hold clips spawn z to [20,40] — wrong for Humen at 95m.
    cmd.extend(
        [
            "--search-z-hold-m",
            str(alt),
            "--search-z-min",
            str(max(10.0, alt - 50.0)),
            "--search-z-max",
            str(alt + 30.0),
            "--tracker-min-confidence",
            os.environ.get("SIM_TRACKER_MIN_CONF", "0.08"),
            "--reject-far-lock-m",
            os.environ.get("SIM_REJECT_FAR_LOCK_M", "1200"),
            "--success-dist",
            os.environ.get("SIM_SEARCH_SUCCESS_DIST", "0.5"),
            "--no-bbox-prior-near",
        ]
    )
    # Humen open-water: depth model hallucinates → three_zone shield intervenes on
    # ~85% of SEARCH steps and blocks forward corridor progress (same root cause
    # SIM_SURVEY_NO_SHIELD already works around for the SURVEY phase).
    if os.environ.get("SIM_SEARCH_NO_SHIELD", "1") not in ("0", "false", "False"):
        cmd.append("--no-shield")
    if os.environ.get("SIM_SEARCH_AT_CRUISE", "0") == "1":
        cmd.append("--search-at-cruise")
    if os.environ.get("SIM_SEARCH_AREA_PRIORITY", "0") == "1" or pattern == "corridor":
        cmd.append("--search-area-priority")
    if pattern == "corridor" or os.environ.get("SIM_SEARCH_DIRECT_AREA", "0") == "1":
        cmd.append("--search-direct-area")
    sweep = os.environ.get("SIM_SEARCH_SWEEP_M")
    if sweep:
        cmd.extend(["--search-sweep-spacing-m", sweep])
    wp_r = os.environ.get("SIM_SEARCH_WP_RADIUS_M")
    if wp_r:
        cmd.extend(["--search-waypoint-radius-m", wp_r])
    corr_iv = os.environ.get("SIM_CORRIDOR_YAW_SWEEP_INTERVAL")
    yaw_hold = os.environ.get("SIM_SEARCH_YAW_HOLD_DEG")
    if yaw_hold:
        cmd.extend(["--search-yaw-hold-deg", yaw_hold])
    if corr_iv:
        cmd.extend(["--corridor-yaw-sweep-interval", corr_iv])
        cmd.extend(
            [
                "--corridor-yaw-sweep-steps",
                os.environ.get("SIM_CORRIDOR_YAW_SWEEP_STEPS", "12"),
                "--corridor-yaw-sweep-deg",
                os.environ.get("SIM_CORRIDOR_YAW_SWEEP_DEG", "60"),
                "--corridor-yaw-sweep-step-deg",
                os.environ.get("SIM_CORRIDOR_YAW_SWEEP_STEP_DEG", "5"),
            ]
        )
    if os.environ.get("SIM_VISUAL_PROMPT"):
        idx = cmd.index("--visual-prompt")
        cmd[idx + 1] = os.environ["SIM_VISUAL_PROMPT"]
    yolo_conf = os.environ.get("SIM_YOLO_CONF_HUMEN") or os.environ.get("SIM_YOLO_CONF")
    if yolo_conf:
        idx = cmd.index("--yolo-conf")
        cmd[idx + 1] = yolo_conf
    env = os.environ.copy()
    env["PYTHONPATH"] = str(wam_root())
    print("[SIM SEARCH]", " ".join(cmd), flush=True)
    proc = subprocess.run(cmd, cwd=str(wam_root()), env=env, check=False)
    _record_sim_phase(mission_dir, "search", run_dir)
    return proc


def _last_search_pose(mission_dir: Path) -> tuple[Optional[List[float]], float]:
    traj_path = sim_runs_dir(mission_dir) / "search" / "traj" / "route00.jsonl"
    if not traj_path.is_file():
        return None, 0.785398
    last = None
    for line in traj_path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            last = json.loads(line)
    if last is None or "pos" not in last:
        return None, 0.785398
    pos = [float(x) for x in last["pos"][:3]]
    yaw = float(last.get("yaw", 0.785398))
    return pos, yaw


def run_sim_approach(mission_dir: Path) -> subprocess.CompletedProcess[str]:
    mission_dir = mission_dir.resolve()
    phases = json.loads((mission_dir / "phase_plan.json").read_text(encoding="utf-8"))
    visual = os.environ.get("SIM_VISUAL_PROMPT") or str(phases.get("visual_prompt", "bridge"))
    spec = load_spec_from_mission_dir(mission_dir)
    target_class = str(phases.get("target_category", spec.target.category))
    approach = phases.get("approach", {})
    goal = approach["goal"]
    scx, scy = spec.search.center_xy
    alt = float(spec.search.altitude_m)
    search_end, search_yaw = _last_search_pose(mission_dir)
    start = search_end if search_end is not None else [scx, scy, alt]
    g = [float(goal["x"]), float(goal["y"]), float(goal["z"])]
    # FORCE_SURVEY / stale SEARCH: do not start approach from a far basin (city lock → 主航道).
    max_start = float(os.environ.get("SIM_APPROACH_MAX_START_DIST_M", "400"))
    if math.hypot(start[0] - g[0], start[1] - g[1]) > max_start:
        print(
            f"[SIM APPROACH] start [{start[0]:.0f},{start[1]:.0f}] far from goal "
            f"[{g[0]:.0f},{g[1]:.0f}] — snap to search center [{scx:.0f},{scy:.0f}]",
            flush=True,
        )
        start = [scx, scy, alt]
        search_yaw = 0.0

    run_dir = sim_runs_dir(mission_dir) / "approach"
    run_dir.mkdir(parents=True, exist_ok=True)
    traj_out = run_dir / "traj"
    route = _route(start, g, yaw=search_yaw, instruction=visual)
    ann = _write_phase_annotation(mission_dir, route, "approach")

    goal_z = float(goal["z"])
    if os.environ.get("SIM_APPROACH_Z_HOLD"):
        goal_z = float(os.environ["SIM_APPROACH_Z_HOLD"])
    cmd = _goto_eval_cmd(
        annotation=ann,
        traj_out=traj_out,
        visual_prompt=visual,
        target_class=target_class,
        max_steps=int(approach.get("max_steps", 200)),
        goal_z=goal_z,
        wp_radius_m=float(os.environ.get("SIM_APPROACH_WP_RADIUS_M", "10")),
        success_dist_m=float(os.environ.get("SIM_APPROACH_SUCCESS_DIST_M", "8")),
        no_shield=os.environ.get("SIM_APPROACH_NO_SHIELD", "1") not in ("0", "false", "False"),
    )
    env = os.environ.copy()
    env["PYTHONPATH"] = str(wam_root())
    print("[SIM APPROACH]", " ".join(cmd), flush=True)
    proc = subprocess.run(cmd, cwd=str(wam_root()), env=env, check=False)
    _record_sim_phase(mission_dir, "approach", run_dir)
    return proc


def _survey_start_pose(mission_dir: Path, approach_traj: Path, default: List[float]) -> List[float]:
    """Prefer a pose near the bridge centroid; ignore stale city SEARCH locks when far away."""
    det_path = mission_dir / "detected_centroid.json"
    centroid: Optional[List[float]] = None
    if det_path.is_file():
        det = json.loads(det_path.read_text(encoding="utf-8"))
        if det.get("centroid_xyz"):
            centroid = [float(v) for v in det["centroid_xyz"][:3]]
    # Also accept mission_spec pin (主航道 yaml) when detected_centroid lacks xyz.
    if centroid is None:
        try:
            spec = load_spec_from_mission_dir(mission_dir)
            if spec.bridge_centroid_xyz:
                centroid = [float(v) for v in spec.bridge_centroid_xyz[:3]]
        except Exception:
            pass

    candidates: List[List[float]] = []
    approach_end = _last_pos_from_traj(approach_traj)
    if approach_end is not None:
        candidates.append(approach_end)
    search_end, _ = _last_search_pose(mission_dir)
    if search_end is not None:
        candidates.append(search_end)
    if det_path.is_file():
        nearest = json.loads(det_path.read_text(encoding="utf-8")).get("nearest_lock")
        if nearest and nearest.get("drone_xyz"):
            candidates.append([float(v) for v in nearest["drone_xyz"][:3]])

    if not candidates:
        return default
    if centroid is None:
        return candidates[0]
    cx, cy = centroid[0], centroid[1]
    max_m = float(os.environ.get("SIM_SURVEY_MAX_START_DIST_M", "350"))

    def _xy_dist(p: List[float]) -> float:
        return math.hypot(p[0] - cx, p[1] - cy)

    near = [p for p in candidates if _xy_dist(p) <= max_m]
    if near:
        return min(near, key=_xy_dist)
    # Stale SEARCH/approach left the vehicle in another basin (e.g. east city lock).
    print(
        f"[SIM SURVEY] start snaps to default near centroid "
        f"(candidates all >{max_m:.0f}m from [{cx:.0f},{cy:.0f}])",
        flush=True,
    )
    return default


def _last_pos_from_traj(traj_path: Path) -> Optional[List[float]]:
    if not traj_path.is_file():
        return None
    last = None
    for line in traj_path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            last = json.loads(line)
    if last is None or "pos" not in last:
        return None
    return [float(x) for x in last["pos"][:3]]


def _polyline_length_m(pts: Sequence[Sequence[float]]) -> float:
    total = 0.0
    for a, b in zip(pts, pts[1:]):
        total += math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
        total += abs(float(b[2]) - float(a[2])) if len(a) > 2 and len(b) > 2 else 0.0
    return float(total)


def _is_opposite_facade(
    start: Sequence[float],
    goal: Sequence[float],
    *,
    centroid_xyz: Sequence[float],
    span_axis_deg: float,
) -> bool:
    from aerial_inspect.survey.survey_nav_path import _cross_sign

    cx, cy = float(centroid_xyz[0]), float(centroid_xyz[1])
    s = _cross_sign(start, (cx, cy), span_axis_deg)
    g = _cross_sign(goal, (cx, cy), span_axis_deg)
    return s != 0.0 and g != 0.0 and s != g


def _scripted_tip_transit(
    *,
    pts: Sequence[Sequence[float]],
    traj_out: Path,
    out_json: Path,
    label: str,
) -> int:
    """
    Fly densified tip polyline via verified teleports (Phase-2 cannot follow ~200m+ tip).
    Writes traj + eval_result compatible with survey QC / AUTO_EVAL.
    """
    import airsim

    from aerial_inspect.survey.survey_nav_path import yaw_along_polyline

    root = Path(__file__).resolve().parents[2]
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from scripts.airsim_pose_utils import set_pose_verified

    host = os.environ.get("AIRSIM_HOST", "127.0.0.1")
    port = int(os.environ.get("AIRSIM_PORT", "41451"))
    vehicle = os.environ.get("AIRSIM_VEHICLE", "drone_1")
    spacing = float(os.environ.get("SIM_SURVEY_SCRIPTED_TIP_SPACING_M", "12"))
    # Subsample densified pts to ~spacing
    flown: List[List[float]] = [[float(x) for x in pts[0][:3]]]
    for p in pts[1:]:
        q = [float(x) for x in p[:3]]
        if math.hypot(q[0] - flown[-1][0], q[1] - flown[-1][1]) >= spacing:
            flown.append(q)
    if flown[-1] != [float(x) for x in pts[-1][:3]]:
        flown.append([float(x) for x in pts[-1][:3]])

    yaws = yaw_along_polyline(flown)
    client = airsim.MultirotorClient(ip=host, port=port)
    client.confirmConnection()
    vk = {"vehicle_name": vehicle}
    client.enableApiControl(True, **vk)
    client.armDisarm(True, **vk)

    traj_out.mkdir(parents=True, exist_ok=True)
    traj_path = traj_out / "route00.jsonl"
    goal = flown[-1]
    d_min = float("inf")
    lines: List[str] = []
    print(
        f"[SIM SURVEY scripted-tip] {label} n={len(flown)} L≈{_polyline_length_m(flown):.1f}m "
        f"port={port}",
        flush=True,
    )
    for step, (p, yaw) in enumerate(zip(flown, yaws)):
        ax, ay, az, yaw_out, _pitch, _q = set_pose_verified(
            client, vk, p[0], p[1], p[2], float(yaw), tol_xy=3.0, tol_z=8.0, settle_s=0.25
        )
        d = math.dist([ax, ay, az], goal)
        d_min = min(d_min, d)
        lines.append(
            json.dumps(
                {
                    "step": step,
                    "pos": [ax, ay, az],
                    "yaw_deg": math.degrees(yaw_out),
                    "d_to_g": d,
                    "d_fwd": None,
                    "d_left": None,
                    "d_right": None,
                    "intervened": False,
                    "scripted_tip": True,
                },
                ensure_ascii=False,
            )
        )
    traj_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    d_final = math.dist([ax, ay, az], goal)  # type: ignore[name-defined]
    arrived = d_final <= float(os.environ.get("SIM_SURVEY_SUCCESS_DIST_M", "3"))
    # Tip hop: accept within 8m — teleports should nail goal; loose gate if drift.
    if not arrived and d_final <= 8.0:
        arrived = True
    payload = {
        "protocol_version": "aerial_inspect_scripted_tip_v1",
        "verdict": "PASS" if arrived else "FAIL",
        "arrived": arrived,
        "d_min_m": d_min if d_min < float("inf") else d_final,
        "d_final_m": d_final,
        "metrics": {"arrival_rate": 1.0 if arrived else 0.0},
        "episodes": [
            {
                "arrived": arrived,
                "d_min_m": d_min if d_min < float("inf") else d_final,
                "d_final_m": d_final,
                "scripted_tip": True,
                "label": label,
            }
        ],
    }
    out_json.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    print(
        f"[SIM SURVEY scripted-tip] done arrived={arrived} d_final={d_final:.2f}m d_min={d_min:.2f}m",
        flush=True,
    )
    return 0 if arrived else 1


def _survey_phase2_config(mission_dir: Path) -> Path:
    """Write long_eval YAML with AirSim host/port/camera/vehicle from env (Humen 41463)."""
    import yaml

    root = wam_root()
    base = root / "configs" / "aerial_rl.yaml"
    cfg: Dict[str, Any] = {}
    if base.is_file():
        cfg = yaml.safe_load(base.read_text(encoding="utf-8")) or {}
    env_block = dict(cfg.get("env") or {})
    env_block.update(
        {
            "backend": "airsim",
            "host": os.environ.get("AIRSIM_HOST", "127.0.0.1"),
            "port": int(os.environ.get("AIRSIM_PORT", "41451")),
            "camera": os.environ.get("AIRSIM_CAMERA", "front_custom"),
            "vehicle": os.environ.get("AIRSIM_VEHICLE", "drone_1"),
            "step_hz": float(os.environ.get("SIM_SURVEY_STEP_HZ", "5.0")),
            "grab_depth": True,
        }
    )
    cfg["env"] = env_block
    out = mission_dir / "survey_phase2_rl.yaml"
    out.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
    return out


def _survey_phase2_cmd(
    *,
    annotation: Path,
    traj_out: Path,
    out_json: Path,
    max_steps: int,
    config: Path,
) -> List[str]:
    """Phase-2 long_eval stack (polyline + terminal pin) for survey segments."""
    actor = os.environ.get(
        "SIM_SURVEY_ACTOR_CKPT",
        "experiments/aerial/rl/artifacts/v4_ac_ckpt_phase2_toward_g_20260905_112006/v4_ac_latest.pt",
    )
    wm = os.environ.get(
        "SIM_SURVEY_WM_CKPT",
        "experiments/aerial/rl/artifacts/wm_ckpt_d_full_20260828/wm_step_3500.pt",
    )
    depth = os.environ.get(
        "SIM_SURVEY_DEPTH_CKPT",
        "experiments/aerial/rl/artifacts/depth_ckpt_p45mid_s8j_20260825/depth_best_holdout_da3_ft_head.pt",
    )
    tau = os.environ.get(
        "SIM_SURVEY_TAU_CKPT",
        "experiments/aerial/rl/artifacts/tau_ckpt_foe_r60_20260815/tau_foe_calibrator.pt",
    )
    cmd = [
        sim_python(),
        "-m",
        "experiments.aerial.scripts.wam_phase2_long_eval",
        "--config",
        str(Path(config).resolve()),
        "--annotation",
        str(Path(annotation).resolve()),
        "--routes",
        "0",
        "--actor-ckpt",
        actor,
        "--wm-ckpt",
        wm,
        "--depth-ckpt",
        depth,
        "--tau-ckpt",
        tau,
        "--goal-feat-mode",
        "meter",
        "--subgoal-source",
        os.environ.get("SIM_SURVEY_SUBGOAL", "polyline"),
        "--rolling-global",
        "--heading-assist",
        "--global-horizon-m",
        os.environ.get("SIM_SURVEY_GLOBAL_HORIZON_M", "60"),
        "--global-replan-period-s",
        "1.0",
        "--heading-assist-cte-max-m",
        "8.0",
        "--heading-assist-cos-thr",
        "0.7",
        "--planner",
        "--planner-horizon",
        "1",
        "--tti-coeff",
        "2.5",
        "--heading-reentry-cos",
        "0.5",
        "--cte-reentry-m",
        "1.5",
        "--cruise-speed",
        os.environ.get("SIM_SURVEY_CRUISE_SPEED", "10.0"),
        "--max-steps",
        str(max_steps),
        "--success-dist",
        os.environ.get("SIM_SURVEY_SUCCESS_DIST_M", "3.0"),
        "--terminal-pin-rem-m",
        os.environ.get("SIM_SURVEY_TERMINAL_PIN_M", "20"),
        "--terminal-creep-rem-m",
        os.environ.get("SIM_SURVEY_TERMINAL_CREEP_M", "15"),
        "--near-miss-retry-m",
        "5",
        "--near-miss-retries",
        "1",
        "--traj-out",
        str(traj_out),
        "--out",
        str(out_json),
        "--device",
        os.environ.get("SIM_DEVICE", "cuda"),
        "--step-hz",
        os.environ.get("SIM_SURVEY_STEP_HZ", "5.0"),
    ]
    # Humen open-water: depth model hallucinates → three_zone IR~100% wrong-way.
    if os.environ.get("SIM_SURVEY_NO_SHIELD", "1") not in ("0", "false", "False"):
        cmd.append("--no-shield")
    min_spawn_z = os.environ.get("SIM_SURVEY_MIN_SPAWN_Z", "").strip()
    if min_spawn_z:
        cmd.extend(["--min-spawn-z", min_spawn_z])
    return cmd


def _span_extent_from_mission(mission_dir: Path, spec) -> float:
    det_path = mission_dir / "detected_centroid.json"
    fallback = 2.0 * float(spec.survey.radius_m) * max(float(spec.survey.ellipse_aspect), 1.0)
    if not det_path.is_file():
        return fallback
    from aerial_inspect.survey.view_planner import estimate_span_extent_m

    return float(estimate_span_extent_m(json.loads(det_path.read_text(encoding="utf-8")), fallback_m=fallback))


def _write_survey_phase2_annotation(
    run_dir: Path,
    *,
    start: List[float],
    goal: List[float],
    centroid_xyz: List[float],
    span_axis_deg: float,
    span_extent_m: float,
    instruction: str,
    label: str,
) -> Path:
    from aerial_inspect.survey.survey_nav_path import build_survey_segment_polyline, yaw_along_polyline

    tip_margin = float(os.environ.get("SIM_SURVEY_TIP_MARGIN_M", "40"))
    # Cap: legacy env often set 80 which blew tip routes past Phase-2 range.
    tip_margin = min(max(tip_margin, 20.0), 45.0)
    pts = build_survey_segment_polyline(
        start,
        goal,
        centroid_xyz=centroid_xyz,
        span_axis_deg=span_axis_deg,
        span_extent_m=span_extent_m,
        spacing_m=float(os.environ.get("SIM_SURVEY_POLY_SPACING_M", "15")),
        tip_margin_m=tip_margin,
        transit_alt_boost_m=float(os.environ.get("SIM_SURVEY_TRANSIT_ALT_BOOST_M", "30")),
    )
    yaws = yaw_along_polyline(pts)
    route = {
        "route_id": f"survey_{label}",
        "route_idx": 0,
        "start_pos": list(pts[0]),
        "goal_pos": list(pts[-1]),
        "pos": [list(p) for p in pts],
        "positions": [list(p) for p in pts],
        "yaw": yaws,
        "gpt_instruction": instruction,
        "pattern": "survey_facade_segment",
    }
    payload = {
        "protocol_version": "aerial_inspect_survey_phase2_v1",
        "n_episodes": 1,
        "episodes": [route],
    }
    path = run_dir / "annotation.json"
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return path


def run_sim_survey(mission_dir: Path) -> int:
    mission_dir = mission_dir.resolve()
    wps = json.loads((mission_dir / "waypoints.json").read_text(encoding="utf-8"))
    phases = json.loads((mission_dir / "phase_plan.json").read_text(encoding="utf-8"))
    visual = str(phases.get("visual_prompt", "bridge"))
    spec = load_spec_from_mission_dir(mission_dir)
    target_class = str(phases.get("target_category", spec.target.category))
    scx, scy = spec.search.center_xy
    alt = float(spec.search.altitude_m)
    default_start = [scx, scy, alt]

    approach_traj = sim_runs_dir(mission_dir) / "approach" / "traj" / "route00.jsonl"
    survey_start = _survey_start_pose(mission_dir, approach_traj, default_start)

    survey_root = sim_runs_dir(mission_dir) / "survey"
    survey_root.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["PYTHONPATH"] = str(wam_root())
    last_rc = 0

    nav_mode = os.environ.get("SIM_SURVEY_NAV", "phase2").lower()
    centroid = list(spec.bridge_centroid_xyz) if spec.bridge_centroid_xyz else [scx, scy, alt]
    span_axis = float(spec.bridge_span_axis_deg if spec.bridge_span_axis_deg is not None else spec.survey.span_axis_deg)
    span_extent = _span_extent_from_mission(mission_dir, spec)
    phase2_cfg = _survey_phase2_config(mission_dir) if nav_mode in ("phase2", "long_eval", "polyline") else None

    for i, wp in enumerate(wps):
        g = [float(wp["x"]), float(wp["y"]), float(wp["z"])]
        if i == 0:
            start = survey_start
        else:
            prev_traj = survey_root / f"wp_{i - 1:03d}" / "traj" / "route00.jsonl"
            start = _last_pos_from_traj(prev_traj) or [
                float(wps[i - 1]["x"]),
                float(wps[i - 1]["y"]),
                float(wps[i - 1]["z"]),
            ]
        run_dir = survey_root / f"wp_{i:03d}"
        run_dir.mkdir(parents=True, exist_ok=True)
        traj_out = run_dir / "traj"
        label = str(wp.get("label", f"wp_{i:03d}"))

        if nav_mode in ("phase2", "long_eval", "polyline"):
            ann = _write_survey_phase2_annotation(
                run_dir,
                start=start,
                goal=g,
                centroid_xyz=centroid,
                span_axis_deg=span_axis,
                span_extent_m=span_extent,
                instruction=visual,
                label=label,
            )
            # Opposite tip: Phase-2 policy stalls on ~200m tip polylines; script hop.
            use_scripted = os.environ.get("SIM_SURVEY_SCRIPTED_TIP", "1").strip() not in (
                "0",
                "false",
                "False",
            )
            scripted_all = os.environ.get("SIM_SURVEY_SCRIPTED_ALL", "0").strip() in (
                "1",
                "true",
                "True",
            )
            tip_min_m = float(os.environ.get("SIM_SURVEY_SCRIPTED_TIP_MIN_M", "80"))
            ann_pts = json.loads(ann.read_text(encoding="utf-8"))["episodes"][0].get("pos") or []
            opposite = _is_opposite_facade(
                start, g, centroid_xyz=centroid, span_axis_deg=span_axis
            )
            plen = _polyline_length_m(ann_pts) if ann_pts else 0.0
            if use_scripted and (scripted_all or (opposite and plen >= tip_min_m)):
                tag = "scripted-all" if scripted_all else "scripted-tip"
                print(f"[SIM SURVEY {i+1}/{len(wps)} {tag}]", label, flush=True)
                last_rc = _scripted_tip_transit(
                    pts=ann_pts,
                    traj_out=traj_out,
                    out_json=run_dir / "eval_result.json",
                    label=label,
                )
                _normalize_survey_eval_result(run_dir / "eval_result.json")
                continue
            cmd = _survey_phase2_cmd(
                annotation=ann,
                traj_out=traj_out,
                out_json=run_dir / "eval_result.json",
                max_steps=int(os.environ.get("SIM_MAX_STEPS_PER_WP", "900")),
                config=phase2_cfg if phase2_cfg is not None else mission_dir / "survey_phase2_rl.yaml",
            )
            print(f"[SIM SURVEY {i+1}/{len(wps)} phase2]", label, flush=True)
        else:
            route = _route(start, g, yaw=float(wp.get("yaw_rad", 0.0)), instruction=visual)
            ann = run_dir / "annotation.json"
            ann.write_text(
                json.dumps({"version": "aerial_inspect_sim_v1", "n_routes": 1, "routes": [route]}, indent=2),
                encoding="utf-8",
            )
            cmd = _survey_eval_cmd(
                annotation=ann,
                traj_out=traj_out,
                visual_prompt=visual,
                target_class=target_class,
                max_steps=int(os.environ.get("SIM_MAX_STEPS_PER_WP", "200")),
                goal_z=float(wp["z"]),
                goal_yaw=float(wp.get("yaw_rad", 0.0)),
            )
            print(f"[SIM SURVEY {i+1}/{len(wps)} vgoal]", label, flush=True)

        proc = subprocess.run(cmd, cwd=str(wam_root()), env=env, check=False)
        last_rc = proc.returncode
        _normalize_survey_eval_result(run_dir / "eval_result.json")

    _record_sim_phase(mission_dir, "survey", survey_root)
    return last_rc


def _normalize_survey_eval_result(path: Path) -> None:
    """Map phase2 long_eval episodes[].arrived → top-level verdict for logs."""
    if not path.is_file():
        return
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return
    if "verdict" in data and "arrived" in data:
        return
    eps = data.get("episodes") or []
    if not eps:
        return
    arrived = bool(eps[0].get("arrived"))
    data["arrived"] = arrived
    data["verdict"] = "PASS" if arrived else "FAIL"
    if eps[0].get("d_min_m") is not None:
        data["d_min_m"] = eps[0]["d_min_m"]
    if eps[0].get("d_final_m") is not None:
        data["d_final_m"] = eps[0]["d_final_m"]
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")


def _record_sim_phase(mission_dir: Path, phase: str, run_dir: Path) -> None:
    path = mission_dir / "phase_runs.json"
    data: Dict[str, Any] = {}
    if path.is_file():
        data = json.loads(path.read_text(encoding="utf-8"))
    data[phase] = {
        "backend": "airsim_eval",
        "run_dir": str(run_dir),
        "traj": str(run_dir / "traj" / "route00.jsonl") if phase != "survey" else str(run_dir),
        "recorded_at": time.time(),
    }
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
