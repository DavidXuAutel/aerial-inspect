"""AirSim simulation via WAM wam_vgoal_eval (10.229.20.84 / 125 bench)."""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

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
    if det in ("yolo", "open_vocab", "semantic"):
        cmd.extend(
            [
                "--yolo-model",
                os.environ.get("SIM_YOLO_MODEL", "yolov8s-worldv2.pt"),
                "--yolo-imgsz",
                os.environ.get("SIM_YOLO_IMGSZ", "1280"),
                "--yolo-conf",
                os.environ.get("SIM_YOLO_CONF", "0.1"),
            ]
        )
    return cmd


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
    route = _route([scx, scy, alt], [scx, scy, alt], instruction=visual)
    ann = _write_phase_annotation(mission_dir, route, "search")

    cmd = _base_eval_cmd(
        annotation=ann,
        traj_out=traj_out,
        visual_prompt=visual,
        target_class=target_class,
        max_steps=int(search.get("max_steps", 400)),
        search_area_half_m=float(spec.search.radius_m),
    )
    env = os.environ.copy()
    env["PYTHONPATH"] = str(wam_root())
    print("[SIM SEARCH]", " ".join(cmd), flush=True)
    proc = subprocess.run(cmd, cwd=str(wam_root()), env=env, check=False)
    _record_sim_phase(mission_dir, "search", run_dir)
    return proc


def run_sim_approach(mission_dir: Path) -> subprocess.CompletedProcess[str]:
    mission_dir = mission_dir.resolve()
    phases = json.loads((mission_dir / "phase_plan.json").read_text(encoding="utf-8"))
    visual = str(phases.get("visual_prompt", "bridge"))
    spec = load_spec_from_mission_dir(mission_dir)
    target_class = str(phases.get("target_category", spec.target.category))
    approach = phases.get("approach", {})
    goal = approach["goal"]
    scx, scy = spec.search.center_xy
    alt = float(spec.search.altitude_m)
    start = [scx, scy, alt]
    g = [float(goal["x"]), float(goal["y"]), float(goal["z"])]

    run_dir = sim_runs_dir(mission_dir) / "approach"
    run_dir.mkdir(parents=True, exist_ok=True)
    traj_out = run_dir / "traj"
    route = _route(start, g, instruction=visual)
    ann = _write_phase_annotation(mission_dir, route, "approach")

    cmd = _base_eval_cmd(
        annotation=ann,
        traj_out=traj_out,
        visual_prompt=visual,
        target_class=target_class,
        max_steps=int(approach.get("max_steps", 200)),
        search_area_half_m=float(spec.search.radius_m) * 0.25,
    )
    cmd.extend(["--search-pattern", "scan"])
    env = os.environ.copy()
    env["PYTHONPATH"] = str(wam_root())
    print("[SIM APPROACH]", " ".join(cmd), flush=True)
    proc = subprocess.run(cmd, cwd=str(wam_root()), env=env, check=False)
    _record_sim_phase(mission_dir, "approach", run_dir)
    return proc


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

    survey_root = sim_runs_dir(mission_dir) / "survey"
    survey_root.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["PYTHONPATH"] = str(wam_root())
    last_rc = 0

    for i, wp in enumerate(wps):
        g = [float(wp["x"]), float(wp["y"]), float(wp["z"])]
        start = default_start if i == 0 else g
        run_dir = survey_root / f"wp_{i:03d}"
        run_dir.mkdir(parents=True, exist_ok=True)
        traj_out = run_dir / "traj"
        route = _route(start, g, yaw=float(wp.get("yaw_rad", 0.0)), instruction=visual)
        ann = run_dir / "annotation.json"
        ann.write_text(
            json.dumps({"version": "aerial_inspect_sim_v1", "n_routes": 1, "routes": [route]}, indent=2),
            encoding="utf-8",
        )
        cmd = _base_eval_cmd(
            annotation=ann,
            traj_out=traj_out,
            visual_prompt=visual,
            target_class=target_class,
            max_steps=int(os.environ.get("SIM_MAX_STEPS_PER_WP", "150")),
            search_area_half_m=20.0,
        )
        cmd.extend(["--search-pattern", "scan"])
        print(f"[SIM SURVEY {i+1}/{len(wps)}]", wp.get("label", ""), flush=True)
        proc = subprocess.run(cmd, cwd=str(wam_root()), env=env, check=False)
        last_rc = proc.returncode

    _record_sim_phase(mission_dir, "survey", survey_root)
    return last_rc


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
