"""Thin adapter to aerial-wam-v2 deploy stack (subprocess, no torch import)."""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional


def wam_root() -> Path:
    return Path(os.environ.get("AERIAL_WAM_ROOT", Path.home() / "Projects" / "aerial-wam-v2")).resolve()


def vgoal_root() -> Path:
    return Path(os.environ.get("AERIAL_VGOAL_ROOT", Path.home() / "Projects" / "aerial-vgoal-wam")).resolve()


def _mission_spec_dict(mission_dir: Path) -> Dict[str, Any]:
    spec_path = mission_dir / "mission_spec.json"
    if not spec_path.is_file():
        return {}
    return json.loads(spec_path.read_text(encoding="utf-8"))


def export_wam_phases(mission_dir: Path) -> Path:
    """Write wam_phases.json for SEARCH / APPROACH deploy scripts."""
    phases_in = mission_dir / "phase_plan.json"
    if not phases_in.is_file():
        raise FileNotFoundError(f"missing {phases_in}; run plan first")
    phases = json.loads(phases_in.read_text(encoding="utf-8"))
    out = mission_dir / "wam_phases.json"
    payload = {
        "wam_root": str(wam_root()),
        "vgoal_repo": str(vgoal_root()),
        "deploy_module": "experiments.aerial.scripts.wam_vgoal_deploy",
        "visual_prompt": phases.get("visual_prompt", "bridge"),
        "search": phases.get("search", {}),
        "approach": phases.get("approach", {}),
    }
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return out


def export_wam_waypoints(mission_dir: Path) -> Path:
    """Write wam_waypoints.json for sequential deploy."""
    wp_in = mission_dir / "waypoints.json"
    if not wp_in.is_file():
        raise FileNotFoundError(f"missing {wp_in}; run plan first")
    waypoints = json.loads(wp_in.read_text(encoding="utf-8"))
    out = mission_dir / "wam_waypoints.json"
    payload = {
        "wam_root": str(wam_root()),
        "vgoal_repo": str(vgoal_root()),
        "deploy_module": "experiments.aerial.scripts.wam_vgoal_deploy",
        "defaults": {
            "step_hz": 5.0,
            "max_steps_per_waypoint": 150,
            "visual_prompt": None,
            "record_auto": True,
        },
        "waypoints": waypoints,
    }
    spec = _mission_spec_dict(mission_dir)
    if spec:
        payload["defaults"]["visual_prompt"] = (spec.get("target") or {}).get("visual_prompt")
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return out


def _base_deploy_cmd(
    *,
    mavlink_port: str,
    visual_prompt: Optional[str],
    max_steps: int,
    record_auto: bool,
) -> List[str]:
    cmd = [
        sys.executable,
        "-m",
        "experiments.aerial.scripts.wam_vgoal_deploy",
        "--vgoal-repo",
        str(vgoal_root()),
        "--mavlink-port",
        mavlink_port,
        "--max-steps",
        str(max_steps),
    ]
    if visual_prompt:
        cmd.extend(["--visual-prompt", visual_prompt])
    if record_auto:
        cmd.append("--record-auto")
    return cmd


def _finalize_deploy_cmd(cmd: List[str], *, mock_camera: bool, flight: bool) -> List[str]:
    if mock_camera:
        cmd.append("--mock-camera")
    if flight:
        cmd.extend(["--offboard", "--run", "--i-know-props-are-on"])
    return cmd


def build_search_cmd(
    phase: Dict[str, Any],
    *,
    visual_prompt: str,
    mavlink_port: str = "/dev/ttyACM0",
    mock_camera: bool = False,
    flight: bool = False,
) -> List[str]:
    """Visual search: no fixed goal; vgoal detector drives exploration."""
    cmd = _base_deploy_cmd(
        mavlink_port=mavlink_port,
        visual_prompt=visual_prompt,
        max_steps=int(phase.get("max_steps", 400)),
        record_auto=bool(phase.get("record_auto", True)),
    )
    center = phase.get("center_xy") or [0.0, 0.0]
    alt = float(phase.get("altitude_m", 40.0))
    cmd.extend([
        "--goal-x", str(center[0]),
        "--goal-y", str(center[1]),
        "--goal-z", str(alt),
        "--corpus-leg", "search",
    ])
    return _finalize_deploy_cmd(cmd, mock_camera=mock_camera, flight=flight)


def build_approach_cmd(
    phase: Dict[str, Any],
    *,
    visual_prompt: str,
    mavlink_port: str = "/dev/ttyACM0",
    mock_camera: bool = False,
    flight: bool = False,
) -> List[str]:
    """Approach standoff waypoint with visual prompt for detection lock."""
    goal = phase.get("goal") or {}
    cmd = _base_deploy_cmd(
        mavlink_port=mavlink_port,
        visual_prompt=visual_prompt,
        max_steps=int(phase.get("max_steps", 200)),
        record_auto=bool(phase.get("record_auto", True)),
    )
    cmd.extend([
        "--goal-x", str(goal["x"]),
        "--goal-y", str(goal["y"]),
        "--goal-z", str(goal["z"]),
        "--corpus-leg", "approach",
    ])
    return _finalize_deploy_cmd(cmd, mock_camera=mock_camera, flight=flight)


def build_deploy_cmd(
    wp: Dict[str, Any],
    *,
    mavlink_port: str = "/dev/ttyACM0",
    mock_camera: bool = False,
    dry_run: bool = False,
    visual_prompt: Optional[str] = None,
) -> List[str]:
    cmd = _base_deploy_cmd(
        mavlink_port=mavlink_port,
        visual_prompt=visual_prompt,
        max_steps=150,
        record_auto=True,
    )
    cmd.extend([
        "--goal-x", str(wp["x"]),
        "--goal-y", str(wp["y"]),
        "--goal-z", str(wp["z"]),
        "--corpus-leg", "survey",
    ])
    return _finalize_deploy_cmd(cmd, mock_camera=mock_camera, flight=not dry_run)


def run_waypoint(
    wp: Dict[str, Any],
    *,
    mavlink_port: str = "/dev/ttyACM0",
    mock_camera: bool = True,
    dry_run: bool = True,
) -> subprocess.CompletedProcess[str]:
    cmd = build_deploy_cmd(wp, mavlink_port=mavlink_port, mock_camera=mock_camera, dry_run=dry_run)
    root = wam_root()
    env = os.environ.copy()
    env["PYTHONPATH"] = str(root)
    if dry_run:
        print("[dry-run]", " ".join(cmd))
        return subprocess.CompletedProcess(cmd, 0, "", "")
    return subprocess.run(cmd, cwd=str(root), env=env, capture_output=True, text=True, check=False)


def load_orin_recorder_class():
    """Import OrinDeployRecorder from WAM when running in same venv as deploy."""
    root = wam_root()
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from experiments.aerial.deploy.orin_deploy_recorder import OrinDeployRecorder  # noqa: WPS433

    return OrinDeployRecorder
