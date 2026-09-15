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
    spec_path = mission_dir / "mission_spec.json"
    if spec_path.is_file():
        spec = json.loads(spec_path.read_text(encoding="utf-8"))
        payload["defaults"]["visual_prompt"] = (spec.get("target") or {}).get("visual_prompt")
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return out


def build_deploy_cmd(
    wp: Dict[str, Any],
    *,
    mavlink_port: str = "/dev/ttyACM0",
    mock_camera: bool = False,
    dry_run: bool = False,
) -> List[str]:
    root = wam_root()
    cmd = [
        sys.executable,
        "-m",
        "experiments.aerial.scripts.wam_vgoal_deploy",
        "--vgoal-repo",
        str(vgoal_root()),
        "--mavlink-port",
        mavlink_port,
        "--goal-x",
        str(wp["x"]),
        "--goal-y",
        str(wp["y"]),
        "--goal-z",
        str(wp["z"]),
        "--max-steps",
        "150",
        "--record-auto",
    ]
    if mock_camera:
        cmd.append("--mock-camera")
    if dry_run:
        return cmd
    cmd.extend(["--offboard", "--run", "--i-know-props-are-on"])
    env = os.environ.copy()
    env["PYTHONPATH"] = str(root)
    return cmd


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
