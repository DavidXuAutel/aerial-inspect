"""Locate WAM OrinDeployRecorder runs and link them to mission phases."""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Dict, Optional

from aerial_inspect.adapters.wam_platform import wam_root


def orin_deploy_base() -> Path:
    return wam_root() / "artifacts" / "orin_deploy"


def _run_leg(run_dir: Path) -> Optional[str]:
    manifest = run_dir / "manifest.json"
    if not manifest.is_file():
        return None
    data = json.loads(manifest.read_text(encoding="utf-8"))
    corpus = data.get("corpus") or {}
    return str(corpus.get("leg") or data.get("leg") or "")


def find_latest_run(
    *,
    leg: Optional[str] = None,
    since_monotonic: Optional[float] = None,
    base: Optional[Path] = None,
) -> Optional[Path]:
    root = base or orin_deploy_base()
    if not root.is_dir():
        return None
    candidates = sorted(root.glob("run_*"), key=lambda p: p.stat().st_mtime, reverse=True)
    for run in candidates:
        if since_monotonic is not None and run.stat().st_mtime < since_monotonic:
            continue
        if leg is not None and _run_leg(run) != leg:
            continue
        if (run / "traj.jsonl").is_file():
            return run
    return None


def record_phase_run(mission_dir: Path, phase: str, run_dir: Path) -> Dict[str, Any]:
    mission_dir.mkdir(parents=True, exist_ok=True)
    path = mission_dir / "phase_runs.json"
    data: Dict[str, Any] = {}
    if path.is_file():
        data = json.loads(path.read_text(encoding="utf-8"))
    data[phase] = {
        "run_dir": str(run_dir),
        "traj": str(run_dir / "traj.jsonl"),
        "recorded_at": time.time(),
    }
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    return data
