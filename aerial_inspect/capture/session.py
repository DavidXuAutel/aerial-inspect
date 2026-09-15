"""Capture session metadata (frames live in WAM OrinDeployRecorder)."""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional


@dataclass
class CaptureSessionRef:
    """Pointer to a WAM recorder run directory."""

    run_dir: Path
    mission_id: str
    manifest: Dict[str, Any]

    @classmethod
    def from_run_dir(cls, run_dir: Path) -> "CaptureSessionRef":
        manifest_path = run_dir / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.is_file() else {}
        mid = str(manifest.get("mission_id", run_dir.name))
        return cls(run_dir=run_dir, mission_id=mid, manifest=manifest)

    def frame_count(self) -> int:
        frames = self.run_dir / "frames"
        if not frames.is_dir():
            return 0
        return len(list(frames.glob("*.jpg")))

    def traj_path(self) -> Optional[Path]:
        p = self.run_dir / "traj.jsonl"
        return p if p.is_file() else None

    def to_capture_manifest(self, out_path: Path) -> Dict[str, Any]:
        summary = {
            "mission_id": self.mission_id,
            "run_dir": str(self.run_dir),
            "n_frames": self.frame_count(),
            "traj_jsonl": str(self.traj_path()) if self.traj_path() else None,
            "wam_manifest": self.manifest,
        }
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
        return summary
