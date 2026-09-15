#!/usr/bin/env python3
"""Offline COLMAP reconstruction from a WAM capture run directory."""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aerial_inspect.reconstruct.pipeline import prepare_colmap_workspace, quality_check, run_colmap_sfm


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: run_offline_reconstruct.py <capture_run_dir>")
        return 1
    capture = Path(sys.argv[1]).resolve()
    qc = quality_check(capture)
    print("capture QC:", json.dumps(qc, indent=2))
    workspace = ROOT / "artifacts" / "models" / capture.name
    prepare_colmap_workspace(capture, workspace)
    report = run_colmap_sfm(workspace / "images", workspace)
    print(json.dumps(report, indent=2))
    return 0 if report.get("status") in ("ok", "skipped") else 1


if __name__ == "__main__":
    raise SystemExit(main())
