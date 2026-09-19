#!/usr/bin/env python3
"""CLI entry: aerial-inspect plan | export-wam | check-capture."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from aerial_inspect.adapters.wam_platform import export_wam_phases, export_wam_waypoints
from aerial_inspect.adapters.wam_runs import find_latest_run, record_phase_run
from aerial_inspect.adapters.wam_sim import (
    export_sim_annotations,
    run_sim_approach,
    run_sim_search,
    run_sim_survey,
    sim_runs_dir,
)
from aerial_inspect.capture.session import CaptureSessionRef
from aerial_inspect.mission.centroid import (
    estimate_centroid_from_traj,
    estimate_span_axis_from_traj,
    write_detected_centroid,
)
from aerial_inspect.mission.orchestrator import (
    load_spec_from_mission_dir,
    load_mission_yaml,
    plan_mission,
    replan_survey,
)
from aerial_inspect.reconstruct.pipeline import quality_check


def _cmd_plan(args: argparse.Namespace) -> int:
    spec = load_mission_yaml(Path(args.config))
    out = Path(args.output)
    include_survey = bool(args.include_survey)
    summary = plan_mission(spec, out, include_survey=include_survey)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


def _first_traj_jsonl(root: Path) -> Path | None:
    """Resolve wam_vgoal_eval output (traj.jsonl or traj/route00.jsonl)."""
    if root.is_file():
        return root
    direct = root / "traj.jsonl"
    if direct.is_file():
        return direct
    nested = root / "traj" / "route00.jsonl"
    if nested.is_file():
        return nested
    matches = sorted(root.glob("**/route*.jsonl"))
    return matches[0] if matches else None


def _resolve_search_traj(
    mission_dir: Path,
    from_run: str | None,
    from_traj: str | None,
) -> Path:
    if from_traj:
        p = Path(from_traj).resolve()
        if p.is_file():
            return p
        raise FileNotFoundError(f"missing traj file: {p}")

    if from_run:
        run = Path(from_run).resolve()
        if run.is_file():
            return run
        found = _first_traj_jsonl(run)
        if found is not None:
            return found
        raise FileNotFoundError(f"no traj.jsonl under {run}")

    runs_path = mission_dir / "phase_runs.json"
    if runs_path.is_file():
        data = json.loads(runs_path.read_text(encoding="utf-8"))
        search = data.get("search") or {}
        traj = search.get("traj")
        if traj:
            found = _first_traj_jsonl(Path(traj).resolve())
            if found is not None:
                return found
        run_dir = search.get("run_dir")
        if run_dir:
            found = _first_traj_jsonl(Path(run_dir).resolve())
            if found is not None:
                return found

    sim_root = sim_runs_dir(mission_dir) / "search"
    found = _first_traj_jsonl(sim_root)
    if found is not None:
        return found

    run = find_latest_run(leg="search")
    if run is not None:
        found = _first_traj_jsonl(run)
        if found is not None:
            return found

    raise FileNotFoundError(
        "no SEARCH traj; pass --from-traj / --from-run, or run search (sim or deploy) first"
    )


def _cmd_replan_survey(args: argparse.Namespace) -> int:
    mission_dir = Path(args.mission_dir)

    span_axis_deg: float | None = None
    if args.span_axis_deg is not None:
        span_axis_deg = float(args.span_axis_deg)

    if args.centroid:
        centroid = tuple(float(x) for x in args.centroid)
        source = "manual"
        write_detected_centroid(
            mission_dir,
            {"status": "ok", "centroid_xyz": list(centroid), "source": source},
        )
    else:
        det_path = mission_dir / "detected_centroid.json"
        use_existing = False
        force_recompute = bool(args.from_traj or args.from_run or args.require_det_hit)
        if det_path.is_file() and not force_recompute:
            existing = json.loads(det_path.read_text(encoding="utf-8"))
            if existing.get("status") == "ok" and existing.get("source") in (
                "humen_geometric_scan",
                "humen_visual_probe",
                "detected",
            ):
                centroid = tuple(existing["centroid_xyz"])
                source = str(existing.get("source", "detected"))
                use_existing = True

        if not use_existing:
            traj = _resolve_search_traj(mission_dir, args.from_run, args.from_traj)
            detection = estimate_centroid_from_traj(
                traj,
                extrapolate_standoff_m=float(args.extrapolate_standoff)
                if args.extrapolate_standoff is not None
                else None,
                min_samples=int(args.min_samples),
                require_det_hit=bool(args.require_det_hit),
                min_goal_rel_dist_m=float(args.min_goal_rel_dist_m),
            )
            detection["source_traj"] = str(traj)
            if span_axis_deg is None:
                axis_result = estimate_span_axis_from_traj(traj, min_samples=int(args.min_samples))
                detection["span_axis"] = axis_result
                if axis_result["status"] == "ok":
                    span_axis_deg = float(axis_result["span_axis_deg"])
            detection["source"] = "detected"
            write_detected_centroid(mission_dir, detection)
            if detection["status"] != "ok":
                print(json.dumps(detection, indent=2, ensure_ascii=False))
                return 1
            centroid = tuple(detection["centroid_xyz"])
            source = "detected"

    mission_yaml = Path(args.config) if getattr(args, "config", None) else None
    summary = replan_survey(
        mission_dir,
        centroid,
        source=source,
        span_axis_deg=span_axis_deg,
        mission_yaml=mission_yaml,
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))

    if args.export_wam:
        export_wam_phases(mission_dir)
        wp = export_wam_waypoints(mission_dir)
        print(f"wrote {wp}")
    return 0


def _cmd_export_sim(args: argparse.Namespace) -> int:
    out = export_sim_annotations(Path(args.mission_dir))
    print(f"wrote {out}")
    return 0


def _cmd_sim_search(args: argparse.Namespace) -> int:
    return int(run_sim_search(Path(args.mission_dir)).returncode)


def _cmd_sim_approach(args: argparse.Namespace) -> int:
    return int(run_sim_approach(Path(args.mission_dir)).returncode)


def _cmd_sim_survey(args: argparse.Namespace) -> int:
    return int(run_sim_survey(Path(args.mission_dir)))


def _cmd_sim_pipeline(args: argparse.Namespace) -> int:
    spec = load_mission_yaml(Path(args.config))
    mission_dir = Path(args.output) if args.output else Path(f"artifacts/{spec.mission_id}")
    plan_mission(spec, mission_dir, include_survey=False)
    export_wam_phases(mission_dir)
    export_sim_annotations(mission_dir)

    rc = _cmd_sim_search(argparse.Namespace(mission_dir=str(mission_dir)))
    if rc != 0 and not args.continue_on_error:
        return rc

    rc = _cmd_replan_survey(
        argparse.Namespace(
            mission_dir=str(mission_dir),
            from_run=None,
            from_traj=None,
            centroid=None,
            min_samples=3,
            extrapolate_standoff=None,
            span_axis_deg=None,
            export_wam=True,
        )
    )
    if rc != 0:
        return rc

    export_sim_annotations(mission_dir)
    rc = _cmd_sim_approach(argparse.Namespace(mission_dir=str(mission_dir)))
    if rc != 0 and not args.continue_on_error:
        return rc

    return _cmd_sim_survey(argparse.Namespace(mission_dir=str(mission_dir)))


def _cmd_record_phase_run(args: argparse.Namespace) -> int:
    mission_dir = Path(args.mission_dir)
    run = find_latest_run(leg=args.phase)
    if run is None:
        print(f"no orin_deploy run found for leg={args.phase}", file=sys.stderr)
        return 1
    data = record_phase_run(mission_dir, args.phase, run)
    print(json.dumps(data[args.phase], indent=2, ensure_ascii=False))
    return 0


def _cmd_export_wam(args: argparse.Namespace) -> int:
    out = export_wam_waypoints(Path(args.mission_dir))
    print(f"wrote {out}")
    return 0


def _cmd_export_wam_phases(args: argparse.Namespace) -> int:
    out = export_wam_phases(Path(args.mission_dir))
    print(f"wrote {out}")
    return 0


def _cmd_check_capture(args: argparse.Namespace) -> int:
    ref = CaptureSessionRef.from_run_dir(Path(args.capture_dir))
    q = quality_check(ref.run_dir)
    print(json.dumps(q, indent=2))
    return 0 if q.get("min_frames_met") else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="aerial-inspect")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_plan = sub.add_parser("plan", help="Pre-search mission plan (phases; survey after detection)")
    p_plan.add_argument("config", help="configs/missions/*.yaml")
    p_plan.add_argument("-o", "--output", required=True, help="artifacts/mission_xxx")
    p_plan.add_argument(
        "--include-survey",
        action="store_true",
        help="Also plan survey waypoints (requires bridge_centroid_xyz in YAML)",
    )
    p_plan.set_defaults(func=_cmd_plan)

    p_rp = sub.add_parser(
        "replan-survey",
        help="Estimate bridge center from SEARCH capture and generate orbit waypoints",
    )
    p_rp.add_argument("mission_dir")
    p_rp.add_argument("--from-run", help="WAM capture run directory (default: phase_runs.json search)")
    p_rp.add_argument("--from-traj", help="Direct path to SEARCH traj.jsonl (AirSim sim)")
    p_rp.add_argument("--centroid", nargs=3, type=float, metavar=("X", "Y", "Z"), help="Manual centroid override")
    p_rp.add_argument("--min-samples", type=int, default=3)
    p_rp.add_argument(
        "--require-det-hit",
        action="store_true",
        help="Only use traj rows where open_vocab/YOLO det_hit=true",
    )
    p_rp.add_argument(
        "--min-goal-rel-dist-m",
        type=float,
        default=0.0,
        help="Reject near-field false locks (goal_rel distance in m)",
    )
    p_rp.add_argument(
        "--extrapolate-standoff",
        type=float,
        default=None,
        help="If set, extrapolate centroid beyond goal_rel by this distance (m)",
    )
    p_rp.add_argument(
        "--span-axis-deg",
        type=float,
        default=None,
        help="Bridge long-axis bearing (deg); default: PCA from SEARCH traj",
    )
    p_rp.add_argument("--export-wam", action="store_true", help="Also write wam_phases.json + wam_waypoints.json")
    p_rp.add_argument(
        "--config",
        help="Mission YAML (refresh survey pattern/target from config; keeps mission_id from mission_dir)",
    )
    p_rp.set_defaults(func=_cmd_replan_survey)

    p_rec = sub.add_parser("record-phase-run", help="Link latest WAM orin_deploy run to mission phase")
    p_rec.add_argument("mission_dir")
    p_rec.add_argument("phase", choices=["search", "approach", "survey"])
    p_rec.set_defaults(func=_cmd_record_phase_run)

    p_sim = sub.add_parser("export-sim", help="Export AirSim wam_vgoal_eval annotations")
    p_sim.add_argument("mission_dir")
    p_sim.set_defaults(func=_cmd_export_sim)

    for name, fn in (
        ("sim-search", _cmd_sim_search),
        ("sim-approach", _cmd_sim_approach),
        ("sim-survey", _cmd_sim_survey),
    ):
        sp = sub.add_parser(name, help=f"Run {name} on AirSim via wam_vgoal_eval")
        sp.add_argument("mission_dir")
        sp.set_defaults(func=fn)

    p_pipe = sub.add_parser("sim-pipeline", help="Full AirSim闭环: search→replan→approach→survey")
    p_pipe.add_argument("config", help="configs/missions/*.yaml")
    p_pipe.add_argument("-o", "--output", help="artifacts dir (default artifacts/<mission_id>)")
    p_pipe.add_argument("--continue-on-error", action="store_true")
    p_pipe.set_defaults(func=_cmd_sim_pipeline)

    p_exp = sub.add_parser("export-wam", help="Export wam_waypoints.json for deploy")
    p_exp.add_argument("mission_dir")
    p_exp.set_defaults(func=_cmd_export_wam)

    p_ph = sub.add_parser("export-wam-phases", help="Export wam_phases.json for SEARCH/APPROACH")
    p_ph.add_argument("mission_dir")
    p_ph.set_defaults(func=_cmd_export_wam_phases)

    p_chk = sub.add_parser("check-capture", help="QC a recorder run directory")
    p_chk.add_argument("capture_dir")
    p_chk.set_defaults(func=_cmd_check_capture)

    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
