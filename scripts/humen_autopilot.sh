#!/usr/bin/env bash
# Humen mainline autopilot: FORCE_SURVEY → evaluate → auto next.
# Does NOT stop until denser dual-facade gate passes (or hard max rounds).
set -euo pipefail
cd ~/Projects/aerial-inspect
# shellcheck source=/dev/null
source scripts/env_sim_84_humen.sh
unset SIM_SURVEY_MIN_SPAWN_Z || true
export SIM_SURVEY_NO_SHIELD=1
export SIM_SURVEY_SCRIPTED_TIP="${SIM_SURVEY_SCRIPTED_TIP:-1}"
PY=/data/linux/workspace/venvs/aerial-wam/bin/python
MD=artifacts/bridge_humen_001
LOG=$MD/humen_autopilot.log
EVAL=$MD/AUTO_EVAL.json
ROUND_FILE=$MD/autopilot_round.txt
MISSION_YAML=configs/missions/bridge_humen.yaml
# Gate: denser reconstruction + reliable nav (not just overnight SR).
GATE_MIN_SR="${GATE_MIN_SR:-0.70}"
GATE_MIN_PTS="${GATE_MIN_PTS:-400}"
GATE_MIN_N="${GATE_MIN_N:-16}"
GATE_MIN_Y_SPAN_M="${GATE_MIN_Y_SPAN_M:-40}"   # need both facades in sparse cloud
MAX_ROUNDS="${HUMEN_AUTOPILOT_MAX_ROUNDS:-40}"
mkdir -p "$MD"
round=$(cat "$ROUND_FILE" 2>/dev/null || echo 0)
log() { echo "[autopilot $(date +%H:%M:%S)] $*" | tee -a "$LOG"; }

(
  while true; do
    pid=$(pgrep -f "wam_phase2_long_eval.*survey_phase2_rl" | head -1 || true)
    if [[ -n "${pid:-}" ]]; then
      et=$(ps -o etimes= -p "$pid" 2>/dev/null | tr -d " " || echo 0)
      traj=$(tr "\0" " " < /proc/"$pid"/cmdline 2>/dev/null | grep -o '\-\-traj-out [^ ]*' | awk '{print $2}' || true)
      nlines=0
      if [[ -n "$traj" && -f "$traj/route00.jsonl" ]]; then
        nlines=$(wc -l < "$traj/route00.jsonl")
      fi
      if [[ -n "$et" && "$et" -gt 600 && "$nlines" -lt 5 ]]; then
        log "KILL hung long_eval pid=$pid et=$et traj_lines=$nlines"
        kill -9 "$pid" 2>/dev/null || true
      fi
    fi
    sleep 30
  done
) &
HANG_WD=$!

apply_densify() {
  # Escalate survey density in mission yaml (in-place on 84).
  local overlap="$1"
  local laps="$2"
  log "densify overlap=$overlap num_laps=$laps"
  OVERLAP="$overlap" LAPS="$laps" YAML="$MISSION_YAML" $PY - <<'PY'
import os, re
from pathlib import Path
p = Path(os.environ["YAML"])
t = p.read_text(encoding="utf-8")
ov = os.environ["OVERLAP"]
laps = os.environ["LAPS"]
t2, n1 = re.subn(r"(?m)^(  heading_overlap:\s*)([0-9.]+)", rf"\g<1>{ov}", t, count=1)
t2, n2 = re.subn(r"(?m)^(  num_laps:\s*)([0-9]+)", rf"\g<1>{laps}", t2, count=1)
if n1 < 1 or n2 < 1:
    raise SystemExit(f"yaml densify patch failed n1={n1} n2={n2}")
p.write_text(t2, encoding="utf-8")
print(f"patched {p} overlap={ov} laps={laps}")
PY
}

run_pipeline() {
  local tag="$1"
  log "START tag=$tag round=$round NO_SHIELD=$SIM_SURVEY_NO_SHIELD CRUISE=${SIM_SURVEY_CRUISE_SPEED:-10} gate=sr>=${GATE_MIN_SR}/pts>=${GATE_MIN_PTS}/n>=${GATE_MIN_N}"
  pkill -9 -f "aerial_inspect.cli" 2>/dev/null || true
  pkill -9 -f "wam_phase2_long_eval" 2>/dev/null || true
  pkill -9 -f "wam_vgoal_eval" 2>/dev/null || true
  sleep 2
  rm -rf "$MD/sim_runs/survey"
  rm -f "$MD/v13_ready.flag" "$MD/AUTOPILOT_DONE.flag"
  FORCE_SURVEY=1 bash scripts/run_humen_sim_pipeline.sh configs/missions/bridge_humen.yaml \
    > "$MD/humen_pipeline_${tag}.log" 2>&1 || true
  log "pipeline exited tag=$tag"
}

post_process() {
  local tag="$1"
  log "post_process $tag"
  $PY scripts/validate_survey_complete.py "$MD" || true
  $PY scripts/validate_survey_poses.py "$MD" --max-xy-error-m 15 --max-z-error-m 12 || true
  $PY scripts/export_capture_from_sim_survey.py "$MD" || true
  if [[ ! -f "$MD/capture_survey/traj.jsonl" ]]; then
    log "WARN missing capture_survey/traj.jsonl after export"
  else
    log "capture traj lines=$(wc -l < "$MD/capture_survey/traj.jsonl")"
  fi
  $PY scripts/qc_capture_bridge.py "$MD/capture_survey" --min-fraction 0.30 || true
  $PY scripts/stitch_capture_video.py "$MD" || true
  if [[ "${SKIP_HEAVY_VIDEO:-0}" == "1" ]]; then
    log "SKIP_HEAVY_VIDEO=1 — skip orbit flythrough export"
  else
    $PY scripts/export_sim_survey_video.py "$MD" --out "$MD/capture_survey/survey_path_flythrough_${tag}.mp4" || true
  fi
  # denser hybrid defaults for known-pose triangulation
  # Humen span is ~km-scale; 250m centroid filter nukes most inliers (1360→120).
  $PY scripts/run_pycolmap_reconstruct.py "$MD/capture_survey" \
    --workspace artifacts/models/bridge_humen_001 --mode hybrid \
    --sequential-overlap 14 --lap-stride 8 \
    --max-reproj-px 8.0 --min-tri-angle-deg 0.15 \
    --max-dist-from-centroid-m 1200 || true
  pts=$($PY -c "import json;from pathlib import Path;p=Path('artifacts/models/bridge_humen_001/reconstruct_report.json');print(json.load(open(p)).get('num_points3D',0) if p.is_file() else 0)")
  log "COLMAP hybrid pts=$pts"
  if [[ "${pts:-0}" -lt 50 ]]; then
    log "COLMAP hybrid pts=$pts — retry blind_sfm"
    $PY scripts/run_pycolmap_reconstruct.py "$MD/capture_survey" \
      --workspace artifacts/models/bridge_humen_001 --mode blind_sfm || true
  fi
}

evaluate() {
  local tag="$1"
  TAG="$tag" GATE_MIN_SR="$GATE_MIN_SR" GATE_MIN_PTS="$GATE_MIN_PTS" \
  GATE_MIN_N="$GATE_MIN_N" GATE_MIN_Y_SPAN_M="$GATE_MIN_Y_SPAN_M" $PY - <<'PY'
import json, glob, os
from pathlib import Path
md = Path("artifacts/bridge_humen_001")
tag = os.environ.get("TAG", "unknown")
min_sr = float(os.environ.get("GATE_MIN_SR", "0.70"))
min_pts = int(os.environ.get("GATE_MIN_PTS", "400"))
min_n = int(os.environ.get("GATE_MIN_N", "16"))
min_y_span = float(os.environ.get("GATE_MIN_Y_SPAN_M", "40"))
evals = sorted(glob.glob(str(md / "sim_runs/survey/wp_*/eval_result.json")))
rows = []
arrived = 0
wrong_way = 0
for p in evals:
    d = json.load(open(p))
    arr = bool(d.get("arrived"))
    arrived += int(arr)
    dmin, dfin = d.get("d_min_m"), d.get("d_final_m")
    ww = bool(dmin is not None and dfin is not None and float(dfin) > float(dmin) + 20)
    wrong_way += int(ww)
    rows.append(
        {
            "wp": Path(p).parent.name,
            "verdict": d.get("verdict"),
            "arrived": arr,
            "d_min_m": dmin,
            "d_final_m": dfin,
            "wrong_way": ww,
        }
    )
qc = {}
qcp = md / "capture_survey" / "qc_report.json"
if qcp.is_file():
    qc = json.load(open(qcp))
recon = {}
rp = Path("artifacts/models/bridge_humen_001/reconstruct_report.json")
if rp.is_file():
    recon = json.load(open(rp))
n = len(evals)
sr = arrived / n if n else 0.0
pts = int(
    recon.get("num_points3D")
    or recon.get("num_points")
    or recon.get("n_points")
    or recon.get("points")
    or 0
)
# Far/near facade coverage from sparse PLY y extent (if present).
y_span = 0.0
ply = Path("artifacts/models/bridge_humen_001/sparse_points.ply")
if ply.is_file():
    ys = []
    for line in ply.read_text(encoding="utf-8", errors="ignore").splitlines():
        parts = line.strip().split()
        if len(parts) >= 3:
            try:
                ys.append(float(parts[1]))
            except ValueError:
                continue
    if ys:
        y_span = max(ys) - min(ys)

decision, reason = "done_pull", "gate_ok"
if n < min_n:
    decision, reason = "retry_densify", f"too_few_traj n={n}<{min_n}"
elif sr < min_sr:
    decision = "retry_cruise6" if wrong_way >= 3 else "retry_densify"
    reason = f"low_sr={sr:.2f}<{min_sr} ww={wrong_way}"
elif pts < min_pts:
    decision, reason = "retry_densify", f"colmap_pts={pts}<{min_pts}"
elif y_span > 0 and y_span < min_y_span:
    decision, reason = "retry_densify", f"y_span={y_span:.1f}<{min_y_span}"

out = {
    "tag": tag,
    "n_eval": n,
    "arrived": arrived,
    "sr": round(sr, 3),
    "wrong_way": wrong_way,
    "qc": qc,
    "recon_points": pts,
    "y_span_m": round(y_span, 2),
    "gate": {
        "min_sr": min_sr,
        "min_pts": min_pts,
        "min_n": min_n,
        "min_y_span_m": min_y_span,
    },
    "decision": decision,
    "reason": reason,
    "rows": rows,
}
(md / "AUTO_EVAL.json").write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding="utf-8")
print(json.dumps({"decision": decision, "reason": reason, "sr": sr, "n": n, "pts": pts, "y_span": y_span}, indent=2))
PY
}

escalate_for_round() {
  # Progressive densify on 主航道 only — do not rewrite centroid/span.
  case "$round" in
    0) apply_densify 0.70 2 ;;
    1|2) apply_densify 0.80 2 ;;
    3|4) apply_densify 0.85 3; export SIM_SURVEY_CRUISE_SPEED=6 ;;
    *) apply_densify 0.90 3; export SIM_SURVEY_CRUISE_SPEED=6 ;;
  esac
}

auto_next() {
  decision=$($PY -c "import json; print(json.load(open('$EVAL'))['decision'])")
  reason=$($PY -c "import json; print(json.load(open('$EVAL'))['reason'])")
  pts=$($PY -c "import json; print(json.load(open('$EVAL')).get('recon_points',0))")
  log "EVAL decision=$decision reason=$reason round=$round pts=$pts"
  round=$((round + 1))
  echo "$round" > "$ROUND_FILE"
  if [[ "$round" -gt "$MAX_ROUNDS" ]]; then
    log "HIT MAX_ROUNDS=$MAX_ROUNDS — still failing gate; keep flag for pull of best"
    echo DONE_MAX_ROUNDS > "$MD/AUTOPILOT_DONE.flag"
    echo READY_FOR_PULL > "$MD/v13_ready.flag"
    return 0
  fi
  case "$decision" in
    done_pull)
      echo READY_FOR_PULL > "$MD/v13_ready.flag"
      echo DONE_OK > "$MD/AUTOPILOT_DONE.flag"
      log "DONE_OK — gate passed"
      ;;
    retry_cruise6)
      export SIM_SURVEY_CRUISE_SPEED=6
      escalate_for_round
      run_pipeline "v15_c6_r${round}"
      post_process "v15_c6_r${round}"
      evaluate "v15_c6_r${round}"
      auto_next
      ;;
    *)
      escalate_for_round
      run_pipeline "v15_r${round}"
      post_process "v15_r${round}"
      evaluate "v15_r${round}"
      auto_next
      ;;
  esac
}

# If SKIP_FIRST=1, only evaluate existing survey then auto_next (for handoff after live fly).
if [[ "${SKIP_FIRST:-0}" == "1" ]]; then
  log "SKIP_FIRST=1 — post_process+evaluate existing then auto_next"
  tag="${HANDOFF_TAG:-v15}"
  post_process "$tag"
  evaluate "$tag"
  auto_next
else
  escalate_for_round
  tag="${START_TAG:-v15_r${round}}"
  run_pipeline "$tag"
  post_process "$tag"
  evaluate "$tag"
  auto_next
fi
kill "$HANG_WD" 2>/dev/null || true
log "autopilot session end"
