#!/usr/bin/env bash
# Outer gate loop: wait for in-flight survey, then keep autopilot until denser gate passes.
set -euo pipefail
cd ~/Projects/aerial-inspect
MD=artifacts/bridge_humen_001
LOG=$MD/humen_gate_until.log
mkdir -p "$MD"
log() { echo "[gate $(date +%H:%M:%S)] $*" | tee -a "$LOG"; }

wait_inflight() {
  log "waiting for in-flight survey/pipeline to finish (no kill)"
  while pgrep -f "run_humen_sim_pipeline|aerial_inspect.cli sim-survey|wam_phase2_long_eval.*survey" >/dev/null 2>&1; do
    n=$(ls "$MD/sim_runs/survey" 2>/dev/null | wc -l | tr -d ' ')
    pass=0
    for f in "$MD"/sim_runs/survey/wp_*/eval_result.json; do
      [[ -f "$f" ]] || continue
      python3 -c "import json,sys;d=json.load(open(sys.argv[1]));sys.exit(0 if d.get('arrived') else 1)" "$f" 2>/dev/null && pass=$((pass+1)) || true
    done
    log "inflight wps=$n arrived=$pass"
    sleep 60
  done
  log "inflight clear"
}

gate_ok() {
  [[ -f "$MD/AUTO_EVAL.json" ]] || return 1
  python3 - <<'PY'
import json, sys
from pathlib import Path
d = json.load(open("artifacts/bridge_humen_001/AUTO_EVAL.json"))
ok = d.get("decision") == "done_pull" and Path("artifacts/bridge_humen_001/AUTOPILOT_DONE.flag").is_file()
flag = Path("artifacts/bridge_humen_001/AUTOPILOT_DONE.flag")
flag_txt = flag.read_text().strip() if flag.is_file() else ""
sys.exit(0 if (ok and flag_txt == "DONE_OK") else 1)
PY
}

log "=== gate_until_done start ==="
export GATE_MIN_SR="${GATE_MIN_SR:-0.70}"
export GATE_MIN_PTS="${GATE_MIN_PTS:-400}"
export GATE_MIN_N="${GATE_MIN_N:-16}"
export GATE_MIN_Y_SPAN_M="${GATE_MIN_Y_SPAN_M:-40}"
export HUMEN_AUTOPILOT_MAX_ROUNDS="${HUMEN_AUTOPILOT_MAX_ROUNDS:-40}"

wait_inflight

# Handoff: process whatever v15 just produced, then continue if gate fails.
if [[ ! -f "$MD/AUTOPILOT_DONE.flag" ]] || ! grep -q DONE_OK "$MD/AUTOPILOT_DONE.flag" 2>/dev/null; then
  # Kill stale autopilot bash (functions already bound to old gate) if still around
  pkill -9 -f "bash scripts/humen_autopilot.sh" 2>/dev/null || true
  sleep 2
  log "handoff SKIP_FIRST=1 HANDOFF_TAG=v15"
  SKIP_FIRST=1 HANDOFF_TAG=v15 nohup bash scripts/humen_autopilot.sh \
    >> "$MD/humen_autopilot.log" 2>&1 &
  echo $! > "$MD/humen_autopilot.pid"
  log "started autopilot pid=$(cat "$MD/humen_autopilot.pid")"
fi

# Keep restarting autopilot until DONE_OK (covers crash / max-round restart)
while true; do
  sleep 120
  if gate_ok; then
    log "GATE PASSED — DONE_OK"
    # pull hint
    echo READY_FOR_PULL > "$MD/v13_ready.flag"
    exit 0
  fi
  if ! pgrep -f "bash scripts/humen_autopilot.sh" >/dev/null 2>&1 \
     && ! pgrep -f "run_humen_sim_pipeline|aerial_inspect.cli sim-survey|wam_phase2_long_eval.*survey" >/dev/null 2>&1; then
    if [[ -f "$MD/AUTOPILOT_DONE.flag" ]] && grep -q DONE_OK "$MD/AUTOPILOT_DONE.flag"; then
      log "DONE_OK flag present"
      exit 0
    fi
    log "autopilot dead without gate — restart"
    rm -f "$MD/AUTOPILOT_DONE.flag"
    # continue from next round (do not SKIP_FIRST)
    nohup bash scripts/humen_autopilot.sh >> "$MD/humen_autopilot.log" 2>&1 &
    echo $! > "$MD/humen_autopilot.pid"
    log "restarted autopilot pid=$(cat "$MD/humen_autopilot.pid")"
  else
    # progress tick
    if [[ -f "$MD/AUTO_EVAL.json" ]]; then
      python3 -c "import json;d=json.load(open('artifacts/bridge_humen_001/AUTO_EVAL.json'));print(d.get('tag'),d.get('decision'),d.get('reason'),'sr',d.get('sr'),'pts',d.get('recon_points'),'n',d.get('n_eval'))" 2>/dev/null | tee -a "$LOG" || true
    fi
    n=$(ls "$MD/sim_runs/survey" 2>/dev/null | wc -l | tr -d ' ')
    log "alive survey_wps=$n"
  fi
done
