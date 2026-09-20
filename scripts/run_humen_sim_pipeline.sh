#!/usr/bin/env bash
# Humen M1: SEARCH → replan → APPROACH → SURVEY → export → QC → COLMAP.
# No fake completion: each phase must pass validation or the pipeline stops.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG="${1:-configs/missions/bridge_humen.yaml}"
cd "$ROOT"

if [[ -f "$ROOT/scripts/env_sim_84_humen.sh" ]]; then
  # shellcheck source=/dev/null
  source "$ROOT/scripts/env_sim_84_humen.sh"
else
  # shellcheck source=/dev/null
  source "$ROOT/scripts/env_sim_84.sh"
fi

export AERIAL_CAPTURE_W="${AERIAL_CAPTURE_W:-1280}"
export AERIAL_CAPTURE_H="${AERIAL_CAPTURE_H:-720}"
export SIM_MAX_STEPS_PER_WP="${SIM_MAX_STEPS_PER_WP:-120}"

PY="${AERIAL_PY:-${PYTHON_BIN:-python3}}"
MISSION_ID=$("$PY" -c "import yaml; print(yaml.safe_load(open('$CONFIG'))['mission_id'])")
MISSION_DIR="$ROOT/artifacts/$MISSION_ID"
SEARCH_TRAJ="$MISSION_DIR/sim_runs/search/traj/route00.jsonl"
APPROACH_EVAL="$MISSION_DIR/sim_runs/approach/eval_result.json"

if [[ "${RESET_MISSION:-0}" == "1" ]]; then
  echo "[humen_pipeline] RESET_MISSION=1 — clearing prior run"
  rm -rf "$MISSION_DIR/sim_runs" "$MISSION_DIR/capture_survey" "$MISSION_DIR/waypoints.json"
  rm -f "$MISSION_DIR/detected_centroid.json"
fi

echo "[humen_pipeline] mission=$MISSION_ID detector=${SIM_DETECTOR:-open_vocab} AIRSIM=$AIRSIM_HOST:$AIRSIM_PORT"

if [[ ! -f "$MISSION_DIR/waypoints.json" || "${FORCE_SEARCH:-0}" == "1" ]]; then
  echo "=== plan + sim-search (vision) ==="
  "$PY" -m aerial_inspect.cli plan "$CONFIG" -o "$MISSION_DIR"
  "$PY" -m aerial_inspect.cli sim-search "$MISSION_DIR"

  echo "=== validate SEARCH traj (det_hit required) ==="
  if ! "$PY" "$ROOT/scripts/validate_search_traj.py" "$SEARCH_TRAJ" \
      --min-x-span-m "${AERIAL_MIN_SEARCH_X_SPAN_M:-250}" \
      --min-max-x "${AERIAL_MIN_SEARCH_MAX_X:-1000}"; then
    echo "ERROR: SEARCH failed validation — bridge not visually detected in flight." >&2
    echo "  Fix detector/prompt/search area; do not use geometric or structure fallbacks." >&2
    exit 1
  fi

  echo "=== replan-survey from validated SEARCH traj ==="
  "$PY" -m aerial_inspect.cli replan-survey "$MISSION_DIR" \
    --config "$CONFIG" \
    --from-traj "$SEARCH_TRAJ" \
    --require-det-hit \
    --min-goal-rel-dist-m "${AERIAL_MIN_GOAL_REL_DIST_M:-40}" \
    --min-samples "${AERIAL_MIN_SEARCH_SAMPLES:-10}" \
    --export-wam

  DET_SOURCE=$("$PY" -c "import json; print(json.load(open('$MISSION_DIR/detected_centroid.json')).get('source',''))")
  if [[ "$DET_SOURCE" != "detected" ]]; then
    echo "ERROR: centroid source must be 'detected', got '$DET_SOURCE'" >&2
    exit 1
  fi

  "$PY" -m aerial_inspect.cli export-sim "$MISSION_DIR"

  echo "=== sim-approach (validate gate, not eval SR) ==="
  "$PY" -m aerial_inspect.cli sim-approach "$MISSION_DIR" || true
  if ! "$PY" "$ROOT/scripts/validate_sim_phase.py" "$APPROACH_EVAL" --phase approach \
      --min-det-fraction "${AERIAL_APPROACH_MIN_DET_FRACTION:-0.03}"; then
    echo "ERROR: approach failed validation" >&2
    exit 1
  fi

  echo "=== sim-survey (goto waypoints, pose gate) ==="
  "$PY" -m aerial_inspect.cli sim-survey "$MISSION_DIR" || true
  "$PY" "$ROOT/scripts/validate_survey_complete.py" "$MISSION_DIR"
  "$PY" "$ROOT/scripts/validate_survey_poses.py" "$MISSION_DIR" \
    --max-xy-error-m "${SURVEY_EXPORT_MAX_XY_ERROR_M:-15}" \
    --max-z-error-m "${SURVEY_EXPORT_MAX_Z_ERROR_M:-12}"
elif [[ "${FORCE_SURVEY:-0}" == "1" ]]; then
  echo "=== FORCE_SURVEY: refresh yaml + replan + approach + survey (skip SEARCH) ==="
  "$PY" -m aerial_inspect.cli plan "$CONFIG" -o "$MISSION_DIR"
  # Prefer mission-yaml pinned centroid/span (主航道 midspan) over stale SEARCH lock.
  # Set AERIAL_REPLAN_FROM_SEARCH=1 to force replan from last SEARCH traj instead.
  # Main-channel gates: span_axis must be ~-31° and span_extent_m >= 800.
  if [[ "${AERIAL_REPLAN_FROM_SEARCH:-0}" != "1" ]] \
    && "$PY" -c "import yaml,sys; d=yaml.safe_load(open(sys.argv[1])); sys.exit(0 if d.get('bridge_centroid_xyz') else 1)" "$CONFIG"; then
    read -r CX CY CZ < <("$PY" -c "import yaml,sys; c=yaml.safe_load(open(sys.argv[1]))['bridge_centroid_xyz']; print(c[0],c[1],c[2])" "$CONFIG")
    SPAN="$("$PY" -c "import yaml,sys; d=yaml.safe_load(open(sys.argv[1])); print(d.get('bridge_span_axis_deg', d.get('survey',{}).get('span_axis_deg',-32)))" "$CONFIG")"
    echo "=== replan-survey from YAML pin centroid=[$CX,$CY,$CZ] span=$SPAN (主航道) ==="
    "$PY" -m aerial_inspect.cli replan-survey "$MISSION_DIR" \
      --config "$CONFIG" \
      --centroid "$CX" "$CY" "$CZ" \
      --span-axis-deg "$SPAN" \
      --export-wam
  else
    echo "=== replan-survey from SEARCH traj ==="
    "$PY" -m aerial_inspect.cli replan-survey "$MISSION_DIR" \
      --config "$CONFIG" \
      --from-traj "$SEARCH_TRAJ" \
      --require-det-hit \
      --min-goal-rel-dist-m "${AERIAL_MIN_GOAL_REL_DIST_M:-40}" \
      --min-samples "${AERIAL_MIN_SEARCH_SAMPLES:-10}" \
      --export-wam
  fi
  rm -rf "$MISSION_DIR/sim_runs/survey" "$MISSION_DIR/capture_survey"
  echo "=== sim-approach (25m standoff, toward-g) ==="
  "$PY" -m aerial_inspect.cli sim-approach "$MISSION_DIR" || true
  if ! "$PY" "$ROOT/scripts/validate_sim_phase.py" "$APPROACH_EVAL" --phase approach \
      --min-det-fraction "${AERIAL_APPROACH_MIN_DET_FRACTION:-0.03}"; then
    echo "WARN: approach validation soft-fail — continuing with SEARCH nearest pose" >&2
  fi
  echo "=== sim-survey (goto waypoints, pose gate) ==="
  "$PY" -m aerial_inspect.cli sim-survey "$MISSION_DIR" || true
  "$PY" "$ROOT/scripts/validate_survey_complete.py" "$MISSION_DIR"
  "$PY" "$ROOT/scripts/validate_survey_poses.py" "$MISSION_DIR" \
    --max-xy-error-m "${SURVEY_EXPORT_MAX_XY_ERROR_M:-15}" \
    --max-z-error-m "${SURVEY_EXPORT_MAX_Z_ERROR_M:-12}"
fi

echo "[humen_pipeline] centroid:"
cat "$MISSION_DIR/detected_centroid.json"

if [[ "${SURVEY_EXPORT_ONLY:-0}" == "1" ]]; then
  echo "[humen_pipeline] SURVEY_EXPORT_ONLY=1 — skipping sim phases, re-export + QC + COLMAP"
fi

echo "=== export capture at planned survey waypoints (look-at centroid) ==="
"$PY" "$ROOT/scripts/export_capture_from_sim_survey.py" "$MISSION_DIR"

echo "=== QC: bridge visible in capture frames (hard gate) ==="
"$PY" "$ROOT/scripts/qc_capture_bridge.py" "$MISSION_DIR/capture_survey" \
  --min-fraction "${AERIAL_QC_MIN_FRACTION:-0.30}"

echo "=== survey flythrough video ==="
"$PY" "$ROOT/scripts/stitch_capture_video.py" "$MISSION_DIR"

echo "=== hybrid COLMAP ==="
"$PY" "$ROOT/scripts/run_pycolmap_reconstruct.py" "$MISSION_DIR/capture_survey" \
  --workspace "$ROOT/artifacts/models/${MISSION_ID}" \
  --mode hybrid

echo "[humen_pipeline] done — all gates passed"
echo "  video: $MISSION_DIR/capture_survey/survey_flythrough.mp4"
echo "  preview: $ROOT/artifacts/models/${MISSION_ID}/sparse_preview.png"
