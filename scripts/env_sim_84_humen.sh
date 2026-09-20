#!/usr/bin/env bash
# 84 + Avant-AirSim HumenCorridor (ports 2200 / 41463; OpenFly on 41451 unchanged).
# source scripts/env_sim_84_humen.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "$ROOT/scripts/env_sim_84.sh"

export AIRSIM_SCENE_ID=avant_humen_corridor
export AIRSIM_SCENE_CONFIG=configs/sim/humen_scene.yaml
export AIRSIM_PORT=41463
export AIRSIM_CAMERA=0
export AIRSIM_VEHICLE=SimpleFlight
export AVANT_AIRSIM_ROOT="${AVANT_AIRSIM_ROOT:-/data/linux/workspace/Avant-AirSim/Avant-AirSim}"
export BRIDGE_CAMERA_PITCH="${BRIDGE_CAMERA_PITCH:--20}"

# Mainline: Humen corridor multi-class open-vocab (tower + bridge prompts).
# Probe: bridge<0.02, tower~0.35 at x=1200 — humen_corridor, no geometric fallback.
# Override env_sim_84 default (open_vocab) — Humen needs corridor multi-class detector.
export SIM_DETECTOR=humen_corridor
export SIM_VISUAL_PROMPT="${SIM_VISUAL_PROMPT:-tower suspension bridge tower cable}"
export SIM_YOLO_CONF=0.04
export SIM_YOLO_CONF_HUMEN=0.04
export SIM_TRACKER_MIN_CONF=0.10
export SIM_REJECT_FAR_LOCK_M=1200
export SIM_SEARCH_AT_CRUISE=1
export SIM_SEARCH_PATTERN=corridor
export SIM_SEARCH_START_YAW_DEG=45
export SIM_SEARCH_YAW_HOLD_DEG=35
export SIM_SEARCH_SWEEP_M=30
export SIM_SEARCH_WP_RADIUS_M=10
export SIM_CORRIDOR_YAW_SWEEP_INTERVAL=45
export SIM_CORRIDOR_YAW_SWEEP_STEPS=12
export SIM_CORRIDOR_YAW_SWEEP_DEG=60
export SIM_CORRIDOR_YAW_SWEEP_STEP_DEG=5
export SIM_SEARCH_AREA_PRIORITY=1
export AERIAL_REQUIRE_DET_HIT=1
export AERIAL_MIN_SEARCH_X_SPAN_M=200
# 主航道主跨在 x≈[2274,3174]（视觉确认「虎门大桥」塔牌，见 bridge_humen.yaml）。
# 旧版本误以为主航道在 spawn/西段走廊 x<1000，已废弃；SEARCH 必须真正飞到主跨附近才算通过。
export AERIAL_MIN_SEARCH_MAX_X="${AERIAL_MIN_SEARCH_MAX_X:-2000}"
export AERIAL_APPROACH_MIN_DET_FRACTION=0.03
# Reject SEARCH centroids that leave the main channel corridor band.
export AERIAL_CORRIDOR_Y="${AERIAL_CORRIDOR_Y:--50}"
export AERIAL_CORRIDOR_Y_TOL_M="${AERIAL_CORRIDOR_Y_TOL_M:-40}"
export AERIAL_CENTROID_X_MAX="${AERIAL_CENTROID_X_MAX:-200}"
export SIM_APPROACH_Z_HOLD=95
export SIM_APPROACH_WP_RADIUS_M=10
export SIM_APPROACH_SUCCESS_DIST_M=8
# Corridor pattern flies along the deck centerline itself (not toward it from
# outside), so genuine detections are naturally close-range (~10-50m) the whole
# way; a 40m median-distance gate flags real hits as "near-field false lock".
export AERIAL_MIN_GOAL_REL_DIST_M=15
export AERIAL_MIN_SEARCH_GOAL_REL_M="${AERIAL_MIN_SEARCH_GOAL_REL_M:-15}"
export AERIAL_MIN_SEARCH_SAMPLES=10
export QC_DETECTOR=humen_corridor
export AERIAL_QC_MIN_FRACTION=0.45
export SURVEY_AIM_MODE=deck_lookat
export SURVEY_CAMERA_MOUNT_PITCH=0
export SIM_SURVEY_WP_RADIUS_M=12
export SIM_SURVEY_SUCCESS_DIST_M=3
export SIM_SURVEY_NAV=phase2
export SIM_SURVEY_POLY_SPACING_M=15
export SIM_SURVEY_TIP_MARGIN_M="${SIM_SURVEY_TIP_MARGIN_M:-40}"
export SIM_SURVEY_TRANSIT_ALT_BOOST_M="${SIM_SURVEY_TRANSIT_ALT_BOOST_M:-30}"
export SIM_SURVEY_TERMINAL_PIN_M=20
export SIM_SURVEY_TERMINAL_CREEP_M=15
export SIM_SURVEY_CRUISE_SPEED=10.0
# Opposite tip transit ~200m+; give Phase-2 more steps than near-facade segments.
export SIM_MAX_STEPS_PER_WP="${SIM_MAX_STEPS_PER_WP:-900}"
# Tip transit: Phase-2 cannot follow long opposite polylines — scripted teleport hop.
export SIM_SURVEY_SCRIPTED_TIP="${SIM_SURVEY_SCRIPTED_TIP:-1}"
export SIM_SURVEY_SCRIPTED_TIP_MIN_M="${SIM_SURVEY_SCRIPTED_TIP_MIN_M:-80}"
# Disabled: min-spawn-z retries hung AirSim reset on Humen (2026-09-17).
# export SIM_SURVEY_MIN_SPAWN_Z=40
# Humen open-water depth hallucinates → three_zone IR~100% and wrong-way flight.
export SIM_SURVEY_NO_SHIELD="${SIM_SURVEY_NO_SHIELD:-1}"
export SURVEY_EXPORT_REQUIRE_POSE=1
export SURVEY_EXPORT_MAX_XY_ERROR_M=15
export SURVEY_EXPORT_MAX_Z_ERROR_M=12

echo "[env_sim_84_humen] AIRSIM=$AIRSIM_HOST:$AIRSIM_PORT scene=$AIRSIM_SCENE_CONFIG"
echo "[env_sim_84_humen] detector=$SIM_DETECTOR (no fake structure fallback)"
echo "[env_sim_84_humen] AVANT_AIRSIM_ROOT=$AVANT_AIRSIM_ROOT"
echo "[env_sim_84_humen] vehicle=$AIRSIM_VEHICLE camera=$AIRSIM_CAMERA pitch=$BRIDGE_CAMERA_PITCH"
