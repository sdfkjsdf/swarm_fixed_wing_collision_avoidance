#!/usr/bin/env bash

# Launch exactly one vehicle's production guidance node. The same installed
# script is used by local HILS processes and by the onboard Docker image.
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: run_guidance_vehicle.sh VEHICLE_ID [TOTAL_AGENT_NUM]" >&2
    exit 2
fi

VEHICLE_ID=$1
TOTAL_AGENT_NUM=${2:-5}
if [[ ! "${TOTAL_AGENT_NUM}" =~ ^[1-9][0-9]*$ ]]; then
    echo "TOTAL_AGENT_NUM must be a positive integer" >&2
    exit 2
fi
if [[ ! "${VEHICLE_ID}" =~ ^[0-9]+$ ]] \
        || (( VEHICLE_ID < 0 || VEHICLE_ID >= TOTAL_AGENT_NUM )); then
    echo "VEHICLE_ID must be in [0, TOTAL_AGENT_NUM)" >&2
    exit 2
fi

PACKAGE_PREFIX=$(ros2 pkg prefix collision_avoidance)
PACKAGE_SHARE=$(ros2 pkg prefix --share collision_avoidance)
GUIDANCE_BIN=${GUIDANCE_BIN:-${PACKAGE_PREFIX}/lib/collision_avoidance/vtol_guidance_node}
GUIDANCE_CONFIG=${GUIDANCE_CONFIG:-${PACKAGE_SHARE}/config/${GUIDANCE_CONFIG_NAME:-flocking_params.yaml}}
AMAC_POLICY_CONFIG=${AMAC_POLICY_CONFIG:-${PACKAGE_SHARE}/config/${AMAC_POLICY_CONFIG_NAME:-amac_distributed_formation.yaml}}

for required_file in "${GUIDANCE_BIN}" "${GUIDANCE_CONFIG}" "${AMAC_POLICY_CONFIG}"; do
    if [[ ! -e "${required_file}" ]]; then
        echo "required guidance artifact not found: ${required_file}" >&2
        exit 1
    fi
done

exec "${GUIDANCE_BIN}" \
    --ros-args \
    --params-file "${GUIDANCE_CONFIG}" \
    --params-file "${AMAC_POLICY_CONFIG}" \
    -r "__node:=vtol_guidance_${VEHICLE_ID}" \
    -p "vehicle_ID:=${VEHICLE_ID}" \
    -p "total_agent_num:=${TOTAL_AGENT_NUM}" \
    -p "test_guidance_mode:=${GUIDANCE_MODE:-formation}" \
    -p "point_target_north_m:=${POINT_TARGET_NORTH_M:-0.0}" \
    -p "point_target_east_m:=${POINT_TARGET_EAST_M:-0.0}" \
    -p "preflight_desired_course_rad:=${PREFLIGHT_DESIRED_COURSE_RAD:-0.0}" \
    -p "preflight_desired_ground_speed_mps:=${PREFLIGHT_DESIRED_GROUND_SPEED_MPS:-20.0}" \
    -p "collision_avoidance_shadow_only:=${COLLISION_AVOIDANCE_SHADOW_ONLY:-false}" \
    -p "avoidance_execution_policy:=${AVOIDANCE_EXECUTION_POLICY:-amac_ad_threshold}" \
    -p "amac_communication_delay_margin_m:=${AMAC_COMMUNICATION_DELAY_MARGIN_M:-0.0}" \
    -p "amac_interaction_graph_enabled:=${AMAC_INTERACTION_GRAPH_ENABLED:-false}" \
    -p "amac_interaction_graph_ad_screen_m:=${AMAC_INTERACTION_GRAPH_AD_SCREEN_M:-0.0}" \
    -p "amac_trajectory_library_version:=${AMAC_TRAJECTORY_LIBRARY_VERSION:-1}" \
    -p "amac_ad_masd_config_version:=${AMAC_AD_MASD_CONFIG_VERSION:-1}" \
    -p "maneuver_selection_exhaustive_test_mode:=${MANEUVER_SELECTION_EXHAUSTIVE_TEST_MODE:-false}" \
    -p "v4_safe_control_enabled:=${V4_SAFE_CONTROL_ENABLED:-false}" \
    -p "v4_shadow_only:=${V4_SHADOW_ONLY:-true}" \
    -p "v4_control_architecture:=${V4_CONTROL_ARCHITECTURE:-legacy_safe_control_set}" \
    -p "positive_margin_filter_enabled:=${POSITIVE_MARGIN_FILTER_ENABLED:-false}"
