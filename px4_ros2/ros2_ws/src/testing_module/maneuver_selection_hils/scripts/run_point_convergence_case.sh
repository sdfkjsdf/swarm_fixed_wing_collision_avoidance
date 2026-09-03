#!/usr/bin/env bash

# Run one headless five-aircraft point-convergence case, record first, then
# generate all metrics and the MP4 from the completed bag.
set -o pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
HILS_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
ROS2_WS=${ROS2_WS:-/home/hmcl/workspace/swarm-fixed-wing/ros2_ws}
# Use the same built workspace for ROS interfaces, the guidance binary, and
# rosbag analysis. Sourcing a second stale in-repository overlay can silently
# mix incompatible ManeuverSelectionDecision schemas.
COLLISION_WS=${COLLISION_WS:-${ROS2_WS}}
FORMATION_HILS=${FORMATION_HILS:-${ROS2_WS}/src/testing_module/formation_hils}
RESULT_ROOT=${RESULT_ROOT:-${HILS_ROOT}/result}
RUN_DURATION_SECONDS=${RUN_DURATION_SECONDS:-55}
MODE=${1:-avoidance}
# HILS validation evaluates every configured roll candidate.  The heuristic
# three-candidate reduction remains available only as an explicit comparison.
SEARCH_MODE=${MANEUVER_SEARCH_MODE:-exhaustive}
V4_MODE=${V4_MODE:-shadow}
EXECUTION_POLICY=${AVOIDANCE_EXECUTION_POLICY:-amac_ad_threshold}
V4_CONTROL_ARCHITECTURE=${V4_CONTROL_ARCHITECTURE:-legacy_safe_control_set}
AMAC_POLICY_CONFIG=${AMAC_POLICY_CONFIG:-${HILS_ROOT}/config/amac_dynamic_best.yaml}
AMAC_COMMUNICATION_DELAY_MARGIN_M=${AMAC_COMMUNICATION_DELAY_MARGIN_M:-0.0}
AMAC_INTERACTION_GRAPH_SHADOW_ENABLED=${AMAC_INTERACTION_GRAPH_SHADOW_ENABLED:-false}
AMAC_INTERACTION_GRAPH_COMPONENT_CUTOVER_ENABLED=${AMAC_INTERACTION_GRAPH_COMPONENT_CUTOVER_ENABLED:-false}
AMAC_INTERACTION_GRAPH_AD_SCREEN_M=${AMAC_INTERACTION_GRAPH_AD_SCREEN_M:-0.0}
AMAC_TRAJECTORY_LIBRARY_VERSION=${AMAC_TRAJECTORY_LIBRARY_VERSION:-1}
AMAC_AD_MASD_CONFIG_VERSION=${AMAC_AD_MASD_CONFIG_VERSION:-1}
TRAFFIC_PATTERN=${TRAFFIC_PATTERN:-point_convergence}
# Keep the old names as input aliases only. User-facing scenario/result
# semantics use Formation consistently.
if [[ "${TRAFFIC_PATTERN}" == "flocking" ]]; then
    TRAFFIC_PATTERN=formation
elif [[ "${TRAFFIC_PATTERN}" == "flocking_pentagon" ]]; then
    TRAFFIC_PATTERN=formation_pentagon
fi

if [[ "${MODE}" != "avoidance" && "${MODE}" != "baseline" ]]; then
    echo "Usage: run_point_convergence_case.sh [avoidance|baseline]"
    exit 2
fi
if [[ "${SEARCH_MODE}" != "heuristic" && "${SEARCH_MODE}" != "exhaustive" ]]; then
    echo "MANEUVER_SEARCH_MODE must be heuristic or exhaustive"
    exit 2
fi
if [[ "${V4_MODE}" != "shadow" && "${V4_MODE}" != "cutover" ]]; then
    echo "V4_MODE must be shadow or cutover"
    exit 2
fi
if [[ "${V4_CONTROL_ARCHITECTURE}" != "legacy_safe_control_set" \
        && "${V4_CONTROL_ARCHITECTURE}" != "closed_form_backup_mode_b" ]]; then
    echo "V4_CONTROL_ARCHITECTURE must be legacy_safe_control_set or closed_form_backup_mode_b"
    exit 2
fi
if [[ "${EXECUTION_POLICY}" != "amac_ad_threshold" \
        && "${EXECUTION_POLICY}" != "continuous_v4" \
        && "${EXECUTION_POLICY}" != "horizon_gated_v4" ]]; then
    echo "AVOIDANCE_EXECUTION_POLICY must be amac_ad_threshold, continuous_v4, or horizon_gated_v4"
    exit 2
fi
if [[ "${EXECUTION_POLICY}" == "amac_ad_threshold" \
        && "${V4_MODE}" != "shadow" ]]; then
    echo "amac_ad_threshold comparison requires V4_MODE=shadow"
    exit 2
fi
if [[ "${AMAC_INTERACTION_GRAPH_COMPONENT_CUTOVER_ENABLED}" == "true" \
        && ("${AMAC_INTERACTION_GRAPH_SHADOW_ENABLED}" != "true" \
            || "${SEARCH_MODE}" != "exhaustive" \
            || "${EXECUTION_POLICY}" != "amac_ad_threshold") ]]; then
    echo "interaction-graph component cutover requires graph enabled, exhaustive search, and amac_ad_threshold"
    exit 2
fi
if [[ ("${EXECUTION_POLICY}" == "continuous_v4" \
        || "${EXECUTION_POLICY}" == "horizon_gated_v4") \
        && "${V4_MODE}" != "cutover" ]]; then
    echo "V4 execution policies require V4_MODE=cutover"
    exit 2
fi
if [[ "${V4_MODE}" == "cutover" && "${SEARCH_MODE}" == "exhaustive" ]]; then
    echo "V4 cutover is incompatible with the legacy exhaustive roll search"
    exit 2
fi
if [[ "${TRAFFIC_PATTERN}" != "point_convergence" \
        && "${TRAFFIC_PATTERN}" != "opposite_edge_crossing" \
        && "${TRAFFIC_PATTERN}" != "formation" \
        && "${TRAFFIC_PATTERN}" != "formation_pentagon" ]]; then
    echo "TRAFFIC_PATTERN must be point_convergence, opposite_edge_crossing, formation, or formation_pentagon"
    exit 2
fi

SHADOW_ONLY=true
if [[ "${MODE}" == "avoidance" ]]; then
    SHADOW_ONLY=false
fi
EXHAUSTIVE_TEST_MODE=false
if [[ "${SEARCH_MODE}" == "exhaustive" ]]; then
    EXHAUSTIVE_TEST_MODE=true
fi
V4_SHADOW_ONLY=true
# Keep the Lockheed/AMAC comparison free of the project TC-CBF candidate
# gate.  TC-CBF experiments must opt in explicitly through the environment.
POSITIVE_MARGIN_FILTER_ENABLED=${POSITIVE_MARGIN_FILTER_ENABLED:-false}
if [[ "${V4_MODE}" == "cutover" ]]; then
    V4_SHADOW_ONLY=false
    POSITIVE_MARGIN_FILTER_ENABLED=false
fi
RUN_ID=${RUN_ID:-point_${MODE}_$(date +%Y%m%d_%H%M%S)}
# Common-NED targets and initial courses for the fixed pentagon spawn. The
# opposite-edge pattern sends vehicle i toward the midpoint of the initial
# positions of vehicles i+2 and i+3 (modulo five). Its straight paths cross
# the center but continue toward five distinct destinations.
if [[ "${TRAFFIC_PATTERN}" == "opposite_edge_crossing" ]]; then
    SPAWN_CONFIG=${HILS_ROOT}/config/spawn_point_convergence.yaml
    COORDINATE_CONFIG=${HILS_ROOT}/config/coordinate_point_convergence.yaml
    GUIDANCE_CONFIG=${HILS_ROOT}/config/point_convergence_guidance.yaml
    GUIDANCE_MODE=point_convergence
    SHOW_ASSIGNED_TARGETS=true
    TARGET_NORTHS=(97.746 237.500 463.627 463.627 237.500)
    TARGET_EASTS=(300.000 107.645 181.118 418.882 492.355)
    COURSES=(3.141592654 -1.884955309 -0.628318340 0.628318340 1.884955309)
elif [[ "${TRAFFIC_PATTERN}" == "formation" ]]; then
    SPAWN_CONFIG=${FORMATION_HILS}/config/spawn_config.yaml
    COORDINATE_CONFIG=${COLLISION_WS}/src/collision_avoidance/config/ros_params.yaml
    GUIDANCE_CONFIG=${COLLISION_WS}/src/collision_avoidance/config/flocking_params.yaml
    GUIDANCE_MODE=formation
    SHOW_ASSIGNED_TARGETS=false
    TARGET_NORTHS=(0.0 0.0 0.0 0.0 0.0)
    TARGET_EASTS=(0.0 0.0 0.0 0.0 0.0)
    COURSES=(0.0 0.0 0.0 0.0 0.0)
elif [[ "${TRAFFIC_PATTERN}" == "formation_pentagon" ]]; then
    SPAWN_CONFIG=${HILS_ROOT}/config/spawn_point_convergence.yaml
    COORDINATE_CONFIG=${HILS_ROOT}/config/coordinate_point_convergence.yaml
    GUIDANCE_CONFIG=${COLLISION_WS}/src/collision_avoidance/config/flocking_params.yaml
    GUIDANCE_MODE=formation
    SHOW_ASSIGNED_TARGETS=false
    TARGET_NORTHS=(0.0 0.0 0.0 0.0 0.0)
    TARGET_EASTS=(0.0 0.0 0.0 0.0 0.0)
    COURSES=(0.0 0.0 0.0 0.0 0.0)
else
    SPAWN_CONFIG=${HILS_ROOT}/config/spawn_point_convergence.yaml
    COORDINATE_CONFIG=${HILS_ROOT}/config/coordinate_point_convergence.yaml
    GUIDANCE_CONFIG=${HILS_ROOT}/config/point_convergence_guidance.yaml
    GUIDANCE_MODE=point_convergence
    SHOW_ASSIGNED_TARGETS=true
    TARGET_NORTHS=(300.000 300.000 300.000 300.000 300.000)
    TARGET_EASTS=(300.000 300.000 300.000 300.000 300.000)
    COURSES=(3.141592654 -1.884955592 -0.628318531 0.628318531 1.884955592)
fi
TARGET_NORTHS_CSV=${TARGET_NORTHS[*]}
TARGET_NORTHS_CSV=${TARGET_NORTHS_CSV// /,}
TARGET_EASTS_CSV=${TARGET_EASTS[*]}
TARGET_EASTS_CSV=${TARGET_EASTS_CSV// /,}
BAG_DIR=${RESULT_ROOT}/rosbag/${RUN_ID}
LOG_DIR=${RESULT_ROOT}/log/${RUN_ID}
mkdir -p "${LOG_DIR}" "$(dirname "${BAG_DIR}")"

source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"
if [[ ! -f "${COLLISION_WS}/install/setup.bash" ]]; then
    echo "[run] ERROR: collision overlay is not built: ${COLLISION_WS}"
    exit 1
fi
source "${COLLISION_WS}/install/setup.bash"

AGENT_BIN=${MICRO_XRCE_AGENT_BIN:-/home/hmcl/workspace/swarm-fixed-wing/tools/micro-xrce-dds-agent/bin/MicroXRCEAgent}
AGENT_LIB_DIR=${MICRO_XRCE_AGENT_LIB_DIR:-$(dirname "${AGENT_BIN}")/../lib}
GUIDANCE_BIN=$(ros2 pkg prefix collision_avoidance)/lib/collision_avoidance/vtol_guidance_node
AGENT_PIDS=()
GUIDANCE_PIDS=()
SIM_PID=""
BAG_PID=""
CLEANED=0

stop_bag() {
    if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
        kill -INT "${BAG_PID}" 2>/dev/null || true
        wait "${BAG_PID}" 2>/dev/null || true
    fi
    BAG_PID=""
}

cleanup() {
    [[ ${CLEANED} -eq 1 ]] && return
    CLEANED=1
    stop_bag
    for pid in "${GUIDANCE_PIDS[@]}"; do
        kill -TERM "${pid}" 2>/dev/null || true
    done
    for pid in "${GUIDANCE_PIDS[@]}"; do
        wait "${pid}" 2>/dev/null || true
    done
    if [[ -n "${SIM_PID}" ]] && kill -0 "${SIM_PID}" 2>/dev/null; then
        kill -TERM "${SIM_PID}" 2>/dev/null || true
        wait "${SIM_PID}" 2>/dev/null || true
    fi
    for pid in "${AGENT_PIDS[@]}"; do
        kill -TERM "${pid}" 2>/dev/null || true
    done
    for pid in "${AGENT_PIDS[@]}"; do
        wait "${pid}" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

if [[ ! -x "${AGENT_BIN}" ]]; then
    echo "[run] ERROR: MicroXRCEAgent not executable: ${AGENT_BIN}"
    exit 1
fi
if [[ ! -f "${AMAC_POLICY_CONFIG}" ]]; then
    echo "[run] ERROR: AMAC policy config not found: ${AMAC_POLICY_CONFIG}"
    exit 2
fi
if [[ "${GUIDANCE_MODE}" == "point_convergence" ]] \
        && ! grep -Eq 'test_guidance_mode:[[:space:]]*point_convergence' \
            "${GUIDANCE_CONFIG}"; then
    echo "[run] ERROR: HILS guidance config is not point_convergence: ${GUIDANCE_CONFIG}"
    exit 2
fi
if [[ -e "${BAG_DIR}" ]]; then
    echo "[run] ERROR: result already exists: ${BAG_DIR}"
    exit 2
fi

echo "[run] pattern=${TRAFFIC_PATTERN} mode=${MODE} search=${SEARCH_MODE} v4=${V4_MODE} v4_architecture=${V4_CONTROL_ARCHITECTURE} policy=${EXECUTION_POLICY} ad_threshold=0m communication_delay_margin=${AMAC_COMMUNICATION_DELAY_MARGIN_M}m interaction_graph_shadow=${AMAC_INTERACTION_GRAPH_SHADOW_ENABLED} graph_component_cutover=${AMAC_INTERACTION_GRAPH_COMPONENT_CUTOVER_ENABLED} AD_screen=${AMAC_INTERACTION_GRAPH_AD_SCREEN_M}m active_switch_config=${AMAC_POLICY_CONFIG} run_id=${RUN_ID} duration=${RUN_DURATION_SECONDS}s"
for vehicle in 0 1 2 3 4; do
    port=$((8888 + vehicle))
    pkill -KILL -f "MicroXRCEAgent udp4 -p ${port}" 2>/dev/null || true
    LD_LIBRARY_PATH="${AGENT_LIB_DIR}:${LD_LIBRARY_PATH:-}" \
        "${AGENT_BIN}" udp4 -p "${port}" \
        > "${LOG_DIR}/micro_xrce_${vehicle}.log" 2>&1 &
    AGENT_PIDS+=($!)
done
sleep 2

SPAWN_CONFIG_FILE="${SPAWN_CONFIG}" \
ROS_PARAMS_FILE="${COORDINATE_CONFIG}" \
    bash "${FORMATION_HILS}/scripts/launch_5vtol.sh" noshow \
    > "${LOG_DIR}/simulation.log" 2>&1 &
SIM_PID=$!

echo "[run] waiting for all five PX4 odometry topics"
READY=0
for attempt in $(seq 1 100); do
    topic_list=$(ros2 topic list 2>/dev/null || true)
    ready_count=0
    for vehicle in 0 1 2 3 4; do
        if grep -q "^/px4_${vehicle}/fmu/out/vehicle_odometry$" \
                <<< "${topic_list}"; then
            ready_count=$((ready_count + 1))
        fi
    done
    if [[ ${ready_count} -eq 5 ]]; then
        READY=1
        echo "[run] PX4 topics ready after ${attempt}s"
        break
    fi
    if ! kill -0 "${SIM_PID}" 2>/dev/null; then
        echo "[run] ERROR: simulation launcher exited early"
        tail -80 "${LOG_DIR}/simulation.log" || true
        exit 1
    fi
    sleep 1
done
if [[ ${READY} -ne 1 ]]; then
    echo "[run] ERROR: PX4 topics did not become ready"
    exit 1
fi

bash "${SCRIPT_DIR}/record_point_convergence_bag.sh" "${BAG_DIR}" \
    > "${LOG_DIR}/rosbag.log" 2>&1 &
BAG_PID=$!
sleep 2

for vehicle in 0 1 2 3 4; do
    "${GUIDANCE_BIN}" \
        --ros-args \
        --params-file "${GUIDANCE_CONFIG}" \
        --params-file "${AMAC_POLICY_CONFIG}" \
        -r "__node:=vtol_guidance_${vehicle}" \
        -p "vehicle_ID:=${vehicle}" \
        -p total_agent_num:=5 \
        -p "test_guidance_mode:=${GUIDANCE_MODE}" \
        -p "point_target_north_m:=${TARGET_NORTHS[$vehicle]}" \
        -p "point_target_east_m:=${TARGET_EASTS[$vehicle]}" \
        -p "preflight_desired_course_rad:=${COURSES[$vehicle]}" \
        -p preflight_desired_ground_speed_mps:=20.0 \
        -p "collision_avoidance_shadow_only:=${SHADOW_ONLY}" \
        -p "avoidance_execution_policy:=${EXECUTION_POLICY}" \
        -p "amac_communication_delay_margin_m:=${AMAC_COMMUNICATION_DELAY_MARGIN_M}" \
        -p "amac_interaction_graph_shadow_enabled:=${AMAC_INTERACTION_GRAPH_SHADOW_ENABLED}" \
        -p "amac_interaction_graph_component_cutover_enabled:=${AMAC_INTERACTION_GRAPH_COMPONENT_CUTOVER_ENABLED}" \
        -p "amac_interaction_graph_ad_screen_m:=${AMAC_INTERACTION_GRAPH_AD_SCREEN_M}" \
        -p "amac_trajectory_library_version:=${AMAC_TRAJECTORY_LIBRARY_VERSION}" \
        -p "amac_ad_masd_config_version:=${AMAC_AD_MASD_CONFIG_VERSION}" \
        -p "maneuver_selection_exhaustive_test_mode:=${EXHAUSTIVE_TEST_MODE}" \
        -p v4_safe_control_enabled:=true \
        -p "v4_shadow_only:=${V4_SHADOW_ONLY}" \
        -p "v4_control_architecture:=${V4_CONTROL_ARCHITECTURE}" \
        -p "positive_margin_filter_enabled:=${POSITIVE_MARGIN_FILTER_ENABLED}" \
        > "${LOG_DIR}/guidance_${vehicle}.log" 2>&1 &
    GUIDANCE_PIDS+=($!)
done

echo "[run] waiting for all five guidance nodes to register"
REGISTERED=0
for attempt in $(seq 1 25); do
    registered_count=0
    for vehicle in 0 1 2 3 4; do
        if grep -q "doRegister() 성공" \
                "${LOG_DIR}/guidance_${vehicle}.log" 2>/dev/null; then
            registered_count=$((registered_count + 1))
        fi
    done
    if [[ ${registered_count} -eq 5 ]]; then
        REGISTERED=1
        echo "[run] all guidance nodes registered after ${attempt}s"
        break
    fi
    for pid in "${GUIDANCE_PIDS[@]}"; do
        if ! kill -0 "${pid}" 2>/dev/null; then
            echo "[run] ERROR: a guidance node exited before registration"
            for vehicle in 0 1 2 3 4; do
                tail -20 "${LOG_DIR}/guidance_${vehicle}.log" 2>/dev/null || true
            done
            exit 1
        fi
    done
    sleep 1
done
if [[ ${REGISTERED} -ne 1 ]]; then
    echo "[run] ERROR: guidance registration timed out"
    exit 1
fi

sleep "${RUN_DURATION_SECONDS}"
stop_bag
cleanup
trap - EXIT INT TERM

if [[ ! -f "${BAG_DIR}/metadata.yaml" ]]; then
    echo "[run] ERROR: rosbag was not finalized"
    exit 1
fi

formation_ready_count=0
for vehicle in 0 1 2 3 4; do
    if grep -q "\[Formation\] 활성화" \
            "${LOG_DIR}/guidance_${vehicle}.log" 2>/dev/null; then
        formation_ready_count=$((formation_ready_count + 1))
    fi
done
if [[ ${formation_ready_count} -ne 5 ]]; then
    echo "[run] ERROR: only ${formation_ready_count}/5 vehicles entered Formation; bag is not a valid avoidance sample"
    exit 1
fi

RESULT_ROOT="${RESULT_ROOT}" \
SCENARIO_LABEL="${TRAFFIC_PATTERN}" \
SHOW_ASSIGNED_TARGETS="${SHOW_ASSIGNED_TARGETS}" \
TARGET_NORTHS_CSV="${TARGET_NORTHS_CSV}" \
TARGET_EASTS_CSV="${TARGET_EASTS_CSV}" \
    bash "${SCRIPT_DIR}/process_point_convergence_bag.sh" "${RUN_ID}"
analysis_rc=$?
echo "[run] results: ${RESULT_ROOT} (analysis_rc=${analysis_rc})"
exit "${analysis_rc}"
