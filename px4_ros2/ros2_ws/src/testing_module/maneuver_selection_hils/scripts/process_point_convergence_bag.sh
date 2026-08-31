#!/usr/bin/env bash

set -o pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
HILS_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
ROS2_WS=${ROS2_WS:-/home/hmcl/workspace/swarm-fixed-wing/ros2_ws}
# Keep deserialization on the exact overlay used by the guidance processes.
COLLISION_WS=${COLLISION_WS:-${ROS2_WS}}
ANALYSIS_PYTHON=${ANALYSIS_PYTHON:-/home/hmcl/workspace/swarm-fixed-wing/.envs/px4/bin/python3}
RUN_ID=${1:?Usage: process_point_convergence_bag.sh RUN_ID}
RESULT_ROOT=${RESULT_ROOT:-${HILS_ROOT}/result}
BAG_DIR=${RESULT_ROOT}/rosbag/${RUN_ID}
LOG_DIR=${RESULT_ROOT}/log/${RUN_ID}
SCENARIO_LABEL=${SCENARIO_LABEL:-point_convergence}
SHOW_ASSIGNED_TARGETS=${SHOW_ASSIGNED_TARGETS:-true}
TARGET_NORTHS_CSV=${TARGET_NORTHS_CSV:-300.0,300.0,300.0,300.0,300.0}
TARGET_EASTS_CSV=${TARGET_EASTS_CSV:-300.0,300.0,300.0,300.0,300.0}
IFS=',' read -r -a TARGET_NORTHS <<< "${TARGET_NORTHS_CSV}"
IFS=',' read -r -a TARGET_EASTS <<< "${TARGET_EASTS_CSV}"

if [[ ${#TARGET_NORTHS[@]} -ne 5 || ${#TARGET_EASTS[@]} -ne 5 ]]; then
    echo "[process] ERROR: target arrays must each contain five values"
    exit 2
fi

source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"
if [[ ! -f "${COLLISION_WS}/install/setup.bash" ]]; then
    echo "[process] ERROR: collision overlay is not built: ${COLLISION_WS}"
    exit 2
fi
source "${COLLISION_WS}/install/setup.bash"

if [[ ! -f "${BAG_DIR}/metadata.yaml" ]]; then
    echo "[process] ERROR: rosbag metadata not found: ${BAG_DIR}/metadata.yaml"
    exit 2
fi

# Every guidance pattern enters through the existing FormationMode lifecycle.
# The common evaluation epoch is the latest of the five activation times.
evaluation_start_ns=0
for vehicle in 0 1 2 3 4; do
    timestamp_s=$(grep -m1 "\[Formation\] 활성화" \
        "${LOG_DIR}/guidance_${vehicle}.log" 2>/dev/null \
        | sed -n 's/^\[[^]]*\] \[\([0-9][0-9]*\.[0-9][0-9]*\)\].*/\1/p')
    if [[ -z "${timestamp_s}" ]]; then
        echo "[process] ERROR: vehicle ${vehicle} has no Formation activation timestamp"
        exit 2
    fi
    seconds=${timestamp_s%.*}
    fraction=${timestamp_s#*.}000000000
    fraction=${fraction:0:9}
    timestamp_ns=$((10#${seconds} * 1000000000 + 10#${fraction}))
    if (( timestamp_ns > evaluation_start_ns )); then
        evaluation_start_ns=${timestamp_ns}
    fi
done

ANALYSIS_ARGS=(
    --bag "${BAG_DIR}" \
    --summary-dir "${RESULT_ROOT}/summary/${RUN_ID}" \
    --plot-dir "${RESULT_ROOT}/plot/${RUN_ID}" \
    --video-dir "${RESULT_ROOT}/video/${RUN_ID}" \
    --log-dir "${LOG_DIR}" \
    --evaluation-start-ns "${evaluation_start_ns}" \
    --scenario-label "${SCENARIO_LABEL}" \
    --target-norths "${TARGET_NORTHS[@]}" \
    --target-easts "${TARGET_EASTS[@]}" \
    --desired-separation-distance 10.0
)
if [[ "${SHOW_ASSIGNED_TARGETS}" == "true" ]]; then
    ANALYSIS_ARGS+=(--show-targets)
fi

exec "${ANALYSIS_PYTHON}" \
    "${HILS_ROOT}/analysis/analyze_point_convergence_bag.py" \
    "${ANALYSIS_ARGS[@]}"
