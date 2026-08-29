#!/usr/bin/env bash

set -o pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
HILS_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
ROS2_WS=${ROS2_WS:-/home/hmcl/workspace/swarm-fixed-wing/ros2_ws}
COLLISION_WS=${COLLISION_WS:-$(cd "${HILS_ROOT}/../../.." && pwd)}
ANALYSIS_PYTHON=${ANALYSIS_PYTHON:-/home/hmcl/workspace/swarm-fixed-wing/.envs/px4/bin/python3}
RUN_ID=${1:?Usage: process_point_convergence_bag.sh RUN_ID}
RESULT_ROOT=${RESULT_ROOT:-${HILS_ROOT}/result}
BAG_DIR=${RESULT_ROOT}/rosbag/${RUN_ID}
LOG_DIR=${RESULT_ROOT}/log/${RUN_ID}

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

# Each guidance node enters the point-convergence test through the existing
# FormationMode lifecycle. The common evaluation epoch is the latest of the
# five activation times, so every aircraft is executing point_convergence.
evaluation_start_ns=0
for vehicle in 0 1 2 3 4; do
    timestamp_s=$(grep -m1 "\[Formation\] 활성화" \
        "${LOG_DIR}/guidance_${vehicle}.log" 2>/dev/null \
        | sed -n 's/^\[[^]]*\] \[\([0-9][0-9]*\.[0-9][0-9]*\)\].*/\1/p')
    if [[ -z "${timestamp_s}" ]]; then
        echo "[process] ERROR: vehicle ${vehicle} has no point-convergence activation timestamp"
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

exec "${ANALYSIS_PYTHON}" \
    "${HILS_ROOT}/analysis/analyze_point_convergence_bag.py" \
    --bag "${BAG_DIR}" \
    --summary-dir "${RESULT_ROOT}/summary/${RUN_ID}" \
    --plot-dir "${RESULT_ROOT}/plot/${RUN_ID}" \
    --video-dir "${RESULT_ROOT}/video/${RUN_ID}" \
    --log-dir "${LOG_DIR}" \
    --evaluation-start-ns "${evaluation_start_ns}" \
    --desired-separation-distance 10.0
