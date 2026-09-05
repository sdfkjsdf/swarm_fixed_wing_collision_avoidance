#!/usr/bin/env bash

# Run one installed guidance worker in a Docker container on a remote host.
# Stdout/stderr stay attached to SSH so the local HILS runner records the same
# guidance_<id>.log format used by an all-local run.
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: launch_remote_guidance_container.sh VEHICLE_ID [TOTAL_AGENT_NUM]" >&2
    exit 2
fi
: "${REMOTE_SSH_TARGET:?Set REMOTE_SSH_TARGET, for example ubuntu@192.168.1.50}"

VEHICLE_ID=$1
TOTAL_AGENT_NUM=${2:-5}
REMOTE_DOCKER_IMAGE=${REMOTE_DOCKER_IMAGE:-collision-avoidance:distributed}
REMOTE_CONTAINER_NAME=${REMOTE_CONTAINER_NAME:-collision-avoidance-hils-agent-${VEHICLE_ID}}

cleanup_done=0
cleanup() {
    if (( cleanup_done == 1 )); then
        return
    fi
    cleanup_done=1
    ssh -T "${REMOTE_SSH_TARGET}" \
        docker rm -f "${REMOTE_CONTAINER_NAME}" >/dev/null 2>&1 || true
}
trap 'cleanup; exit 143' INT TERM
trap cleanup EXIT

ssh -T "${REMOTE_SSH_TARGET}" \
    docker rm -f "${REMOTE_CONTAINER_NAME}" >/dev/null 2>&1 || true

docker_args=(
    docker run --rm
    --name "${REMOTE_CONTAINER_NAME}"
    --network host
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
    -e "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}"
    -e "VEHICLE_ID=${VEHICLE_ID}"
    -e "TOTAL_AGENT_NUM=${TOTAL_AGENT_NUM}"
)
for variable in \
    GUIDANCE_CONFIG_NAME AMAC_POLICY_CONFIG_NAME GUIDANCE_MODE \
    POINT_TARGET_NORTH_M POINT_TARGET_EAST_M \
    PREFLIGHT_DESIRED_COURSE_RAD PREFLIGHT_DESIRED_GROUND_SPEED_MPS \
    COLLISION_AVOIDANCE_SHADOW_ONLY AVOIDANCE_EXECUTION_POLICY \
    AMAC_COMMUNICATION_DELAY_MARGIN_M AMAC_INTERACTION_GRAPH_ENABLED \
    AMAC_INTERACTION_GRAPH_AD_SCREEN_M AMAC_TRAJECTORY_LIBRARY_VERSION \
    AMAC_AD_MASD_CONFIG_VERSION MANEUVER_SELECTION_EXHAUSTIVE_TEST_MODE \
    V4_SAFE_CONTROL_ENABLED V4_SHADOW_ONLY V4_CONTROL_ARCHITECTURE \
    POSITIVE_MARGIN_FILTER_ENABLED; do
    docker_args+=(-e "${variable}=${!variable:-}")
done
if [[ -n "${RMW_IMPLEMENTATION:-}" ]]; then
    docker_args+=(-e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}")
fi
if [[ -n "${ROS_DISCOVERY_SERVER:-}" ]]; then
    docker_args+=(-e "ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER}")
fi
docker_args+=(
    --entrypoint /bin/bash
    "${REMOTE_DOCKER_IMAGE}"
    -lc
    'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec ros2 run collision_avoidance run_guidance_vehicle.sh "${VEHICLE_ID}" "${TOTAL_AGENT_NUM}"'
)

printf -v remote_command '%q ' "${docker_args[@]}"
ssh -T "${REMOTE_SSH_TARGET}" "${remote_command}"
