#!/usr/bin/env bash

# Hybrid distributed HILS:
#   Raspberry Pi: vehicle 0 guidance/selection worker
#   Local PC:     vehicles 1..4, five PX4 SITL instances, common transformer,
#                 MicroXRCEAgents, rosbag, and offline analysis
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
: "${REMOTE_SSH_TARGET:?Set REMOTE_SSH_TARGET, for example ubuntu@192.168.1.50}"

# Test scripts come from this source checkout, while ROS interfaces and the
# production node come from the canonical built workspace. A worktree test can
# still select its own overlay by setting ROS2_WS/COLLISION_WS explicitly.
SOURCE_ROS2_WS=$(cd "${SCRIPT_DIR}/../../../.." && pwd)
export ROS2_WS=${ROS2_WS:-/home/hmcl/workspace/swarm-fixed-wing/ros2_ws}
export COLLISION_WS=${COLLISION_WS:-${ROS2_WS}}
export FORMATION_HILS=${FORMATION_HILS:-${SOURCE_ROS2_WS}/src/testing_module/formation_hils}
export ROS_LOCALHOST_ONLY=0

export LOCAL_GUIDANCE_IDS_CSV=1,2,3,4
export REMOTE_GUIDANCE_ID=0
export REMOTE_GUIDANCE_LAUNCHER="${SCRIPT_DIR}/launch_remote_guidance_container.sh"
export FLOCKING_LAYOUT=${FLOCKING_LAYOUT:-pentagon}

exec "${SCRIPT_DIR}/run_flocking_case.sh" "${1:-avoidance}"
