#!/usr/bin/env bash

# Hybrid distributed HILS:
#   Raspberry Pi: vehicle 0 guidance/selection worker
#   Local PC:     vehicles 1..4, five PX4 SITL instances, common transformer,
#                 MicroXRCEAgents, rosbag, and offline analysis
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
: "${REMOTE_SSH_TARGET:?Set REMOTE_SSH_TARGET, for example ubuntu@192.168.1.50}"

# Keep the base ROS/PX4 workspace as the dependency underlay, but force the
# guidance binary, configuration, and test scripts to come from this worktree.
# This prevents a hybrid run from silently picking up the main checkout's
# collision_avoidance install when the wrapper is launched from a worktree.
DISTRIBUTED_ROS2_WS=$(cd "${SCRIPT_DIR}/../../../.." && pwd)
export COLLISION_WS=${COLLISION_WS:-${DISTRIBUTED_ROS2_WS}}
export FORMATION_HILS=${FORMATION_HILS:-${DISTRIBUTED_ROS2_WS}/src/testing_module/formation_hils}
export ROS_LOCALHOST_ONLY=0

export LOCAL_GUIDANCE_IDS_CSV=1,2,3,4
export REMOTE_GUIDANCE_ID=0
export REMOTE_GUIDANCE_LAUNCHER="${SCRIPT_DIR}/launch_remote_guidance_container.sh"
export FLOCKING_LAYOUT=${FLOCKING_LAYOUT:-pentagon}

exec "${SCRIPT_DIR}/run_flocking_case.sh" "${1:-avoidance}"
