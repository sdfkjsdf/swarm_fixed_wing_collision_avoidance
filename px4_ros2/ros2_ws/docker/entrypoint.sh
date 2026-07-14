#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

VEHICLE_ID=${VEHICLE_ID:-0}
TOTAL_AGENTS=${TOTAL_AGENTS:-5}

echo "=== collision_avoidance container ==="
echo "  VEHICLE_ID:   ${VEHICLE_ID}"
echo "  TOTAL_AGENTS: ${TOTAL_AGENTS}"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
echo ""
echo "  노드 실행 명령어:"
echo "  ros2 run collision_avoidance vtol_guidance_node --ros-args --params-file /ros2_ws/install/collision_avoidance/share/collision_avoidance/config/flocking_params.yaml -p vehicle_ID:=${VEHICLE_ID} -p total_agent_num:=${TOTAL_AGENTS} -r __node:=vtol_guidance_${VEHICLE_ID}"
echo ""

exec bash
