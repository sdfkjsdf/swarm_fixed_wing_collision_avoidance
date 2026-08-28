#!/usr/bin/env bash

set -o pipefail

OUTPUT_DIR=${1:?Usage: record_point_convergence_bag.sh OUTPUT_DIR}
TOPICS=()
for vehicle in 0 1 2 3 4; do
    TOPICS+=(
        "/common/px4_${vehicle}/trans_vehicle_odometry"
        "/common/px4_${vehicle}/maneuver_selection_decision"
        "/common/px4_${vehicle}/trajectory_intent"
        "/px4_${vehicle}/fmu/out/vehicle_attitude"
        "/px4_${vehicle}/fmu/out/vtol_vehicle_status"
    )
done

exec ros2 bag record \
    --storage sqlite3 \
    --output "${OUTPUT_DIR}" \
    --max-cache-size 104857600 \
    "${TOPICS[@]}"
