#!/usr/bin/env bash

set -e

OUTPUT_DIR=${1:-}
TOPIC_NS=${2:-/px4_0}
AGENT_NAME=${TOPIC_NS#/}
COMMON_NS=/common/${AGENT_NAME}

if [[ -z "${OUTPUT_DIR}" ]]; then
    echo "usage: $0 OUTPUT_DIR [TOPIC_NAMESPACE]" >&2
    exit 2
fi

if [[ -e "${OUTPUT_DIR}" ]]; then
    echo "ERROR: bag output already exists: ${OUTPUT_DIR}" >&2
    exit 2
fi

mkdir -p "$(dirname "${OUTPUT_DIR}")"

TOPICS=(
    "${TOPIC_NS}/fmu/out/estimator_trajectory_belief"
    "${TOPIC_NS}/fmu/out/vehicle_local_position_v1"
    "${TOPIC_NS}/fmu/out/vehicle_odometry"
    "${TOPIC_NS}/fmu/out/vehicle_gps_position"
    "${TOPIC_NS}/fmu/out/vehicle_attitude"
    "${TOPIC_NS}/fmu/out/vehicle_local_position_groundtruth_v1"
    "${TOPIC_NS}/fmu/out/vehicle_attitude_groundtruth"
    "${TOPIC_NS}/fmu/out/vehicle_status_v3"
    "${TOPIC_NS}/fmu/out/airspeed_validated_v1"
    "${TOPIC_NS}/fmu/out/wind"
    "${TOPIC_NS}/collision_estimation/trajectory_cone"
    "${TOPIC_NS}/collision_estimation/key_samples"
    "${COMMON_NS}/trans_estimator_trajectory_belief"
    "${COMMON_NS}/trans_vehicle_local_position"
    "${COMMON_NS}/trans_vehicle_odometry"
    "${COMMON_NS}/trans_vehicle_local_position_groundtruth"
    "${COMMON_NS}/trans_trajectory_cone"
)

echo "[rosbag] output=${OUTPUT_DIR} namespace=${TOPIC_NS} common=${COMMON_NS}"
exec ros2 bag record \
    --storage sqlite3 \
    --output "${OUTPUT_DIR}" \
    --max-cache-size 104857600 \
    "${TOPICS[@]}"
