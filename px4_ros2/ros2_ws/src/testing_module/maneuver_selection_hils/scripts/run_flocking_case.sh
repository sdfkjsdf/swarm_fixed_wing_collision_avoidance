#!/usr/bin/env bash

# Run the production FlockingGuidance inside the same five-aircraft HILS and
# distributed maneuver-selection/avoidance pipeline.
set -o pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# Keep the provisional formation profile confined to the explicitly selected
# flocking HILS case. Point-convergence/crossing runs keep their own policy.
AMAC_POLICY_CONFIG=${AMAC_POLICY_CONFIG:-${SCRIPT_DIR}/../config/amac_flocking_formation.yaml}
export AMAC_POLICY_CONFIG
FLOCKING_LAYOUT=${FLOCKING_LAYOUT:-line}
if [[ "${FLOCKING_LAYOUT}" == "line" ]]; then
    export TRAFFIC_PATTERN=flocking
elif [[ "${FLOCKING_LAYOUT}" == "pentagon" ]]; then
    export TRAFFIC_PATTERN=flocking_pentagon
else
    echo "FLOCKING_LAYOUT must be line or pentagon"
    exit 2
fi
MODE=${1:-avoidance}
exec "${SCRIPT_DIR}/run_point_convergence_case.sh" "${MODE}"
