#!/usr/bin/env bash

# Run the production FlockingGuidance inside the same five-aircraft HILS and
# distributed maneuver-selection/avoidance pipeline.
set -o pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export TRAFFIC_PATTERN=flocking
MODE=${1:-avoidance}
exec "${SCRIPT_DIR}/run_point_convergence_case.sh" "${MODE}"
