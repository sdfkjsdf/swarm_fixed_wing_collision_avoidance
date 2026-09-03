#!/usr/bin/env bash

# Run the production FlockingGuidance inside the same five-aircraft HILS and
# distributed maneuver-selection/avoidance pipeline.
set -o pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# Keep the Formation HILS AMAC policy confined to this explicitly selected
# case. The policy intentionally contains no geometry-specific threat
# exemption, so line and pentagon layouts use the same AD activation rule.
AMAC_POLICY_CONFIG=${AMAC_POLICY_CONFIG:-${SCRIPT_DIR}/../config/amac_flocking_formation.yaml}
export AMAC_POLICY_CONFIG
# The shared point-convergence runner historically enabled the experimental
# positive-margin filter for every shadow run through a later CLI override.
# Keep the Lockheed/AMAC flocking comparison AD-only unless explicitly
# overridden by the caller.
POSITIVE_MARGIN_FILTER_ENABLED=${POSITIVE_MARGIN_FILTER_ENABLED:-false}
export POSITIVE_MARGIN_FILTER_ENABLED
# The distributed AMAC experiment does not run the unrelated V4/CBF shadow
# evaluator. CBF experiments use their dedicated worktree and runner.
V4_SAFE_CONTROL_ENABLED=${V4_SAFE_CONTROL_ENABLED:-false}
export V4_SAFE_CONTROL_ENABLED
# This worktree validates the distributed component graph as the primary AMAC
# proposal path. A caller may still disable it explicitly for baseline replay.
AMAC_INTERACTION_GRAPH_ENABLED=${AMAC_INTERACTION_GRAPH_ENABLED:-true}
export AMAC_INTERACTION_GRAPH_ENABLED
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
