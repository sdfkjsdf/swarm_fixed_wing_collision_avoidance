#!/usr/bin/env bash

# Reuse the five-aircraft HILS pipeline with per-aircraft destinations on the
# opposite edge of the initial pentagon instead of one common target.
set -o pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export TRAFFIC_PATTERN=opposite_edge_crossing
MODE=${1:-avoidance}
exec "${SCRIPT_DIR}/run_point_convergence_case.sh" "${MODE}"
