#!/usr/bin/env bash

# Run the two control policies as separate headless SILS cases. The AMAC case
# uses the strict AD < 0 baseline; DSD remains inside MASD.
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
BASE_RUN_ID=${BASE_RUN_ID:-execution_policy_$(date +%Y%m%d_%H%M%S)}

echo "[compare] A: baseline AMAC, AD < 0 m"
RUN_ID="${BASE_RUN_ID}_amac_ad_0m" \
MANEUVER_SEARCH_MODE=exhaustive \
V4_MODE=shadow \
V4_CONTROL_ARCHITECTURE=legacy_safe_control_set \
AVOIDANCE_EXECUTION_POLICY=amac_ad_threshold \
    "${SCRIPT_DIR}/run_point_convergence_case.sh" avoidance

echo "[compare] B: continuous closed-form Mode-B control"
RUN_ID="${BASE_RUN_ID}_mode_b_cbf" \
MANEUVER_SEARCH_MODE=heuristic \
V4_MODE=cutover \
V4_CONTROL_ARCHITECTURE=closed_form_backup_mode_b \
AVOIDANCE_EXECUTION_POLICY=continuous_v4 \
    "${SCRIPT_DIR}/run_point_convergence_case.sh" avoidance

echo "[compare] completed: ${BASE_RUN_ID}"
