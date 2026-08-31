#!/usr/bin/env bash

# Run the two control policies as separate headless SILS cases. The AMAC
# comparison threshold defaults to the agreed DSD-matched 10 m experimental
# reserve.  The production AMAC default remains the disclosed AD=0 baseline.
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
AMAC_THRESHOLD_M=${1:-10.0}
BASE_RUN_ID=${BASE_RUN_ID:-execution_policy_$(date +%Y%m%d_%H%M%S)}

if ! awk -v value="${AMAC_THRESHOLD_M}" \
        'BEGIN { exit !(value + 0 == value && value > 0) }'; then
    echo "AMAC threshold must be a finite positive number in metres"
    exit 2
fi
THRESHOLD_TAG=${AMAC_THRESHOLD_M//./p}

echo "[compare] A: early AMAC, AD < ${AMAC_THRESHOLD_M} m"
RUN_ID="${BASE_RUN_ID}_amac_ad_${THRESHOLD_TAG}m" \
V4_MODE=shadow \
AVOIDANCE_EXECUTION_POLICY=amac_ad_threshold \
AMAC_AD_THRESHOLD_M="${AMAC_THRESHOLD_M}" \
    "${SCRIPT_DIR}/run_point_convergence_case.sh" avoidance

echo "[compare] B: horizon-gated V4 cone-barrier control"
RUN_ID="${BASE_RUN_ID}_horizon_gated_v4" \
V4_MODE=cutover \
AVOIDANCE_EXECUTION_POLICY=horizon_gated_v4 \
AMAC_AD_THRESHOLD_M=0.0 \
    "${SCRIPT_DIR}/run_point_convergence_case.sh" avoidance

echo "[compare] completed: ${BASE_RUN_ID}"
