#!/usr/bin/env bash
# Collect FixedWing ZOH propagation cases, then analyze the bags offline.

# ROS/PX4 setup scripts inspect optional variables, so nounset is avoided.
set -o pipefail

ROS2_WS=${ROS2_WS:-/home/hmcl/workspace/swarm-fixed-wing/ros2_ws}
HILS_SRC=${HILS_SRC:-${ROS2_WS}/src/testing_module/trajectory_prediction_hils}
RESULT_ROOT=${RESULT_ROOT:-${HILS_SRC}/result}
MATRIX=${MATRIX:-${HILS_SRC}/config/propagation_scenario_matrix.yaml}
ANALYSIS_PYTHON=${ANALYSIS_PYTHON:-/home/hmcl/workspace/swarm-fixed-wing/.envs/px4/bin/python3}
PROFILE=wiring_smoke
BATCH_ID=""
DRY_RUN=0
COLLECT_ONLY=0
LIST_PROFILES=0

usage() {
    echo "Usage: $0 [--profile NAME] [--batch-id ID] [--dry-run] [--collect-only]"
    echo "          [--matrix PATH] [--result-root PATH] [--list-profiles]"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile) PROFILE=${2:-}; shift 2 ;;
        --batch-id|--run-id) BATCH_ID=${2:-}; shift 2 ;;
        --matrix) MATRIX=${2:-}; shift 2 ;;
        --result-root) RESULT_ROOT=${2:-}; shift 2 ;;
        --dry-run) DRY_RUN=1; shift ;;
        --collect-only) COLLECT_ONLY=1; shift ;;
        --list-profiles) LIST_PROFILES=1; shift ;;
        --help|-h) usage; exit 0 ;;
        *) echo "[propagation-batch] ERROR: unknown argument '$1'"; usage; exit 2 ;;
    esac
done

if [[ ! -x "${ANALYSIS_PYTHON}" ]]; then
    ANALYSIS_PYTHON=python3
fi
RESOLVER=${HILS_SRC}/analysis/expand_propagation_scenarios.py
if [[ ${LIST_PROFILES} -eq 1 ]]; then
    "${ANALYSIS_PYTHON}" "${RESOLVER}" --matrix "${MATRIX}" \
        --profile "${PROFILE}" --list-profiles
    exit $?
fi

TEMP_DIR=$(mktemp -d /tmp/trajectory_propagation_manifest.XXXXXX)
trap 'rm -rf -- "${TEMP_DIR}"' EXIT
TEMP_MANIFEST=${TEMP_DIR}/scenario_manifest.tsv
if ! "${ANALYSIS_PYTHON}" "${RESOLVER}" --matrix "${MATRIX}" \
        --profile "${PROFILE}" > "${TEMP_MANIFEST}"; then
    exit 2
fi
RUN_COUNT=$(awk 'NR > 1 {count++} END {print count + 0}' "${TEMP_MANIFEST}")

if [[ ${DRY_RUN} -eq 1 ]]; then
    echo "[propagation-batch] profile=${PROFILE}, planned_runs=${RUN_COUNT}"
    column -t -s $'\t' "${TEMP_MANIFEST}" 2>/dev/null \
        || sed -n '1,240p' "${TEMP_MANIFEST}"
    exit 0
fi

if [[ -z "${BATCH_ID}" ]]; then
    BATCH_ID=propagation_${PROFILE}_$(date +%Y%m%d_%H%M%S)
fi
ROSBAG_ROOT=${RESULT_ROOT}/rosbag/${BATCH_ID}
RAW_ROOT=${RESULT_ROOT}/raw/${BATCH_ID}
PLOT_ROOT=${RESULT_ROOT}/plot/${BATCH_ID}
SUMMARY_ROOT=${RESULT_ROOT}/summary/${BATCH_ID}
LOG_ROOT=${RESULT_ROOT}/log/${BATCH_ID}
for target in "${ROSBAG_ROOT}" "${RAW_ROOT}" "${PLOT_ROOT}" \
        "${SUMMARY_ROOT}" "${LOG_ROOT}"; do
    if [[ -e "${target}" ]]; then
        echo "[propagation-batch] ERROR: batch output already exists: ${target}"
        exit 2
    fi
done
mkdir -p "${ROSBAG_ROOT}" "${RAW_ROOT}" "${PLOT_ROOT}" \
    "${SUMMARY_ROOT}" "${LOG_ROOT}"
cp "${TEMP_MANIFEST}" "${SUMMARY_ROOT}/scenario_manifest.tsv"
printf 'run_id\tlaunch_rc\tbag_recorded\n' \
    > "${SUMMARY_ROOT}/collection_status.tsv"

echo "[propagation-batch] ${PROFILE}: ${RUN_COUNT} run(s), batch=${BATCH_ID}"
FAILED=0
while IFS=$'\t' read -r run_id case_id dataset repetition gazebo_seed \
        trim_speed_mps lateral_acceleration timeout_s pause_s; do
    [[ "${run_id}" == "run_id" ]] && continue
    [[ -z "${run_id}" ]] && continue
    bag_dir=${ROSBAG_ROOT}/${run_id}
    launch_log=${LOG_ROOT}/${run_id}_launch.log
    echo "[propagation-batch] collect ${run_id} "\
"(dataset=${dataset}, seed=${gazebo_seed}, trim_V=${trim_speed_mps}, "\
"h_dot=0, a_lat=${lateral_acceleration})"

    SKIP_ROSBAG_ANALYSIS=1 \
    SKIP_CHUNK_ANALYSIS=1 \
    HILS_RUN_ID=${run_id} \
    BAG_OUTPUT_DIR=${bag_dir} \
    GAZEBO_SEED=${gazebo_seed} \
    TEST_CASE_ID=${case_id} \
    TEST_V_CMD=${trim_speed_mps} \
    TEST_ALAT_CMD=${lateral_acceleration} \
    timeout --signal=INT --kill-after=25s "${timeout_s}s" \
        bash "${HILS_SRC}/scripts/launch_propagation_test.sh" --record-bag \
        > "${launch_log}" 2>&1
    launch_rc=$?
    bag_recorded=0
    if [[ -f "${bag_dir}/metadata.yaml" ]]; then
        bag_recorded=1
    fi
    printf '%s\t%s\t%s\n' "${run_id}" "${launch_rc}" "${bag_recorded}" \
        >> "${SUMMARY_ROOT}/collection_status.tsv"
    if [[ ${launch_rc} -ne 0 || ${bag_recorded} -ne 1 ]]; then
        echo "[propagation-batch] WARN: ${run_id} collection failed "\
"(launch=${launch_rc}, bag=${bag_recorded})"
        FAILED=$((FAILED + 1))
    fi
    if [[ ${pause_s} -gt 0 ]]; then
        sleep "${pause_s}"
    fi
done < "${SUMMARY_ROOT}/scenario_manifest.tsv"

echo "[propagation-batch] collection complete; failure count=${FAILED}"
if [[ ${COLLECT_ONLY} -eq 0 ]]; then
    echo "[propagation-batch] starting offline rosbag processing"
    RESULT_ROOT=${RESULT_ROOT} ANALYSIS_PYTHON=${ANALYSIS_PYTHON} \
        bash "${HILS_SRC}/scripts/process_cone_batch.sh" \
        --batch-id "${BATCH_ID}" --propagation-test
    process_rc=$?
    if [[ ${process_rc} -ne 0 ]]; then
        FAILED=$((FAILED + 1))
    fi
else
    echo "[propagation-batch] collect-only selected. Process later with:"
    echo "  ${HILS_SRC}/scripts/process_cone_batch.sh "\
"--batch-id ${BATCH_ID} --propagation-test"
fi

echo "[propagation-batch] result root: ${RESULT_ROOT}"
if [[ ${FAILED} -ne 0 ]]; then
    exit 1
fi
