#!/usr/bin/env bash
# Process an already-recorded trajectory-cone batch without starting PX4/Gazebo.

# ROS setup scripts legitimately inspect optional environment variables, so nounset is avoided.
set -o pipefail

ROS2_WS=${ROS2_WS:-/home/hmcl/workspace/swarm-fixed-wing/ros2_ws}
HILS_SRC=${HILS_SRC:-${ROS2_WS}/src/testing_module/trajectory_prediction_hils}
RESULT_ROOT=${RESULT_ROOT:-${HILS_SRC}/result}
ANALYSIS_PYTHON=${ANALYSIS_PYTHON:-/home/hmcl/workspace/swarm-fixed-wing/.envs/px4/bin/python3}
NAMESPACE=/px4_0
BATCH_ID=""
PROPAGATION_TEST=0

usage() {
    echo "Usage: $0 --batch-id ID [--result-root PATH] [--namespace /px4_0]"
    echo "          [--propagation-test]"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --batch-id) BATCH_ID=${2:-}; shift 2 ;;
        --result-root) RESULT_ROOT=${2:-}; shift 2 ;;
        --namespace) NAMESPACE=${2:-}; shift 2 ;;
        --propagation-test) PROPAGATION_TEST=1; shift ;;
        --help|-h) usage; exit 0 ;;
        *) echo "[process] ERROR: unknown argument '$1'"; usage; exit 2 ;;
    esac
done

if [[ -z "${BATCH_ID}" ]]; then
    echo "[process] ERROR: --batch-id is required"
    usage
    exit 2
fi
if [[ ! -x "${ANALYSIS_PYTHON}" ]]; then
    ANALYSIS_PYTHON=python3
fi

MANIFEST=${RESULT_ROOT}/summary/${BATCH_ID}/scenario_manifest.tsv
ROSBAG_ROOT=${RESULT_ROOT}/rosbag/${BATCH_ID}
RAW_ROOT=${RESULT_ROOT}/raw/${BATCH_ID}
PLOT_ROOT=${RESULT_ROOT}/plot/${BATCH_ID}
SUMMARY_ROOT=${RESULT_ROOT}/summary/${BATCH_ID}
LOG_ROOT=${RESULT_ROOT}/log/${BATCH_ID}
STATUS_FILE=${SUMMARY_ROOT}/processing_status.tsv

if [[ ! -f "${MANIFEST}" ]]; then
    echo "[process] ERROR: scenario manifest not found: ${MANIFEST}"
    exit 2
fi

mkdir -p "${RAW_ROOT}" "${PLOT_ROOT}" "${SUMMARY_ROOT}" "${LOG_ROOT}"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash >/dev/null 2>&1
if [[ -f "${ROS2_WS}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${ROS2_WS}/install/setup.bash" >/dev/null 2>&1
fi

ANALYZER=${HILS_SRC}/analysis/analyze_trajectory_cone_bag.py
PLOTTER=${HILS_SRC}/analysis/plot_trajectory_cone_results.py
SUMMARIZER=${HILS_SRC}/analysis/summarize_cone_batch.py
for required in "${ANALYZER}" "${PLOTTER}" "${SUMMARIZER}"; do
    if [[ ! -f "${required}" ]]; then
        echo "[process] ERROR: required analysis script not found: ${required}"
        exit 2
    fi
done

EXPECTED_RUNS=$(awk 'NR > 1 {count++} END {print count + 0}' "${MANIFEST}")
printf 'run_id\tanalysis_rc\tplot_rc\n' > "${STATUS_FILE}"
FAILED=0
PROCESSED=0

ANALYSIS_ARGS=()
if [[ ${PROPAGATION_TEST} -eq 1 ]]; then
    ANALYSIS_ARGS+=(--require-alignment --require-propagation-contract)
fi

while IFS=$'\t' read -r run_id _manifest_remainder; do
    [[ "${run_id}" == "run_id" ]] && continue
    [[ -z "${run_id}" ]] && continue
    bag_dir=${ROSBAG_ROOT}/${run_id}
    raw_dir=${RAW_ROOT}/${run_id}
    plot_dir=${PLOT_ROOT}/${run_id}
    process_log=${LOG_ROOT}/${run_id}_process.log

    echo "[process] ${run_id}: rosbag offline analysis"
    if [[ ! -f "${bag_dir}/metadata.yaml" ]]; then
        echo "[process] WARN: missing rosbag metadata: ${bag_dir}" | tee "${process_log}"
        printf '%s\t%s\t%s\n' "${run_id}" 2 2 >> "${STATUS_FILE}"
        FAILED=$((FAILED + 1))
        continue
    fi

    "${ANALYSIS_PYTHON}" "${ANALYZER}" "${bag_dir}" \
        --output "${raw_dir}" --namespace "${NAMESPACE}" \
        "${ANALYSIS_ARGS[@]}" \
        > "${process_log}" 2>&1
    analysis_rc=$?
    plot_rc=2
    # 구조 검사가 실패해도 분석 산출물이 있으면 진단 plot은 남긴다.
    if [[ ${analysis_rc} -le 1 \
          && -f "${raw_dir}/summary.json" \
          && -f "${raw_dir}/cone_samples.csv" \
          && -f "${raw_dir}/cone_arrays.npz" ]]; then
        "${ANALYSIS_PYTHON}" "${PLOTTER}" \
            --input "${raw_dir}" --output "${plot_dir}" --title "${run_id}" \
            >> "${process_log}" 2>&1
        plot_rc=$?
    fi
    printf '%s\t%s\t%s\n' "${run_id}" "${analysis_rc}" "${plot_rc}" >> "${STATUS_FILE}"
    if [[ ${analysis_rc} -ne 0 || ${plot_rc} -ne 0 ]]; then
        echo "[process] WARN: ${run_id} failed (analysis=${analysis_rc}, plot=${plot_rc})"
        FAILED=$((FAILED + 1))
    else
        PROCESSED=$((PROCESSED + 1))
    fi
done < "${MANIFEST}"

echo "[process] aggregate summary (${PROCESSED}/${EXPECTED_RUNS} processed)"
"${ANALYSIS_PYTHON}" "${SUMMARIZER}" \
    --raw-root "${RAW_ROOT}" --output "${SUMMARY_ROOT}" \
    --expected-runs "${EXPECTED_RUNS}" \
    > "${LOG_ROOT}/batch_summary.log" 2>&1
summary_rc=$?
if [[ ${summary_rc} -ne 0 ]]; then
    FAILED=$((FAILED + 1))
fi

echo "[process] summary: ${SUMMARY_ROOT}/batch_summary.json"
echo "[process] plots:   ${PLOT_ROOT}"
if [[ ${FAILED} -ne 0 ]]; then
    echo "[process] completed with ${FAILED} failure(s)"
    exit 1
fi
echo "[process] completed successfully"
