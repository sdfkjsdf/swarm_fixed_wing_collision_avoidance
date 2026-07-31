#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════════
# run_all_cases.sh — Stage A-3 (claude_code_task_spec.md §A-3-1)
# ────────────────────────────────────────────────────────────────────
# case_matrix.yaml 의 25 case 를 순차 실행 + CSV 수집.
#
# 각 case 마다:
#   1. SEQUENCE_FILE 환경변수 = cases/{ID}.yaml 로 launch_1vtol_replay.sh 호출
#   2. launch_1vtol_replay.sh 가 SITL + node 자동 부팅 → Replay 끝나면 자동 shutdown
#   3. /tmp/trajectory_*.csv → results/cases/{ID}_<timestamp>.csv 로 이동
#   4. 다음 case
#
# 사용법:
#   ./run_all_cases.sh                    # 모든 25 case
#   ./run_all_cases.sh --phase 1          # Phase 1 (12 case) 만
#   ./run_all_cases.sh --phase 2,3        # Phase 2 + Phase 3
#   ./run_all_cases.sh --cases V01,V02    # 특정 case 만
#
# 시간 예상: 약 4~5 분/case (SITL 부팅 + RTL + Disarm 포함) → 전체 약 100~125 분.
# ════════════════════════════════════════════════════════════════════

ROS2_WS=${ROS2_WS:-/home/hmcl/workspace/swarm-fixed-wing/ros2_ws}
HILS_SRC=${HILS_SRC:-${ROS2_WS}/src/testing_module/trajectory_prediction_hils}
ANALYSIS_DIR=${ANALYSIS_DIR:-${ROS2_WS}/src/testing_module/analysis_tools}
ANALYSIS_PYTHON=${ANALYSIS_PYTHON:-python3}
CASE_DIR="${HILS_SRC}/config/cases"
RESULTS_DIR="${ROS2_WS}/results/cases"
LAUNCH_SCRIPT="${HILS_SRC}/scripts/launch_1vtol_replay.sh"

mkdir -p "${RESULTS_DIR}"

# ─── case list (case_matrix.yaml 과 동기화) ────────────────────────
# ★ 사용자 의도 (2026-05-13): roll 채널 (R 시리즈) 만 test. 다른 시리즈는 yaml 보존 + 배열 비활성.
PHASE_1_CASES=()   # ★ 비활성 — V/H/P 시리즈 사용 안 함 (yaml 은 cases/ 안 보존)
PHASE_2_CASES=()
PHASE_3_CASES=()
PHASE_4_CASES=()
PHASE_LH_CASES=()  # ★ 비활성 — R 시리즈와 기능 중복 (a_lat 명령 표기 차이만)
PHASE_R_CASES=(R05P R05M R15P R15M R25P R25M)   # ★ 활성 — roll setpoint sweep

# ─── 인자 처리 ─────────────────────────────────────────────────────
SELECTED_CASES=()

if [[ "${1:-}" == "--phase" ]]; then
    IFS=',' read -r -a PHASES <<< "${2:-}"
    for P in "${PHASES[@]}"; do
        case "${P}" in
            1)  SELECTED_CASES+=("${PHASE_1_CASES[@]}") ;;
            2)  SELECTED_CASES+=("${PHASE_2_CASES[@]}") ;;
            3)  SELECTED_CASES+=("${PHASE_3_CASES[@]}") ;;
            4)  SELECTED_CASES+=("${PHASE_4_CASES[@]}") ;;
            LH) SELECTED_CASES+=("${PHASE_LH_CASES[@]}") ;;
            R)  SELECTED_CASES+=("${PHASE_R_CASES[@]}")  ;;
            all)
                SELECTED_CASES+=("${PHASE_1_CASES[@]}" "${PHASE_2_CASES[@]}" \
                                 "${PHASE_3_CASES[@]}" "${PHASE_4_CASES[@]}" \
                                 "${PHASE_LH_CASES[@]}" "${PHASE_R_CASES[@]}")
                ;;
            *) echo "[runner] ERROR: unknown phase '${P}' (use 1|2|3|4|LH|R|all)"; exit 1 ;;
        esac
    done
elif [[ "${1:-}" == "--cases" ]]; then
    IFS=',' read -r -a SELECTED_CASES <<< "${2:-}"
elif [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^#=*$/p' "$0" | sed 's/^# *//'
    exit 0
else
    SELECTED_CASES=("${PHASE_1_CASES[@]}" "${PHASE_2_CASES[@]}" \
                    "${PHASE_3_CASES[@]}" "${PHASE_4_CASES[@]}")
fi

# ─── 출력 ──────────────────────────────────────────────────────────
N_CASES=${#SELECTED_CASES[@]}
echo "═══════════════════════════════════════════════════════════════"
echo "[runner] 실행 ${N_CASES} case → ${RESULTS_DIR}/"
echo "[runner] case: ${SELECTED_CASES[*]}"
echo "[runner] 예상 시간: 약 $((N_CASES * 4)) 분 (case 당 ~4 분)"
echo "═══════════════════════════════════════════════════════════════"

# ─── 실행 루프 ─────────────────────────────────────────────────────
SUCCESS=()
FAILED=()
T_START=$(date +%s)

for IDX in "${!SELECTED_CASES[@]}"; do
    CASE_ID="${SELECTED_CASES[$IDX]}"
    CASE_FILE="${CASE_DIR}/${CASE_ID}.yaml"

    if [[ ! -f "${CASE_FILE}" ]]; then
        echo "[runner] WARN: ${CASE_FILE} 없음 — skip"
        FAILED+=("${CASE_ID}(missing)")
        continue
    fi

    NOW=$(date +%H:%M:%S)
    echo ""
    echo "═══════════════════════════════════════════════════════════════"
    echo "[runner] [$((IDX+1))/${N_CASES}] ${CASE_ID}  (${NOW})"
    echo "[runner] yaml: ${CASE_FILE}"
    echo "═══════════════════════════════════════════════════════════════"

    # launch_1vtol_replay.sh 호출 (env 로 case yaml 전달, chunk_analysis 스킵).
    # launch script 가 끝나면 (RTL+Disarm+shutdown) 자동 종료.
    SEQUENCE_FILE="${CASE_FILE}" \
    SKIP_CHUNK_ANALYSIS=1 \
        bash "${LAUNCH_SCRIPT}"
    LAUNCH_RC=$?

    # 가장 최근 CSV 를 results 로 이동.
    # ★ 2026-05-20: spline CSV (/tmp/trajectory_spline_*.csv) 분리 이동.
    #   predict CSV 패턴: /tmp/trajectory_2*.csv  (timestamp 가 '2' 로 시작 — 2026 이후)
    #   spline  CSV 패턴: /tmp/trajectory_spline_*.csv
    LATEST_CSV=$(ls -t /tmp/trajectory_2*.csv 2>/dev/null | head -1)
    LATEST_SPLINE_CSV=$(ls -t /tmp/trajectory_spline_*.csv 2>/dev/null | head -1)
    if [[ -n "${LATEST_CSV}" ]]; then
        STAMP=$(date +%Y%m%d_%H%M%S)
        TARGET="${RESULTS_DIR}/${CASE_ID}_${STAMP}.csv"
        mv "${LATEST_CSV}" "${TARGET}"
        echo "[runner] CSV → ${TARGET}"
        # ★ spline CSV 도 동일 STAMP 로 짝 이동 (predict 와 timestamp 일치).
        if [[ -n "${LATEST_SPLINE_CSV}" ]]; then
            SPLINE_TARGET="${RESULTS_DIR}/${CASE_ID}_spline_${STAMP}.csv"
            mv "${LATEST_SPLINE_CSV}" "${SPLINE_TARGET}"
            echo "[runner] spline CSV → ${SPLINE_TARGET}"
        else
            echo "[runner] WARN: spline CSV 안 생성 — reconstruct 통합 실패?"
        fi
        SUCCESS+=("${CASE_ID}")
    else
        echo "[runner] ERROR: case ${CASE_ID} 의 CSV 안 생성 (rc=${LAUNCH_RC})"
        FAILED+=("${CASE_ID}")
    fi

    # 다음 case 전에 SITL 잔여 프로세스 확실히 정리 (launch trap 이 처리하지만 안전망)
    pkill -KILL -f "trajectory_replay_node" 2>/dev/null
    pkill -KILL -f "px4 -i 0" 2>/dev/null
    pkill -KILL -f "gzserver Tools/simulation" 2>/dev/null
    pkill -KILL -f "MicroXRCEAgent udp4 -p 8888" 2>/dev/null
    sleep 2
done

# ─── 요약 보고 ─────────────────────────────────────────────────────
T_END=$(date +%s)
ELAPSED=$(( (T_END - T_START) / 60 ))
echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "[runner] 완료 (소요: ${ELAPSED} 분)"
echo "[runner] 성공: ${#SUCCESS[@]} — ${SUCCESS[*]}"
echo "[runner] 실패: ${#FAILED[@]} — ${FAILED[*]}"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "[runner] 분석 명령:"
echo "  ${ANALYSIS_PYTHON} \\"
echo "      ${ANALYSIS_DIR}/analyze_cases.py \\"
echo "      --results ${RESULTS_DIR}/ \\"
echo "      --matrix ${HILS_SRC}/config/case_matrix.yaml \\"
echo "      --out-dir ${ROS2_WS}/results/"

if [[ ${#FAILED[@]} -gt 0 ]]; then
    exit 1
fi
