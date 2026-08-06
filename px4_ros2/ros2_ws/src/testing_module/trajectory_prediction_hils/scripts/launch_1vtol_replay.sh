#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════════
# launch_1vtol_replay.sh — 1대 VTOL SITL + trajectory_replay_node 통합 부팅
# ────────────────────────────────────────────────────────────────────
# 단계 (모두 sleep 으로 직렬화, 각 sleep 후 ready 검증):
#   1. 정리            : 기존 px4/gzserver/MicroXRCEAgent/노드 강제 종료 + 락 정리
#   2. MicroXRCEAgent  : udp4 -p 8888 (백그라운드, /tmp/micro_xrce_agent.log)
#   3. gzserver        : empty.world headless (백그라운드, /tmp/gzserver.log)
#   4. PX4 instance 0  : multi-vehicle namespace (PX4_UXRCE_DDS_NS=px4_0) +
#                        standard_vtol airframe. simulator 연결 대기 상태로 기동.
#   5. SDF spawn       : standard_vtol_0 모델을 gzserver 에 추가
#                        → PX4 와 TCP 4560 연결 → uXRCE-DDS 토픽 발행 시작
#   6. ready check     : /px4_0/fmu/out/register_ext_component_reply_v1 토픽
#                        뜰 때까지 polling (max 30s)
#   7. (옵션) 노드 실행 : --no-node 면 SITL 만 띄우고 종료. 기본은 trajectory_replay_node 실행.
#
# Trap : Ctrl+C 또는 노드 종료 시 모든 백그라운드 프로세스 정리.
#
# 사용법:
#   $ launch_1vtol_replay.sh             # 전체 부팅 + 노드 실행 (foreground spin)
#   $ launch_1vtol_replay.sh --no-node   # SITL 만 띄우고 종료 (노드는 별도 셸에서)
#   $ launch_1vtol_replay.sh --record-bag # 필수 검증 토픽 rosbag 기록
#
# 반복 수집에서는 SKIP_ROSBAG_ANALYSIS=1 로 온라인 분석을 끄고, 모든 실행이 끝난 뒤
# process_cone_batch.sh 로 rosbag을 일괄 처리한다.
#
# 종료:
#   - 노드 spin 중 Ctrl+C  → trap 이 모든 자식 정리 후 exit 0
#   - --no-node 모드      → 모든 프로세스 살려두고 exit 0 (수동 정리 필요)
#
# 의존성:
#   - PX4-Autopilot 빌드 완료 ($PX4_DIR/build/px4_sitl_default/bin/px4)
#   - gazebo-classic + gazebo_mavlink_interface plugin
#   - MicroXRCEAgent (apt 설치)
#   - ros2 + ros2_ws 빌드 완료 (trajectory_prediction_hils)
# ════════════════════════════════════════════════════════════════════

# set -u 는 의도적으로 끔 — setup_gazebo.bash / ROS setup.bash 가 미정의 env 사용 시
# (예: $GAZEBO_PLUGIN_PATH: 빈 prefix concat) 셸 전체를 죽임. set -e 도 sleep/timeout
# 거짓 양성으로 안 씀. 대신 각 단계마다 명시적으로 ready 검증.

# ─── 경로/설정 ─────────────────────────────────────────────────────
PX4_DIR=${PX4_DIR:-/home/hmcl/workspace/swarm-fixed-wing/firmware/PX4-Autopilot}
ROS2_WS=${ROS2_WS:-/home/hmcl/workspace/swarm-fixed-wing/ros2_ws}
HILS_SRC=${HILS_SRC:-${ROS2_WS}/src/testing_module/trajectory_prediction_hils}
HILS_RESULT_ROOT=${HILS_RESULT_ROOT:-${HILS_SRC}/result}
ANALYSIS_DIR=${ANALYSIS_DIR:-${ROS2_WS}/src/testing_module/analysis_tools}
ANALYSIS_PYTHON=${ANALYSIS_PYTHON:-python3}
BUILD=${PX4_DIR}/build/px4_sitl_default
PX4_PYTHON_BIN=${PX4_PYTHON_BIN:-${PX4_PYTHON:-}}
if [[ -z "${PX4_PYTHON_BIN}" ]]; then
    if [[ -x /home/hmcl/workspace/swarm-fixed-wing/.envs/px4/bin/python3 ]]; then
        PX4_PYTHON_BIN=/home/hmcl/workspace/swarm-fixed-wing/.envs/px4/bin/python3
    else
        PX4_PYTHON_BIN=python3
    fi
fi
MICRO_XRCE_AGENT_BIN=${MICRO_XRCE_AGENT_BIN:-}
if [[ -z "${MICRO_XRCE_AGENT_BIN}" ]]; then
    if command -v MicroXRCEAgent >/dev/null 2>&1; then
        MICRO_XRCE_AGENT_BIN=$(command -v MicroXRCEAgent)
    else
        MICRO_XRCE_AGENT_BIN=/home/hmcl/workspace/swarm-fixed-wing/tools/micro-xrce-dds-agent/bin/MicroXRCEAgent
    fi
fi
MICRO_XRCE_AGENT_LIB_DIR=${MICRO_XRCE_AGENT_LIB_DIR:-$(dirname "${MICRO_XRCE_AGENT_BIN}")/../lib}
INSTANCE=0
SDF_OUT=/tmp/standard_vtol_${INSTANCE}.sdf

LOG_AGENT=/tmp/micro_xrce_agent.log
LOG_GZ=/tmp/gzserver.log
LOG_PX4=/tmp/px4_inst${INSTANCE}.log

LAUNCH_NODE=1
RECORD_BAG=${RECORD_ROSBAG:-0}
for argument in "$@"; do
    case "${argument}" in
        --no-node) LAUNCH_NODE=0 ;;
        --record-bag) RECORD_BAG=1 ;;
        *) echo "[launch] ERROR: unknown argument '${argument}'"; exit 2 ;;
    esac
done

BAG_PID=""
HILS_RUN_ID=${HILS_RUN_ID:-manual_$(date +%Y%m%d_%H%M%S)}
BAG_OUTPUT_DIR=${BAG_OUTPUT_DIR:-${HILS_RESULT_ROOT}/rosbag/${HILS_RUN_ID}}
BAG_ANALYSIS_DIR=${BAG_ANALYSIS_DIR:-${HILS_RESULT_ROOT}/raw/${HILS_RUN_ID}}

stop_rosbag() {
    if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
        echo "[launch] rosbag flush/종료 중..."
        kill -INT "${BAG_PID}" 2>/dev/null || true
        wait "${BAG_PID}" 2>/dev/null || true
    fi
    BAG_PID=""
}

# 백그라운드 PID 추적 (trap 으로 정리)
PIDS=()

cleanup() {
    echo ""
    echo "[launch] cleanup — 백그라운드 프로세스 정리 중..."
    stop_rosbag
    pkill -KILL -f "trajectory_replay_node" 2>/dev/null
    pkill -KILL -f "px4 -i ${INSTANCE}" 2>/dev/null
    pkill -KILL -f "gzserver Tools/simulation" 2>/dev/null
    pkill -KILL -f "MicroXRCEAgent udp4 -p 8888" 2>/dev/null
    rm -f /tmp/px4_lock-* /tmp/px4-sock-* 2>/dev/null
    echo "[launch] 정리 완료."
}
trap cleanup EXIT INT TERM

# ─── 1. 정리 ────────────────────────────────────────────────────────
echo "[launch 1/7] 기존 프로세스 정리..."
pkill -KILL -f "trajectory_replay_node" 2>/dev/null || true
pkill -KILL -f "px4 -i ${INSTANCE}" 2>/dev/null || true
pkill -KILL -f "gzserver Tools/simulation" 2>/dev/null || true
pkill -KILL -f "MicroXRCEAgent udp4 -p 8888" 2>/dev/null || true
sleep 2
rm -f /tmp/px4_lock-* /tmp/px4-sock-* "${LOG_AGENT}" "${LOG_GZ}" "${LOG_PX4}" 2>/dev/null

# ─── 2. MicroXRCEAgent ─────────────────────────────────────────────
echo "[launch 2/7] MicroXRCEAgent 시작 (udp4 -p 8888)..."
if [[ ! -x "${MICRO_XRCE_AGENT_BIN}" ]]; then
    echo "[launch] ERROR: MicroXRCEAgent 실행 파일 없음: ${MICRO_XRCE_AGENT_BIN}"
    exit 1
fi
LD_LIBRARY_PATH="${MICRO_XRCE_AGENT_LIB_DIR}:${LD_LIBRARY_PATH:-}" \
    "${MICRO_XRCE_AGENT_BIN}" udp4 -p 8888 > "${LOG_AGENT}" 2>&1 &
PIDS+=($!)
sleep 3
if ! pgrep -f "MicroXRCEAgent udp4 -p 8888" > /dev/null; then
    echo "[launch] ERROR: MicroXRCEAgent 시작 실패. 로그: ${LOG_AGENT}"
    exit 1
fi

# ─── 3. gzserver ────────────────────────────────────────────────────
echo "[launch 3/7] gzserver 시작 (empty.world headless)..."
cd "${PX4_DIR}"
# shellcheck disable=SC1091
source Tools/simulation/gazebo-classic/setup_gazebo.bash "${PX4_DIR}" "${BUILD}" >/dev/null 2>&1
gzserver Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world --verbose \
    > "${LOG_GZ}" 2>&1 &
PIDS+=($!)
sleep 8
if ! pgrep -f "gzserver Tools/simulation" > /dev/null; then
    echo "[launch] ERROR: gzserver 시작 실패. 로그: ${LOG_GZ}"
    exit 1
fi

# ─── 4. PX4 instance 0 ────────────────────────────────────────────
echo "[launch 4/7] PX4 instance ${INSTANCE} 시작 (PX4_UXRCE_DDS_NS=px4_${INSTANCE})..."
mkdir -p "${BUILD}/rootfs/${INSTANCE}"
cd "${BUILD}/rootfs/${INSTANCE}"
PX4_HOME_LAT=47.397742 \
PX4_HOME_LON=8.545594 \
PX4_HOME_ALT=488.0 \
PX4_UXRCE_DDS_NS="px4_${INSTANCE}" \
PX4_UXRCE_DDS_PORT=8888 \
PX4_SIM_MODEL=gazebo-classic_standard_vtol \
"${BUILD}/bin/px4" -i "${INSTANCE}" -d "${BUILD}/etc" \
    > "${LOG_PX4}" 2>&1 &
PIDS+=($!)
sleep 8

# ─── 5. SDF spawn ──────────────────────────────────────────────────
echo "[launch 5/7] standard_vtol_${INSTANCE} SDF spawn..."
if ! "${PX4_PYTHON_BIN}" \
    "${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py" \
    "${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/standard_vtol/standard_vtol.sdf.jinja" \
    "${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic" \
    --mavlink_tcp_port 4560 --mavlink_udp_port 14560 --mavlink_id 1 \
    --gst_udp_port 5600 --video_uri 5600 --mavlink_cam_udp_port 14530 \
    --output-file "${SDF_OUT}"; then
    echo "[launch] ERROR: standard_vtol SDF 생성 실패 (Python: ${PX4_PYTHON_BIN})"
    exit 1
fi
if ! gz model --spawn-file="${SDF_OUT}" --model-name="standard_vtol_${INSTANCE}" \
    -x 0 -y 0 -z 0.83; then
    echo "[launch] ERROR: standard_vtol 모델 spawn 실패."
    exit 1
fi

# ─── 6. ready check (poll 토픽) ─────────────────────────────────────
echo "[launch 6/7] uXRCE-DDS 토픽 준비 대기 (max 30s)..."
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash >/dev/null 2>&1
READY=0
for i in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | \
       grep -q "/px4_${INSTANCE}/fmu/out/register_ext_component_reply_v1"; then
        READY=1
        echo "[launch]   → ready (대기 ${i}s)"
        break
    fi
    sleep 1
done

if [[ "${READY}" != "1" ]]; then
    echo "[launch] ERROR: register_ext_component_reply_v1 토픽이 30s 안에 안 떴음."
    echo "         PX4 로그: ${LOG_PX4}"
    echo "         Agent 로그: ${LOG_AGENT}"
    exit 1
fi

# ekf2 수렴 + Ready for takeoff 추가 5초 여유
sleep 5

# ─── 7. (옵션) 노드 실행 ────────────────────────────────────────────
if [[ "${LAUNCH_NODE}" == "1" ]]; then
    echo "[launch 7/7] trajectory_replay_node 실행..."
    cd "${ROS2_WS}"
    # shellcheck disable=SC1091
    source install/setup.bash
    HILS_SHARE=$(ros2 pkg prefix --share trajectory_prediction_hils)
    # foreground 실행 — Ctrl+C 가 트랩으로 전체 정리
    # ─── case yaml override (A-3 run_all_cases.sh 가 SEQUENCE_FILE 환경변수로 전달) ──
    # ★ 중요: -p 가 --params-file 뒤에 와야 ros2 humble 의 ordering 으로 override 됨.
    #         (앞에 놓으면 뒤의 --params-file 가 default sequence_file 로 덮음)
    SEQ_PARAMS=()
    if [[ -n "${SEQUENCE_FILE:-}" ]]; then
        SEQ_PARAMS=(-p "sequence_file:=${SEQUENCE_FILE}")
        echo "[launch] case yaml override: ${SEQUENCE_FILE}"
    fi

    # ─── B_H 환경변수 override (b_h grid 시험용) ──
    # 사용: B_H=0.1 ~/...launch_1vtol_replay.sh
    # 효과: airframe_spec.yaml 의 b_h 를 무시하고 CLI 로 override.
    #       CSV 도 자동으로 /tmp/trajectory_bH<tag>_<timestamp>.csv 로 분리.
    #       grid 모드일 때 자동으로 SKIP_CHUNK_ANALYSIS=1 → 마지막에 한 번 비교.
    B_H_PARAMS=()
    if [[ -n "${B_H:-}" ]]; then
        # 0.1 → bH0p1, 0.05 → bH0p05, 0.0 → bH0p0
        B_H_TAG="bH${B_H//./p}"
        B_H_PARAMS=(-p "b_h:=${B_H}" -p "csv_path_prefix:=/tmp/trajectory_${B_H_TAG}")
        echo "[launch] b_h override: ${B_H} (CSV prefix=/tmp/trajectory_${B_H_TAG}_*)"
        # grid 모드 자동: 매번 chunk_analysis 실행하지 않음
        if [[ -z "${SKIP_CHUNK_ANALYSIS:-}" ]]; then
            export SKIP_CHUNK_ANALYSIS=1
            echo "[launch] B_H grid 모드 → SKIP_CHUNK_ANALYSIS=1 자동 설정"
        fi
    fi

    if [[ "${RECORD_BAG}" == "1" ]]; then
        echo "[launch] rosbag 기록 시작: ${BAG_OUTPUT_DIR}"
        bash "${HILS_SHARE}/../../lib/trajectory_prediction_hils/record_trajectory_cone_bag.sh" \
            "${BAG_OUTPUT_DIR}" "/px4_${INSTANCE}" \
            > /tmp/trajectory_cone_rosbag.log 2>&1 &
        BAG_PID=$!
        PIDS+=("${BAG_PID}")
        sleep 2
        if ! kill -0 "${BAG_PID}" 2>/dev/null; then
            echo "[launch] ERROR: rosbag recorder 시작 실패. 로그: /tmp/trajectory_cone_rosbag.log"
            exit 1
        fi
    fi

    ros2 run trajectory_prediction_hils trajectory_replay_node --ros-args \
        -p topic_namespace_prefix:=/px4_${INSTANCE} \
        -p vehicle_ID:=${INSTANCE} \
        --params-file "${HILS_SHARE}/config/replay_params.yaml" \
        --params-file "${HILS_SHARE}/config/airframe_spec.yaml" \
        "${SEQ_PARAMS[@]}" \
        "${B_H_PARAMS[@]}"
    # rclcpp::spin 끝나면 (Ctrl+C 또는 자동 shutdown) 여기로 옴

    if [[ "${RECORD_BAG}" == "1" ]]; then
        stop_rosbag
        if [[ "${SKIP_ROSBAG_ANALYSIS:-0}" == "1" ]]; then
            echo "[launch] rosbag 기록 완료 — 오프라인 분석 대기: ${BAG_OUTPUT_DIR}"
        else
            echo "[launch] rosbag 분석: ${BAG_ANALYSIS_DIR}"
            "${HILS_SHARE}/../../lib/trajectory_prediction_hils/analyze_trajectory_cone_bag.py" \
                "${BAG_OUTPUT_DIR}" --output "${BAG_ANALYSIS_DIR}" \
                --namespace "/px4_${INSTANCE}" || \
                echo "[launch] WARN: rosbag smoke 분석 실패 — ${BAG_ANALYSIS_DIR}/summary.json 확인"
        fi
    fi

    # ─── 분석: 가장 최근 CSV → chunk_analysis → PNG 2장 ────────────
    # SKIP_CHUNK_ANALYSIS=1 면 분석 스킵 (run_all_cases.sh 가 끝에 별도 analyze_cases.py 호출).
    if [[ -z "${SKIP_CHUNK_ANALYSIS:-}" ]]; then
        LATEST_CSV=$(ls -t /tmp/trajectory_*.csv 2>/dev/null | head -1)
        if [[ -n "${LATEST_CSV}" ]]; then
            OUT_DIR=/tmp/tc_synced
            mkdir -p "${OUT_DIR}"
            echo "[launch] 사후분석: ${LATEST_CSV} → ${OUT_DIR}/"
            "${ANALYSIS_PYTHON}" \
                "${ANALYSIS_DIR}/chunk_analysis.py" \
                "${LATEST_CSV}" --out-dir "${OUT_DIR}/" || \
                echo "[launch] WARN: chunk_analysis 실패 — 수동 실행 필요"
        else
            echo "[launch] WARN: /tmp/trajectory_*.csv 없음 → 분석 스킵"
        fi
    fi
    # → 함수 종료 후 trap 발동 (SITL 정리)
else
    echo "[launch 7/7] --no-node 모드 → SITL 만 띄움."
    echo "             별도 셸에서 다음 명령으로 노드 실행:"
    echo ""
    echo "  cd ${ROS2_WS}"
    echo "  source install/setup.bash"
    echo "  HILS_SHARE=\$(ros2 pkg prefix --share trajectory_prediction_hils)"
    echo "  ros2 run trajectory_prediction_hils trajectory_replay_node --ros-args \\"
    echo "      -p topic_namespace_prefix:=/px4_${INSTANCE} -p vehicle_ID:=${INSTANCE} \\"
    echo "      --params-file \"\${HILS_SHARE}/config/replay_params.yaml\" \\"
    echo "      --params-file \"\${HILS_SHARE}/config/airframe_spec.yaml\""
    echo ""
    echo "  종료: pkill -KILL -f 'px4|gzserver|MicroXRCE|trajectory_replay'"
    # trap 끄기 — --no-node 모드는 SITL 살려두고 종료해야 함
    trap - EXIT INT TERM
    exit 0
fi
