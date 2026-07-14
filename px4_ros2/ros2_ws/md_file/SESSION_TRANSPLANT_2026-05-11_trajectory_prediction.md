# Session Transplant — trajectory_prediction 패키지 (2026-05-11 세션 전체)

이 문서는 새 Claude 세션에서 본 세션의 컨텍스트를 100% 복원하기 위한 종합 핸드오프.
원본 세션 jsonl: `/home/leedonghyuck/.claude/projects/-home-leedonghyuck-ros2-ws/97a94807-02a6-489b-860a-2c29be465a4e.jsonl`

## 0. 새 세션 시작 시 첫 메시지 권장

```
이 문서를 처음부터 끝까지 읽고 컨텍스트 복원해라.
경로: /home/leedonghyuck/ros2_ws/md_file/SESSION_TRANSPLANT_2026-05-11_trajectory_prediction.md
또한 다음 메모리도 함께 참조:
  - ~/.claude/projects/-home-leedonghyuck/memory/project_trajectory_prediction.md
  - ~/.claude/projects/-home-leedonghyuck/memory/MEMORY.md (인덱스)
컨텍스트 복원 완료 후, §13 의 "다음 단계 (미완료)" 부터 이어서 진행.
```

---

## 1. 사용자 정보 (영속)

- **이메일**: id0154199@gmail.com
- **역할**: PX4 고정익 + ROS2 오프보드 제어 개발자
- **작업 환경**: Ubuntu Linux, ROS2 Humble, anaconda3
- **선호 응답 언어**: 한국어
- **코딩 스타일** (`feedback_coding_style.md`): 학습 지향, 코드 흐름 / 동작 원리 자세한 설명 선호. 실제 동작 검증 중시.

---

## 2. 프로젝트 큰 그림

### 2.1 작업 의도

사용자가 보낸 setpoint 가 PX4 컨트롤러를 통과한 뒤 실제로 어떤 궤적을 그릴지 RK4 적분으로 사전 예측 → 실측과 비교 → "모델 vs 실제 동역학" 차이 정량화 → 충돌회피 (collision_avoidance) / 경로계획 입력으로 활용.

### 2.2 워크스페이스 구조

```
/home/leedonghyuck/ros2_ws/
├── src/
│   ├── trajectory_prediction/        ← 본 세션 작업 패키지
│   │   ├── include/trajectory_prediction/
│   │   │   ├── StateType.hpp
│   │   │   ├── TrajectoryPredict.hpp
│   │   │   ├── TrajectoryLogger.hpp
│   │   │   ├── TrajectoryReplayMode.hpp
│   │   │   ├── VtolPreflightMode.hpp
│   │   │   ├── VtolGuidanceExecutor.hpp
│   │   │   └── SetpointSequencer.hpp
│   │   ├── src/
│   │   │   ├── main.cpp
│   │   │   ├── TrajectoryPredict.cpp
│   │   │   ├── TrajectoryLogger.cpp
│   │   │   ├── TrajectoryReplayMode.cpp
│   │   │   ├── VtolPreflightMode.cpp
│   │   │   ├── VtolGuidanceExecutor.cpp
│   │   │   └── SetpointSequencer.cpp
│   │   ├── test/test_trajectory_predict.cpp
│   │   ├── config/
│   │   │   ├── airframe_spec.yaml      ← 본 세션 ★ 시정수 동기화 ★
│   │   │   ├── replay_params.yaml
│   │   │   └── setpoint_sequence.yaml
│   │   ├── scripts/
│   │   │   ├── chunk_analysis.py
│   │   │   └── launch_1vtol_replay.sh  ← 본 세션 신규
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── collision_avoidance/             ← 별도 패키지 (참조용)
│       └── scripts/launch_5vtol.sh      ← 본 세션 환경 fix 의 기준 패턴
├── md_file/
│   ├── TASK_trajectory_predictor.md     ← 라이브러리 명세 (★ ground truth ★)
│   ├── PROPOSAL_trajectory_predictor_update.md
│   ├── TRAJECTORY_PREDICT_HOW_IT_WORKS.md
│   ├── PROJECT_FILE_DIAGRAM.md
│   ├── FORMATION_MODE_ARCHITECTURE.md
│   ├── SUBSCRIBER_DATA_FLOW.md
│   ├── HANDOFF_2026-05-11_tc_sync_verification.md  ← 짧은 버전
│   └── SESSION_TRANSPLANT_2026-05-11_trajectory_prediction.md  ← 본 문서
└── (build / install / log 디렉토리)
```

### 2.3 의존 외부 디렉토리

- PX4: `/home/leedonghyuck/PX4-Autopilot/` (v1.17, 빌드 완료)
- 개발 라이브러리: `/home/leedonghyuck/px4-ros2-interface-lib-main/`
- collision_avoidance 참조: `/home/leedonghyuck/ros2_ws/src/collision_avoidance/scripts/launch_5vtol.sh`

---

## 3. 디자인 원칙 (변경 불가, 명세에 의해 확정)

### 3.1 자료형 (`StateType.hpp`)

```cpp
namespace trajectory_prediction {

// 7-state POD (★ PATCH 후 phi 마지막 ★)
struct PredictState {
    double p_n;     // [m]    NED north
    double p_e;     // [m]    NED east
    double h;       // [m]    altitude (up positive)
    double V;       // [m/s]  airspeed magnitude
    double psi;     // [rad]  heading, wrapped to [-pi, pi]
    double h_dot;   // [m/s]  current climb rate
    double phi;     // [rad]  roll angle (★ PATCH: 이전 a_lat 에서 변경)
};
static_assert(sizeof(PredictState) == 56);   // 1 cache line

// 외부 입력 POD — FwSetpointOutput_rt2mt 와 1:1 단위 매칭
struct PredictInput {
    double V_cmd;       // [m/s]
    double h_dot_cmd;   // [m/s]
    double a_lat_cmd;   // [m/s²]
};

struct PredictParams {
    double tau_V       = 4.0;    // FW_T_TAS_TC
    double tau_hdot    = 2.0;    // FW_T_ALT_TC
    double tau_phi     = 0.5;    // FW_R_TC (★ PATCH: tau_a → tau_phi)
    double V_min       = 12.0;   // FW_AIRSPD_MIN
    double V_max       = 25.0;   // FW_AIRSPD_MAX
    double h_dot_max   = 5.0;    // FW_T_CLMB_MAX (대칭 가정)
    double a_lat_max   = 9.8;    // g * tan(FW_R_LIM)
    double V_h_min     = 1.0;    // numerical safety
};

}  // namespace trajectory_prediction
```

### 3.2 RT 특성 보장

- **stateless / heap-free / lock-free**: 명세 §6 + 단위테스트 (heap-free / WCET / jitter) 로 검증
- **horizon = 45** (= 4.5s × 10Hz), `TrajectoryLogger::kPredictHorizon` constexpr
- **외부 의존 0**: 라이브러리 자체 (`StateType.hpp`, `TrajectoryPredict.hpp/.cpp`) 는 Eigen / rclcpp / px4_msgs include 금지
- **시정수 외부 주입**: `airframe_spec.yaml` 의 `tc_tas/tc_alt/tc_roll`, rt_thread 시작 후 변경 금지
- **분기 회피**: clamp → fmin/fmax, wrapPi → fmod 2회

### 3.3 ODE (Beard-McLain 정형, ★ PATCH 적용 후)

```
V_h          = sqrt(V² - h_dot²)              (수평 속도, V_h_min 으로 floor)
p_n_dot      = V_h * cos(psi)
p_e_dot      = V_h * sin(psi)
h_dot_pos    = h_dot
V_dot        = (V_cmd     - V    ) / tau_V
psi_dot      = g * tan(phi) / V_h              ★ PATCH: 이전 a_lat / V_h
h_ddot       = (h_dot_cmd - h_dot) / tau_hdot
phi_dot      = (phi_cmd   - phi  ) / tau_phi   ★ PATCH: 신규 (1차 지연)
```

외부 입력 변환 (stepRK4 진입 시 1회):
```
phi_cmd = atan2(a_lat_cmd, g)
```

### 3.4 안전 saturation

`applyInputSaturation`:
```cpp
V_cmd     ← clamp(V_cmd,    V_min,        V_max)
h_dot_cmd ← clamp(h_dot_cmd, -h_dot_max,   h_dot_max)
a_lat_cmd ← clamp(a_lat_cmd, -a_lat_max,   a_lat_max)
```

`applyStateSafety`:
```cpp
V         ← max(V, V_h_min)               (수치 안전 floor)
|h_dot|   ← clamp(h_dot, ±0.99·V)        (V_h 발산 방지)
phi       ← clamp(phi, ±60°)              (★ PATCH: tan 발산 방지)
psi       ← wrapPi(psi)
```

### 3.5 RK4 (1 step, ZOH input)

```cpp
PredictState stepRK4(const PredictState & x, const PredictInput & u, double dt) {
    u_sat  = applyInputSaturation(u);
    x_sat  = applyStateSafety(x);
    u_int  = { u_sat.V_cmd, u_sat.h_dot_cmd, atan2(u_sat.a_lat_cmd, g) };  // ★ PATCH

    k1 = evaluateODE(x_sat,                              u_int);
    k2 = evaluateODE(scaleAndAdd(x_sat, 0.5*dt, k1),     u_int);
    k3 = evaluateODE(scaleAndAdd(x_sat, 0.5*dt, k2),     u_int);
    k4 = evaluateODE(scaleAndAdd(x_sat,     dt, k3),     u_int);

    x_next = x_sat + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
    return applyStateSafety(x_next);
}
```

### 3.6 네이밍 컨벤션 (collision_avoidance 와 동일)

- `_mt`   : main thread 가 쓰는 멤버
- `_rt`   : rt_thread 가 쓰는 멤버
- `_mt2rt`: main → rt 단방향 lock-free SPSC
- `_rt2mt`: rt → main 단방향 lock-free SPSC

---

## 4. 파일별 책임 / 핵심 코드

### 4.1 `include/trajectory_prediction/StateType.hpp`
- 위 3.1 의 3개 POD + `static_assert(sizeof(PredictState)==56)`
- 기존 `namespace StateType::FwSetpointOutput_rt2mt` 와 공존

### 4.2 `include/trajectory_prediction/TrajectoryPredict.hpp`
- public API: `stepRK4(x, u, dt)`, `predict<N_STEPS>(x0, u_zoh, dt, out_traj)`, `setParams(...)`
- private: `InternalInput { V_cmd, h_dot_cmd, phi_cmd }` (★ PATCH 신규), `evaluateODE(x, InternalInput)`, `applyStateSafety`, `applyInputSaturation`
- namespace `trajectory_prediction`. Eigen 제거. double.

### 4.3 `src/TrajectoryPredict.cpp`
- 위 3.3 ODE 7개 식 + 3.4 saturation + 3.5 RK4 + psi wrap (fmod 2회)
- 헤더 코멘트로 명세 §2.3 + PATCH 출처 명시
- `constexpr double k_g = 9.80665`
- `constexpr double k_phi_hard_limit = 1.047197551`  // 60° in rad

### 4.4 `include/trajectory_prediction/TrajectoryLogger.hpp` / `src/TrajectoryLogger.cpp`
- `static constexpr std::size_t kPredictHorizon = 45`
- `using PredictedTrajectory = std::array<PredictState, 45>`
- 50Hz wall_timer (별도 thread) → `onTick()` 가 CSV 한 줄 fprintf
- 3개 PX4 토픽 구독 (sensor_data QoS, atomic 슬롯 store):
  - `<ns>/fmu/out/vehicle_local_position_v1` → x, y, z, vn, ve, vd
  - `<ns>/fmu/out/vehicle_attitude` → quaternion → yaw
  - `<ns>/fmu/out/airspeed_validated_v1` → true_airspeed
- `setReplayTime(t)` / `getReplayTime()` atomic (★ schedule-aware 용)
- CSV 컬럼: `t_us, sp_v, sp_h, sp_a, sp_fallback, x, y, z, vn, ve, vd, yaw, true_airspeed, p_pn_0..p_phi_44` (13 + 7×45 = 328)
- ★ PATCH: 마지막 prediction 컬럼 `p_alat_k → p_phi_k`

### 4.5 `src/main.cpp` (전체 흐름 핵심)

**부팅 흐름**:
1. Node 생성 (`trajectory_replay_node`)
2. 파라미터 선언/조회:
   - 노드: `vehicle_ID`, `sequence_file`, `csv_path_prefix`, `log_rate_hz`, `topic_namespace_prefix`
   - airframe_spec.yaml: `airspeed_min/max`, `height_rate_max_climb/min_sink`, `max_roll_deg`, `gravity`, `tc_tas`, `tc_alt`, `tc_roll`
   - replay_params.yaml: `predict_horizon_s`, `predict_rate_hz`, `predict_call_hz`
3. 컴포넌트 인스턴스화: SetpointSequencer / TrajectoryLogger / VtolPreflightMode / TrajectoryReplayMode / TrajectoryPredict
4. Executor 생성 + Predict wall_timer 등록 (10Hz)
5. `doRegister()` (Executor + Replay 둘 다)
6. `rclcpp::spin(node)`

**Predict wall_timer 람다 (★ schedule-aware vs ZOH fallback ★)**:
```cpp
[predictor, logger, sequencer_for_predict, predicted_traj, dt, k_g]() {
    const auto m  = logger->getCurrentMeasurements();
    const auto sp = logger->getCurrentSetpointSnapshot();

    // 6-state 측정 → 7-state PredictState (PATCH 적용)
    PredictState x0;
    x0.p_n   = m.x;
    x0.p_e   = m.y;
    x0.h     = -m.z;                         // NED z (down) → altitude (up)
    x0.V     = m.true_airspeed;
    x0.psi   = m.yaw;
    x0.h_dot = -m.vd;                        // NED vd (down) → climb rate (up)
    x0.phi   = std::atan2(sp.a_lat, k_g);    // ★ PATCH: setpoint a_lat → 등가 phi

    const double t_now = logger->getReplayTime();

    if (std::isfinite(t_now) && sequencer_for_predict) {
        // ─── 모드 A: schedule-aware ───
        auto & traj = *predicted_traj;
        traj[0] = x0;
        PredictState x = x0;
        for (std::size_t k = 0; k + 1 < TrajectoryLogger::kPredictHorizon; ++k) {
            const double t_future = t_now + (k+1) * dt;
            const auto sp_at_t = sequencer_for_predict->lookup(t_future);
            PredictInput u_at_t{ sp_at_t.V, sp_at_t.h_dot, sp_at_t.a_lat };
            x = predictor->stepRK4(x, u_at_t, dt);
            traj[k+1] = x;
        }
    } else {
        // ─── 모드 B: ZOH fallback (Preflight 또는 Replay 비활성) ───
        PredictInput u_zoh{ sp.V, sp.h_dot, sp.a_lat };
        predictor->predict<TrajectoryLogger::kPredictHorizon>(
            x0, u_zoh, dt, *predicted_traj);
    }
    logger->pushPredictedTrajectory(*predicted_traj);
};
```

### 4.6 `src/TrajectoryReplayMode.cpp` (rt_thread 책임)
- rt_loop: 매 cycle 마다 `m_logger->setReplayTime(t)` 호출 (★ schedule-aware 활성화)
- onActivate: CSV 기록 시작 + sequencer t=0 reset
- 시퀀스 lookup → 현재 segment setpoint → PX4 publish

### 4.7 `test/test_trajectory_predict.cpp` (12개 테스트)

**정확성 7개**:
1. 정상상태 수렴 (오차 < 1%)
2. 1차 응답: τ 시점 63.2% 도달
3. 조정선회 원궤도: R = V²/a_lat (오차 < 5%)
4. 직선 등속
5. Saturation 작동
6. 수치 안정성 (dt=0.01 vs 0.1, 1.0초 동일 적분 비교)
7. Stateless (같은 입력 두 번 → 같은 출력)

**RT 특성 3개**:
8. WCET (horizon ∈ {5, 10, 15, 30} × 100,000 호출, mean/median/max/p99)
9. Heap allocation 부재 (`__libc_malloc` mock hook, predict<30>×1000 → malloc 0회)
10. Jitter (σ < 0.1 × mean)

**phi-PATCH 추가 2개**:
11. `AlatPhiEquivalenceAtSteadyState`: 여러 a_lat_cmd 값에 대해 정상상태 phi 가 g·tan(phi)≈a_lat_cmd 1% 이내
12. `LargeRollAngleAccuracy`: a_lat_cmd=9 → phi≈42.5°, atan2 정확값과 small-angle 근사 차이 검증

### 4.8 `config/airframe_spec.yaml` (★ tc 동기화 후)

```yaml
"/**":
  ros__parameters:
    airspeed_min: 10.0          # FW_AIRSPD_MIN (SITL live: 10.0) ← 12 → 10
    airspeed_cruise: 15.0
    airspeed_max: 25.0          # FW_AIRSPD_MAX (SITL live: 25.0)

    height_rate_max_climb: 8.0
    height_rate_min_sink: 2.7
    height_rate_max_sink: 2.7

    gravity: 9.80665
    max_roll_deg: 50.0

    max_pitch_deg: 30.0
    max_pitch_rate_deg_per_sec: 60.0
    energy_fraction: 0.5

    # 2026-05-11: SITL 1040_gazebo-classic_standard_vtol live readout 으로 동기화
    tc_tas:  5.0    # FW_T_TAS_TC : airspeed error TC [s] (PX4 default 5.0) ← 4.0 → 5.0
    tc_alt:  5.0    # FW_T_ALT_TC : altitude error TC [s] (PX4 default 5.0) ← 2.0 → 5.0 ★
    tc_roll: 0.4    # FW_R_TC     : roll TC [s]           (PX4 default 0.4) ← 0.5 → 0.4
```

### 4.9 `config/setpoint_sequence.yaml` (★ 수평 시퀀스 — 종/횡 분리 검증용)

```yaml
end_policy: hold_last

sequence:
  - { t_start:  0.0, t_end: 10.0, airspeed: 16.0, height_rate: 0.0, lateral_acceleration:  0.0 }   # 직진 cruise
  - { t_start: 10.0, t_end: 15.0, airspeed: 16.0, height_rate: 0.0, lateral_acceleration:  4.0 }   # 우선회
  - { t_start: 15.0, t_end: 20.0, airspeed: 20.0, height_rate: 0.0, lateral_acceleration:  0.0 }   # 직진+가속
  - { t_start: 20.0, t_end: 25.0, airspeed: 18.0, height_rate: 0.0, lateral_acceleration: -4.0 }   # 좌선회
  - { t_start: 25.0, t_end: 35.0, airspeed: 16.0, height_rate: 0.0, lateral_acceleration:  0.0 }   # 직진 복귀
```

→ 모든 segment 가 `height_rate: 0` 라 종 채널 무자극. tc_tas / tc_roll 식별 전용.

**★ 주의**: 이 yaml 은 **ROS2 params 파일이 아님**. SetpointSequencer 가 직접 읽는 별도 형식. `--params-file` 로 넘기면 안 됨.
`main.cpp:54` 에서 `sequence_file` 파라미터의 default 가 절대경로로 잡혀있어서 자동 적재됨.

### 4.10 `config/replay_params.yaml`

```yaml
"/**":
  ros__parameters:
    predict_horizon_s: 4.5     # 예측 미래 길이
    predict_rate_hz:   10.0    # predict 내부 dt = 1 / rate = 0.1s
    predict_call_hz:   10.0    # wall_timer 호출 주기
```

### 4.11 `scripts/chunk_analysis.py` (사후분석)

**원리**: rolling-mode CSV 에서 4.5초 간격 (225 rows = 4.5s × 50Hz) 행만 추출 → chunk-mode 와 동치.

```
chunk i 시작 행 = i × 225
chunk i 의 예측 = 그 행의 p_*_0 ~ p_*_44  (45점, 0.1s 간격)
chunk i 의 실측 = 행 (i×225 + k×5) 의 측정값  (k = 0..44)
오차 = 예측 - 실측 (45 페어 × N chunks)
```

**비교 채널**:
- XY: √((p_pn-x)² + (p_pe-y)²)
- 고도 h: |p_h - (-z)|  (NED z down → altitude up)
- 속도 V: |p_V - true_airspeed|
- 헤딩 psi: |wrap(p_psi - yaw)| → degrees

**미비교**: phi (측정 채널 없음), h_dot (구현 안 됨)

**출력**:
- stdout RMSE 표 (chunk × 8 통계: XY/h/V/psi 의 end & mean)
- `chunk_error_curves.png`: 4 subplot, 시간 vs 4 채널 오차, chunk 별 색 다름
- `chunk_trajectories.png`: chunk 별 XY top-view, **녹색 실선 = measured, 빨간 점선 = predicted** (★ 사용자 confirmed 색 규칙)

**사용**:
```bash
./chunk_analysis.py [csv]                  # /tmp/trajectory_*.csv 최신 자동
./chunk_analysis.py [csv] --out-dir /tmp/foo/   # 디렉토리 지정 (사전 mkdir 필요)
./chunk_analysis.py --max-chunks 0         # 전체 chunk plot (기본 8)
```

**★ 주의**: 시스템 python3 에 pandas 없음 → `/home/leedonghyuck/anaconda3/bin/python3` 사용.

**PATCH 호환**: 새 CSV 는 `p_phi_k`, 구 CSV 는 `p_alat_k`. 자동 분기.

### 4.12 `scripts/launch_1vtol_replay.sh` (★ 본 세션 신규, 1차 버그 fix 후)

7 단계 통합 부팅 스크립트:

```bash
#!/usr/bin/env bash
# set -u 는 의도적으로 끔 — setup_gazebo.bash 가 미정의 env 사용 시 셸 죽음

PX4_DIR=${PX4_DIR:-/home/leedonghyuck/PX4-Autopilot}
ROS2_WS=${ROS2_WS:-/home/leedonghyuck/ros2_ws}
BUILD=${PX4_DIR}/build/px4_sitl_default
INSTANCE=0
SDF_OUT=/tmp/standard_vtol_${INSTANCE}.sdf
LOG_AGENT=/tmp/micro_xrce_agent.log
LOG_GZ=/tmp/gzserver.log
LOG_PX4=/tmp/px4_inst${INSTANCE}.log

LAUNCH_NODE=1
[[ "${1:-}" == "--no-node" ]] && LAUNCH_NODE=0

cleanup() {
    pkill -KILL -f "trajectory_replay_node" 2>/dev/null
    pkill -KILL -f "px4 -i ${INSTANCE}" 2>/dev/null
    pkill -KILL -f "gzserver Tools/simulation" 2>/dev/null
    pkill -KILL -f "MicroXRCEAgent udp4 -p 8888" 2>/dev/null
    rm -f /tmp/px4_lock-* /tmp/px4-sock-* 2>/dev/null
}
trap cleanup EXIT INT TERM

# 1. 정리
pkill -KILL -f "trajectory_replay_node" 2>/dev/null || true
pkill -KILL -f "px4 -i ${INSTANCE}" 2>/dev/null || true
pkill -KILL -f "gzserver Tools/simulation" 2>/dev/null || true
pkill -KILL -f "MicroXRCEAgent udp4 -p 8888" 2>/dev/null || true
sleep 2
rm -f /tmp/px4_lock-* /tmp/px4-sock-* "${LOG_AGENT}" "${LOG_GZ}" "${LOG_PX4}" 2>/dev/null

# 2. MicroXRCEAgent
MicroXRCEAgent udp4 -p 8888 > "${LOG_AGENT}" 2>&1 &
sleep 3

# 3. gzserver
cd "${PX4_DIR}"
source Tools/simulation/gazebo-classic/setup_gazebo.bash "${PX4_DIR}" "${BUILD}" >/dev/null 2>&1
gzserver Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world --verbose > "${LOG_GZ}" 2>&1 &
sleep 8

# 4. PX4 instance 0 (multi-vehicle namespace)
mkdir -p "${BUILD}/rootfs/${INSTANCE}"
cd "${BUILD}/rootfs/${INSTANCE}"
PX4_HOME_LAT=47.397742 PX4_HOME_LON=8.545594 PX4_HOME_ALT=488.0 \
PX4_UXRCE_DDS_NS="px4_${INSTANCE}" PX4_UXRCE_DDS_PORT=8888 \
PX4_SIM_MODEL=gazebo-classic_standard_vtol \
"${BUILD}/bin/px4" -i "${INSTANCE}" -d "${BUILD}/etc" > "${LOG_PX4}" 2>&1 &
sleep 8

# 5. SDF spawn
python3 "${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py" \
    "${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/standard_vtol/standard_vtol.sdf.jinja" \
    "${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic" \
    --mavlink_tcp_port 4560 --mavlink_udp_port 14560 --mavlink_id 1 \
    --gst_udp_port 5600 --video_uri 5600 --mavlink_cam_udp_port 14530 \
    --output-file "${SDF_OUT}" >/dev/null 2>&1
gz model --spawn-file="${SDF_OUT}" --model-name="standard_vtol_${INSTANCE}" -x 0 -y 0 -z 0.83 >/dev/null 2>&1

# 6. ready check (max 30s)
source /opt/ros/humble/setup.bash >/dev/null 2>&1
for i in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -q "/px4_${INSTANCE}/fmu/out/register_ext_component_reply_v1"; then
        echo "[launch]   → ready (대기 ${i}s)"; break
    fi
    sleep 1
done
sleep 5   # ekf2 수렴 여유

# 7. 노드 실행 (LAUNCH_NODE=1) 또는 종료 (--no-node)
if [[ "${LAUNCH_NODE}" == "1" ]]; then
    cd "${ROS2_WS}"
    source install/setup.bash
    ros2 run trajectory_prediction trajectory_replay_node --ros-args \
        -p topic_namespace_prefix:=/px4_${INSTANCE} -p vehicle_ID:=${INSTANCE} \
        --params-file src/trajectory_prediction/config/replay_params.yaml \
        --params-file src/trajectory_prediction/config/airframe_spec.yaml
fi
```

**사용법**:
```bash
~/ros2_ws/src/trajectory_prediction/scripts/launch_1vtol_replay.sh           # 전체 + 노드
~/ros2_ws/src/trajectory_prediction/scripts/launch_1vtol_replay.sh --no-node # SITL 만
```

**핵심 디자인**:
- `set -u` 빼야 함 (setup_gazebo.bash 죽임)
- `set -e` 빼야 함 (sleep/timeout 거짓 양성)
- 각 단계 ready 검증 (sleep 만으로 충분히 안 기다리면 doRegister timeout)
- 토픽 polling 으로 PX4 ↔ uXRCE-DDS 연결 확실히 대기
- trap 으로 Ctrl+C 시 모든 자식 정리

---

## 5. 진화 history (시간순)

### 5.1 (이전 세션) 명세 기준 라이브러리 재작성
기존 6-state Eigen + placeholder → 7-state Beard-McLain POD + RK4. namespace `trajectory_prediction`. Eigen 제거. WCET / heap-free / jitter 단위테스트 추가.

### 5.2 (이전 세션) PATCH: a_lat → phi (Beard-McLain 정형)
- 이유: a_lat 은 small-angle 근사 (psi_dot = a_lat / V_h). 큰 롤각에서 부정확.
- 변경:
  - 7-state 마지막 변수 `a_lat → phi`
  - `tau_a → tau_phi`
  - `psi_dot = g·tan(phi) / V_h`
  - `phi_dot = (phi_cmd - phi) / tau_phi`
  - 외부 입력 (`PredictInput`) 은 그대로 (`a_lat_cmd`)
  - 내부에서 `phi_cmd = atan2(a_lat_cmd, g)` 변환 (stepRK4 진입 시 1회)
  - `applyStateSafety` 에 `phi clamp ±60°` 추가
  - CSV 컬럼 `p_alat_k → p_phi_k`
  - 단위테스트 2개 추가 (a_lat↔phi 등가 / 큰 롤각 정확도)

### 5.3 (이전 세션) Schedule-aware predict 패턴
- 발견: chunk 2 의 10m XY 발산은 모델 본질 한계가 아니라 ZOH 단일 입력의 단순화 때문
- 해결:
  - `TrajectoryLogger` 에 `m_replay_t_relative` atomic + `setReplayTime` / `getReplayTime`
  - `TrajectoryReplayMode` rt_loop 에서 매 cycle `m_logger->setReplayTime(t)`
  - `main.cpp` 람다에 별도 `sequencer_for_predict` + schedule-aware 분기 + ZOH fallback
- 결과 (chunk 2, segment 전환):
  - ZOH XY end 10.14m → schedule-aware 7.98m (-22%)
  - ZOH psi end **39.22°** → schedule-aware **2.55°** (-93%)

### 5.4 (이전 세션) 환경 fix — multi-vehicle SITL 패턴
- 문제: PX4 v1.17 + px4_ros2_cpp 의 doRegister() 가 single-vehicle SITL 에서 namespace mismatch 로 실패
- 해결: collision_avoidance/scripts/launch_5vtol.sh 의 multi-vehicle 환경변수 패턴 차용
  - `PX4_UXRCE_DDS_NS=px4_0`, `PX4_UXRCE_DDS_PORT=8888`, `px4 -i 0`
  - 노드 파라미터 `topic_namespace_prefix=/px4_0`
  - 토픽 suffix `_v1` (vehicle_local_position_v1, airspeed_validated_v1)
- ★ "multi-vehicle" 이름이지만 실제로는 1대 + namespace prefix 만 차용

### 5.5 (이전 세션) chunk-mode 사후분석 — 수평 시퀀스 결과 (a_lat 기반 + tc_alt=2.0)
CSV: `/tmp/trajectory_20260511_142459.csv`
Plot: `/tmp/horizontal/chunk_*.png`

| chunk | t_start | 영역 | XY end | h end | psi end |
|---|---|---|---|---|---|
| 2 | 9.0s | seg1→2 (우선회) | 9.33m | 0.49m | 5.64° |
| 3 | 13.5s | seg2→3 (직진 가속) | 1.47m | 0.38m | 5.10° |
| 4 | 18.0s | seg3→4 (좌선회) | 3.96m | 0.55m | 4.04° |

→ 종 채널 거의 0 (h end < 1m), tc_alt=2.0 이 무자극이라 영향 안 나옴.

### 5.6 (이전 세션) chunk-mode 사후분석 — phi 기반 + tc_alt=2.0
CSV: `/tmp/trajectory_20260511_145556.csv`
Plot: `/tmp/phi/chunk_*.png`

| chunk | XY end (a_lat→phi) | psi end (a_lat→phi) |
|---|---|---|
| 1 | 4.04 → 4.12m | 0.48° → 0.34° (similar) |
| 2 | 9.33 → 8.92m | 5.64° → **5.06°** (-10%) |
| 3 | 1.47 → 2.21m | 5.10° → 4.75° (-7%) |
| 4 | 3.96 → 3.93m | 4.04° → 3.76° (-7%) |
| 5 | 4.02 → 3.45m (XY -14%) | — |

→ phi 기반이 marginal 개선 (psi 평균 -8%). 큰 롤각 (a_lat≈9) 시퀀스에서 더 큰 차이 예상.

### 5.7 (★ 본 세션) PX4 SITL live 시정수 readout
사용자가 직접 `pxh>` 콘솔에서 `param show ...` 입력 → 결과 6개:
- `FW_T_TAS_TC = 5.0`  (모델 4.0 → -20% 오차)
- `FW_T_ALT_TC = 5.0`  (모델 2.0 → **-60% 오차** ★)
- `FW_R_TC     = 0.4`  (모델 0.5 → +25% 오차)
- `FW_R_LIM    = 50°`  (OK)
- `FW_AIRSPD_MIN = 10.0`  (모델 12.0 → -17%)
- `FW_AIRSPD_MAX = 25.0`  (OK)

→ 1040 기체 설정이 위 3개 시정수 재정의 안 함 → PX4 default 그대로 적용됨을 확정.

### 5.8 (★ 본 세션) airframe_spec.yaml 동기화
위 §4.8 의 4개 값 갱신: `airspeed_min: 10.0`, `tc_tas: 5.0`, `tc_alt: 5.0`, `tc_roll: 0.4`.

### 5.9 (★ 본 세션) launch_1vtol_replay.sh 신규
위 §4.12. 1차 작성 후 `set -u` 버그로 단계 3 에서 죽음 → fix 후 정상 동작 가능.

### 5.10 (★ 본 세션) 메모리 갱신
- `~/.claude/projects/-home-leedonghyuck/memory/MEMORY.md` 인덱스 한 줄 갱신
- `~/.claude/projects/-home-leedonghyuck/memory/project_trajectory_prediction.md`:
  - frontmatter description 갱신 ("phi PATCH + tc 동기화 완료")
  - 7-state POD 마지막 멤버 `a_lat → phi` 명시
  - 단위테스트 12/12 (phi-PATCH 추가 2개) 반영
  - "★ PX4 SITL live 시정수 동기화 (2026-05-11)" 신규 섹션 추가

### 5.11 (★ 본 세션) 검증 비행 시도 — 미완료
- 1차: 사용자가 `make px4_sitl gazebo-classic_standard_vtol` 띄움 → param 확인 → 종료
- 2차: 직접 단계별로 부팅 → trajectory_replay_node 실행 → **`Quad-chute triggered` (VTOL 천이 실패)** + `failsafe`
- 3차: cleanup 후 재시도 → **`doRegister() 실패` (15초 timeout)** — PX4 가 uXRCE-DDS 연결 완료 전에 노드 시작
- 4차: launch_1vtol_replay.sh 실행 → **단계 3 에서 `set -u` 로 셸 죽음**
- 5차: `set -u` 제거 후 사용자가 다시 실행 안 한 상태 ← **여기서 세션 끝남**

---

## 6. 단위테스트 결과 (호스트 PC, 2026-05-11)

| 분류 | 결과 |
|---|---|
| 정확성 7개 | 7/7 PASS |
| RT 특성 3개 | 3/3 PASS |
| phi-PATCH 추가 2개 | 2/2 PASS |
| **총** | **12/12 PASS** |
| WCET (horizon=15) | mean 2.93μs · p99 3.69μs (목표 < 30μs) ✓ |
| WCET (horizon=30) | mean 5.93μs · p99 7.67μs |
| Heap-free (predict<30> ×1000) | malloc 0 회 ✓ |
| Jitter ratio (σ/mean, horizon=15) | 0.079 (목표 < 0.1) ✓ |

빌드 + 테스트 명령:
```bash
cd /home/leedonghyuck/ros2_ws
colcon build --packages-select trajectory_prediction
source install/setup.bash
colcon test --packages-select trajectory_prediction --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/trajectory_prediction
```

---

## 7. 검증 파이프라인 (실시간 + 사후분석)

### 7.1 실시간 단계 (CSV 한 행에 측정 + 예측 동시 기록)

| 컴포넌트 | 주기 | 역할 |
|---|---|---|
| `main.cpp` wall_timer | 10 Hz | predict 호출 (45점 = 4.5초 미래) → logger->push |
| `TrajectoryLogger::onTick` | 50 Hz | CSV 한 줄 fprintf, 13 + 7×45 = 328 컬럼 |

CSV 한 행 구조:
- 측정: `x, y, z, vn, ve, vd, yaw, true_airspeed`
- 예측 45점: `p_pn_0 ~ p_phi_44` (각 7 채널, t+0.1, t+0.2, ..., t+4.5s)

### 7.2 사후분석 (chunk_analysis.py)

위 §4.11 참조. 4.5초 간격 chunk 추출 → 45 페어 비교 → RMSE 표 + 2 PNG.

### 7.3 색 규칙 (★ 사용자 confirmed)

`chunk_trajectories.png`:
- **녹색 실선** = measured (실제)
- **빨간 점선** = predicted (예측)
- 검정 ● = 시작점
- 녹색 ■ = 실제 끝
- 빨간 ▲ = 예측 끝

`chunk_error_curves.png`: 색은 chunk 번호 구분용 (`tab10` cmap). measured/predicted 구분 자체가 없음 (오차만 그림).

---

## 8. SITL 환경 — 상세 (collision_avoidance/scripts/launch_5vtol.sh 패턴)

### 8.1 왜 multi-vehicle 환경변수가 필수인가

PX4 v1.17 + `px4_ros2_cpp` 라이브러리:
- `make px4_sitl gazebo-classic_standard_vtol` (single-vehicle): 토픽 namespace `/fmu/out/...` (prefix 없음). doRegister 시 `register_ext_component_reply` 위치 mismatch → **handshake 실패**
- multi-vehicle 패턴: 토픽 `/px4_0/fmu/out/...` + 노드 `topic_namespace_prefix=/px4_0` → namespace 일치 → **handshake 성공**
- spawn 기체 수: **두 경우 다 1대** (multi-vehicle 이름은 misnomer, 그냥 namespace prefix 빌려옴)
- MicroXRCEAgent: multi-vehicle 패턴은 별도 기동 필요 (`make` 는 안 띄움)

### 8.2 부팅 5단계 의존성

```
1. MicroXRCEAgent (UDP 8888)        ← ROS2 ↔ PX4 bridge 필수
2. gzserver headless                  ← Gazebo 시뮬 + TCP 11345 master
3. PX4 (env var 셋팅)                 ← TCP 4560 mavlink wait
4. SDF spawn                          ← gzserver 에 모델 추가, mavlink plugin 이 PX4 와 TCP 4560 연결
5. uXRCE-DDS 토픽 발행 시작           ← PX4 가 시뮬 연결 완료 후 자동
```

### 8.3 ready check 의 중요성

PX4 가 시뮬 연결 → uXRCE-DDS 등록까지 **불확정 시간** (3~15초). 이 동안 토픽 발행 안 됨.
trajectory_replay_node 가 너무 일찍 시작하면 `doRegister() 15초 timeout`.

→ **확실한 신호**: `/px4_<id>/fmu/out/register_ext_component_reply_v1` 토픽이 `ros2 topic list` 에 보이면 ready.
launch_1vtol_replay.sh 의 step 6 polling 이 이걸 막아줌.

### 8.4 알려진 트러블 + 해결

| 증상 | 원인 | 해결 |
|---|---|---|
| `doRegister() 15초 timeout` | PX4 ↔ uXRCE-DDS 미연결 | ready check polling 추가 |
| `Quad-chute triggered` (VTOL 천이 실패) | attitude estimator 초기화 글리치 | dataman 캐시 정리 + 재기동 |
| `Cannot have value before ros__parameters at line 20` | `setpoint_sequence.yaml` 을 `--params-file` 로 잘못 넘김 | 그 파일은 SetpointSequencer 가 직접 읽음. `--params-file` 제거 |
| `set -u` 가 setup_gazebo.bash source 시 셸 죽음 | sourced bash 가 미정의 env 사용 | launch script 에서 `set -u` 제거 |
| `param: command not found` (bash 셸에서) | PX4 콘솔이 아닌 일반 bash | PX4 가 실행된 터미널 (pxh>) 에서 입력 |
| `gz model` silently 실패 | `GAZEBO_MODEL_PATH` env 미설정 | `source setup_gazebo.bash` 먼저 |

### 8.5 정리 (cleanup) 전체 명령

```bash
pkill -KILL -f "trajectory_replay_node|px4 -i 0|gzserver Tools/simulation|MicroXRCEAgent udp4 -p 8888" 2>/dev/null
sleep 2
rm -f /tmp/px4_lock-* /tmp/px4-sock-* /tmp/px4_inst*.log /tmp/gzserver.log /tmp/micro_xrce_agent.log 2>/dev/null
# 추가: dataman 캐시 (Quad-chute 방지)
rm -rf /home/leedonghyuck/PX4-Autopilot/build/px4_sitl_default/rootfs/0/dataman 2>/dev/null
```

---

## 9. PX4 파라미터 — 1040_gazebo-classic_standard_vtol

### 9.1 1040 기체가 명시적으로 set 하는 FW 관련 파라미터
파일: `/home/leedonghyuck/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1040_gazebo-classic_standard_vtol`

```
FW_AIRSPD_MAX 25
FW_THR_ASPD_MAX 0.4
NPFG_PERIOD 12
FW_PR_FF 0.2
FW_PR_P 0.9
FW_PSP_OFF 2
FW_P_LIM_MIN -15
FW_RR_FF 0.1
FW_RR_P 0.3
FW_THR_TRIM 0.25
FW_THR_MAX 0.6
FW_THR_MIN 0.05
FW_T_CLMB_MAX 8
FW_T_SINK_MAX 2.7
FW_T_SINK_MIN 2.2
```

### 9.2 1040 기체가 set 안 하는 → PX4 default 그대로
| 파라미터 | PX4 default | 의미 |
|---|---|---|
| `FW_T_TAS_TC` | 5.0 | airspeed error TC [s] |
| `FW_T_ALT_TC` | 5.0 | altitude error TC [s] |
| `FW_R_TC` | 0.4 | roll TC [s] |
| `FW_R_LIM` | 50 | max roll [°] |
| `FW_AIRSPD_MIN` | 10 | min airspeed [m/s] |

소스: `~/PX4-Autopilot/src/modules/fw_lateral_longitudinal_control/fw_lat_long_params.yaml`, `~/PX4-Autopilot/src/modules/fw_att_control/fw_att_control_params.yaml`, `~/PX4-Autopilot/src/modules/fw_mode_manager/fw_mode_manager_params.yaml`.

### 9.3 confirmed live readout (2026-05-11, pxh>)
```
pxh> param show FW_T_TAS_TC      → 5.0000
pxh> param show FW_T_ALT_TC      → 5.0000
pxh> param show FW_R_TC          → 0.4000
pxh> param show FW_R_LIM         → 50.0000
pxh> param show FW_AIRSPD_MIN    → 10.0000
pxh> param show FW_AIRSPD_MAX    → 25.0000
```

---

## 10. ZOH vs Schedule-aware (자세히)

### 10.1 ZOH 의 의미

**Zero-Order Hold** = 제어이론 용어. "샘플 사이 입력값을 직전 값 그대로 freeze".
여기 맥락: 4.5초 동안 setpoint 가 변하지 않는다고 가정하고 단일 setpoint 1개를 45번 stepRK4 모든 stage 에 똑같이 넣음.

### 10.2 두 모드의 PX4 단계 매핑

| 단계 | mode | sequencer 시간 | predict 입력 |
|---|---|---|---|
| 이륙 / VTOL→FW 천이 / 안정화 | `VtolPreflightMode` | **NaN (시작 안 됨)** | ZOH (현재 setpoint) |
| 사용자 시퀀스 비행 | `TrajectoryReplayMode` (활성) | 0초부터 진행 | schedule-aware (미래 lookup) |

### 10.3 코드 흐름 (main.cpp 람다)

위 §4.5 참조.

핵심:
```cpp
const double t_now = logger->getReplayTime();   // Replay 비활성이면 NaN

if (std::isfinite(t_now) && sequencer_for_predict) {
    // Schedule-aware: 매 stage 마다 sequencer.lookup
} else {
    // ZOH fallback
}
```

### 10.4 비유

- ZOH = "지금 GPS 에 표시된 속도/경로가 4.5초 후에도 똑같다고 가정한 길찾기"
- Schedule-aware = "경로 계획서 옆에 두고 4.5초 후 어떤 코너 도는지 알면서 하는 길찾기"

---

## 11. 환경 / 의존성

### 11.1 PX4
- 위치: `/home/leedonghyuck/PX4-Autopilot/`
- 버전: v1.17 (최신)
- 빌드: `make px4_sitl gazebo-classic_standard_vtol` 1회 완료
- 빌드 위치: `build/px4_sitl_default/`

### 11.2 ROS2
- Distro: Humble
- 워크스페이스: `/home/leedonghyuck/ros2_ws/`
- 빌드: `colcon build --packages-select trajectory_prediction` 후 `source install/setup.bash`

### 11.3 Gazebo
- gazebo-classic 11.10.2
- gazebo_mavlink_interface plugin (PX4 빌드 시 함께 빌드)

### 11.4 MicroXRCEAgent
- apt 설치, `/usr/local/bin/MicroXRCEAgent`

### 11.5 Python
- 시스템 python3: pandas 없음
- ★ **`/home/leedonghyuck/anaconda3/bin/python3` 사용** ★ (pandas, numpy, matplotlib 모두 있음)

### 11.6 px4-ros2-interface-lib
- 위치: `/home/leedonghyuck/px4-ros2-interface-lib-main/`
- 외부 모드 등록용 라이브러리 (ModeBase, ModeExecutorBase, doRegister)

---

## 12. 메모리 (영속, 새 세션도 자동 로딩됨)

### 12.1 인덱스
`~/.claude/projects/-home-leedonghyuck/memory/MEMORY.md`

```
- [user_profile.md] — PX4 고정익 + ROS2 오프보드 제어 개발 중인 사용자
- [project_px4_ros2_setup.md] — PX4/ROS2 설치 경로, 버전, 워크스페이스 구조
- [project_fw_offboard.md] — 고정익 오프보드 모드 setpoint 종류 및 제어기 파이프라인
- [project_collision_avoidance.md] — collision_avoidance 패키지 구조, 진행 상황
- [feedback_coding_style.md] — 사용자 코딩 스타일 선호
- [project_fvo_cbf_snapshot_grid.md] — fvo_cbf snapshot grid
- [project_trajectory_prediction.md] — phi 기반 PATCH 적용 + PX4 SITL live 시정수 동기화 (tc_alt 2.0→5.0) 완료, 단위테스트 12/12 통과
```

### 12.2 핵심 메모리 본문
`~/.claude/projects/-home-leedonghyuck/memory/project_trajectory_prediction.md`
- 위 §5 의 진화 history 모두 반영됨
- "★ PX4 SITL live 시정수 동기화 (2026-05-11)" 섹션 포함
- 7-state POD 마지막 멤버 `phi` (PATCH 후) 명시
- 단위테스트 12/12 (phi-PATCH 2개 추가) 반영

### 12.3 코딩 스타일 메모리
`~/.claude/projects/-home-leedonghyuck/memory/feedback_coding_style.md`
- 학습 지향, 코드 흐름 / 동작 원리 자세한 설명 선호
- 실제 동작 검증 중시 (단위테스트 + SITL)
- 헤더 코멘트로 명세 출처 명시 선호
- 분기 회피 (fmin/fmax, fmod) 같은 RT 베스트 프랙티스 선호

---

## 13. 다음 단계 (★ 미완료, 새 세션 첫 작업 ★)

### 13.1 [최우선] 검증 비행

**상태**: launch_1vtol_replay.sh 의 `set -u` 버그 수정 직후 사용자가 재실행 안 함. 새 세션에서 바로 실행.

```bash
~/ros2_ws/src/trajectory_prediction/scripts/launch_1vtol_replay.sh
```

**확인할 메시지** (모두 떠야 정상):
1. `[launch 1/7]` ~ `[launch 6/7]` 단계별
2. `[launch]   → ready (대기 N s)` ← uXRCE-DDS 토픽 polling 통과
3. `[launch 7/7] trajectory_replay_node 실행...`
4. 노드 로그: `[main] Predictor: horizon=4.50s rate=10.0Hz call=10.0Hz (N=45 dt=0.100s tau_V=5.00 tau_hdot=5.00 tau_phi=0.40)` ← **`tau_V=5.00 tau_hdot=5.00 tau_phi=0.40` 가 동기화 적용 핵심 신호**
5. `[main] doRegister() 성공 → spin 시작`
6. PX4 콘솔: `Ready for takeoff!` → `Takeoff detected` → VTOL 천이 → cruise
7. 약 50~55초 후 시퀀스 완료 → Ctrl+C 로 노드 종료 → trap 이 모든 자식 정리

**실패 시 fallback**:
- 또 `Quad-chute` 발생: `rm -rf ~/PX4-Autopilot/build/px4_sitl_default/rootfs/0/dataman` 후 재기동
- `doRegister() timeout`: launch script 의 ready check 가 30초 내 못 잡으면 PX4 ↔ Agent 연결 자체 실패. 수동 진단 필요.
- `set -u` 같은 새 셸 버그: launch_1vtol_replay.sh 다시 검토

### 13.2 [검증 끝나면] chunk_analysis 비교

```bash
mkdir -p /tmp/tc_synced
/home/leedonghyuck/anaconda3/bin/python3 \
  ~/ros2_ws/src/trajectory_prediction/scripts/chunk_analysis.py \
  --out-dir /tmp/tc_synced/
```

**비교 대상**:
| 산출물 | 모델 / 시정수 |
|---|---|
| `/tmp/horizontal/chunk_*.png` | a_lat 기반 + tc_alt=2.0 (이전, baseline) |
| `/tmp/phi/chunk_*.png` | phi 기반 + tc_alt=2.0 (PATCH 적용, 동기화 전) |
| `/tmp/tc_synced/chunk_*.png` | ★ phi 기반 + tc_alt=5.0 (★ 새 데이터 ★) |

**예상 효과**:
- 수평 시퀀스 (h_dot=0) 라 tc_alt 영향 거의 없음 → 대조군. 큰 변화 안 보일 것.
- tc_tas (4.0 → 5.0) / tc_roll (0.5 → 0.4) 미세 변화 정도 기대.
- 결정적 검증은 §13.3 의 상승/강하 시퀀스 추가 후.

**보고 형식 (이전 예시)**:
| chunk | tc_alt=2.0 psi end | tc_alt=5.0 psi end | 개선 % |
|---|---|---|---|
| 2 | 5.06° | ?° | ? |
| 3 | 4.75° | ?° | ? |
| 4 | 3.76° | ?° | ? |

### 13.3 [선택 / 권장] 상승/강하 시퀀스 추가
`~/ros2_ws/src/trajectory_prediction/config/setpoint_sequence.yaml` 일부 segment 에 `height_rate: 2.0` 또는 `-1.5` 추가:

```yaml
sequence:
  - { t_start:  0.0, t_end: 10.0, airspeed: 16.0, height_rate:  0.0, lateral_acceleration:  0.0 }
  - { t_start: 10.0, t_end: 20.0, airspeed: 16.0, height_rate:  2.0, lateral_acceleration:  0.0 }   # 상승
  - { t_start: 20.0, t_end: 30.0, airspeed: 18.0, height_rate: -1.5, lateral_acceleration:  4.0 }   # 하강 + 우선회 (종/횡 결합)
  - { t_start: 30.0, t_end: 40.0, airspeed: 16.0, height_rate:  0.0, lateral_acceleration:  0.0 }
```

→ tc_alt 효과가 chunk 발산에 명확히 나타남. 이게 동기화 전후 비교의 결정적 검증.

### 13.4 [완료 후] 메모리 / handoff 문서 마지막 갱신
- `project_trajectory_prediction.md` 에 검증 결과 추가
- 본 문서 (SESSION_TRANSPLANT) 의 §13 status 업데이트 ("미완료" → "완료")
- 단기 handoff 문서 (`HANDOFF_2026-05-11_tc_sync_verification.md`) 도 결과 반영 또는 archive

### 13.5 [후순위] collision_avoidance::FormationMode::rt_loop 통합
별도 plan 필요. 현재 trajectory_replay_node 의 wall_timer 어댑터를 rt_loop 안으로 이전. mlockall + SCHED_FIFO 부여.

---

## 14. TODO(question) — 향후 결정 필요

1. **PredictState 의 V/psi/h_dot/phi 출처**
   - 현재: `V=true_airspeed`, `psi=yaw`, `h_dot=-vd`, `phi=atan2(setpoint a_lat, g)`
   - phi 도 측정값 (vehicle_attitude quaternion → roll) 으로 바꾸는 게 정확. 추후 도입 검토.
2. **h_dot_max 비대칭** — climb (+8.0) / sink (−2.7) 비대칭이지만 PredictParams 는 단일 값.
   - 현재: 보수적 `min(climb, |sink|) = 2.7`
3. **CSV 컬럼명 변경 사후분석 호환성** — Python 분석 스크립트가 `p_x_k` (구) → `p_pn_k` (신) → `p_phi_k` (신) 헤더 변경 반영 필요. chunk_analysis.py 는 자동 분기 처리됨.
4. **setParams 가 public** — rt_thread 도중 호출 금지가 주석만으로 보호됨. 추후 private 화 또는 lifecycle 가드 검토.
5. **첫 데이터 행 sp NaN guard** — Replay 활성화 ~10ms 직전 한 행에서 sp_*=NaN 이 onTick 에 잡혀 그 시점 predict 의 a_lat 채널 NaN 전파 (1행만 영향). 사후분석 시 첫 행 skip 또는 main.cpp 어댑터에 NaN 가드 검토.

---

## 15. 검증 산출물 (현재까지)

| 디렉토리 | 시각 | 분석 대상 |
|---|---|---|
| `/tmp/` | 13:55 | 초기 ZOH 모드 (schedule-aware 적용 전) |
| `/tmp/aware/` | 14:18 | schedule-aware 적용 후 (chunk 2 psi 39° → 2.55°) |
| `/tmp/horizontal/` | 14:27 | 수평 only 시퀀스 (a_lat 기반, tc_alt=2.0) |
| `/tmp/phi/` | 14:57 | phi 기반 PATCH + 수평 시퀀스 (tc_alt=2.0) |
| `/tmp/tc_synced/` | (미생성) | ★ 새 세션에서 생성할 데이터 (tc_alt=5.0) |

각 디렉토리에 두 PNG:
- `chunk_error_curves.png`: 4 subplot (XY/h/V/psi 오차 시간곡선, chunk 별 색)
- `chunk_trajectories.png`: chunk 별 XY top-view (녹색 measured / 빨간 predicted)

CSV:
- `/tmp/trajectory_*.csv`: 50Hz 기록 파일들 (시간순). 가장 최근 것이 chunk_analysis 의 default 입력.

---

## 16. 점검 에이전트

`~/ros2_ws/.claude/agents/design-compliance-checker.md`
- description 에 "PROACTIVELY USE" 명시
- .cpp/.hpp 수정 후 자동 호출
- 7 카테고리 점검 (네이밍 / thread / rt_loop / ModeBase / fallback / heap0 / sub-pub)
- 2026-05-11 점검 결과: 6 위반 중 4 fix 적용 (wrapPi 분기 제거, applyStateSafety 주석 명확화, `_mt` 접미사, valid 플래그 처리). 나머지 2개 (RK4 중간단계 saturation / mutex 보유 중 fprintf) 는 디자인 트레이드오프로 현 유지.

---

## 17. 명령어 cheat sheet

### 17.1 빌드
```bash
cd /home/leedonghyuck/ros2_ws
colcon build --packages-select trajectory_prediction
source install/setup.bash
```

### 17.2 단위테스트
```bash
colcon test --packages-select trajectory_prediction --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/trajectory_prediction
```

### 17.3 검증 비행 (★ 한 줄)
```bash
~/ros2_ws/src/trajectory_prediction/scripts/launch_1vtol_replay.sh
```

### 17.4 사후분석
```bash
mkdir -p /tmp/tc_synced
/home/leedonghyuck/anaconda3/bin/python3 \
  ~/ros2_ws/src/trajectory_prediction/scripts/chunk_analysis.py \
  --out-dir /tmp/tc_synced/
```

### 17.5 정리 (수동)
```bash
pkill -KILL -f "trajectory_replay_node|px4 -i 0|gzserver Tools/simulation|MicroXRCEAgent udp4 -p 8888" 2>/dev/null
sleep 2
rm -f /tmp/px4_lock-* /tmp/px4-sock-* /tmp/px4_inst*.log 2>/dev/null
rm -rf /home/leedonghyuck/PX4-Autopilot/build/px4_sitl_default/rootfs/0/dataman 2>/dev/null
```

### 17.6 PX4 콘솔 직접 띄우기 (param 확인용)
```bash
cd /home/leedonghyuck/PX4-Autopilot
HEADLESS=1 make px4_sitl gazebo-classic_standard_vtol
# pxh> 프롬프트에서
param show FW_T_TAS_TC
param show FW_T_ALT_TC
param show FW_R_TC
shutdown
```

---

## 18. Out of Scope

다음은 본 세션 / 다음 단계 모두 다루지 않음 (별도 plan 필요):
- collision_avoidance::FormationMode::rt_loop 통합 (호출자 변경)
- mlockall + SCHED_FIFO 부여 (호출자 책임, 명세 §6.2)
- PREEMPT_RT 커널 부여 후 cyclictest/ftrace 재측정
- 라즈베리파이 onboard 노드 포팅
- 풍속 모델 / 신경망 보강
- 측정 phi (vehicle_attitude → roll) 도입

---

## 19. 부록 — 본 세션의 사용자 코멘트 패턴 (대화 흐름 복원용)

본 세션에서 사용자가 한 핵심 한국어 발화 (의도 파악용):
- "지금 여기서 시상수 part 가 실제로 지금 내가 시뮬레이션 하고 있는 기체에서 정의된 것하고 일치하는 지 확인이 가능한가?" → tc 동기화 필요성 발견 시작점
- "지금 여기서 말하는 실제 값이라는 것이 혹시 지금 가제보에 실제로 돌아가는 기체를 의미하는 것인가?" → 저장된 .params 파일이 live 와 다를 수 있음 인지
- "ㅇㅇ 수정해줘" → airframe_spec.yaml 수정 OK
- "3번" → 메모리 갱신 우선
- "지금 일단 비행기의 실제 궤적은 초록색으로 예측한 궤적은 지금 빨간색으로 표현을 해줘" → 색 규칙 confirmed
- "ㅇㅇ 일단 검증을 한 번 해봐바" → 검증 비행 실행 시작
- "왜 지금 궤적 을 예측 하는 것은 단일 기체를 기준으로 하는 것이 아니었나?" → multi-vehicle 환경변수 misnomer 설명 트리거
- "지금 느린 이유는? 그냥 명령어를 주면은 내가 치는 것이 토큰하고 더 빠르나?" → 사용자 직접 실행 모드 선호 확인
- "지금 궁금한게 왜 이렇게 복잡하게 실행하는 것을 만들어 놓음?" → launch wrapper 작성 트리거
- "지금 여기서 니가 만약에 되면은 지금 그걸 1대용으로 줄인 launch_1vtol_replay.sh 를 만들어줘" → wrapper 작성 OK
- "지금 어떻게 실행을 시키면 된다고? 그냥 명령어르 ㄹ나한테 줘ㅏ" → 한 줄 명령 선호
- "지금 그냥 이렇게 나오는데?" → set -u 버그 진단 트리거
- "내가 원하는 것은 지금 현재 세션 전체를 이식시킬 수준의 md 파일이 필요한데" → 본 문서 작성 트리거

→ 다음 세션에서 사용자가:
- 모호한 한 줄 → 의도 빠르게 파악해서 실행
- 복잡한 단계 → wrapper / 한 줄 명령으로 압축 선호
- 검증 결과 → 표로 정리, 색/단위 명확히

---

## 20. 마무리 — 새 세션 시작 직후 권장 액션

1. 본 문서 (§1 ~ §19) 읽기
2. `project_trajectory_prediction.md` 메모리 읽기
3. 사용자에게 한 줄: "이 문서 기준 §13.1 검증 비행부터 시작하면 될까요?" 확인
4. OK 시: `launch_1vtol_replay.sh` 실행 명령만 사용자에게 주고 실행 결과 (CSV 생성 + Ctrl+C) 기다림
5. CSV 생성 후: §13.2 chunk_analysis 자동 실행 + 비교 표 작성
6. 결과 보고 + §13.3 (상승/강하 시퀀스) 진행 여부 확인
7. 모두 완료 후: §13.4 메모리 / 본 문서 갱신

→ 본 문서가 진실. 코드와 디스크 상태가 본 문서와 다르면 디스크가 우선이지만, 디자인 의도는 본 문서를 따라야 함.
