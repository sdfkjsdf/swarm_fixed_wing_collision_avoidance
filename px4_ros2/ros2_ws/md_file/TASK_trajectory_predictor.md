# TASK: TrajectoryPredict — 7-상태 동역학 예측기 명세 / 점검 / 보강

> **수신**: Claude Code
> **발신**: 가이던스 알고리즘 설계자
> **대상 파일**: `/home/leedonghyuck/ros2_ws/src/trajectory_prediction/`
> **상태**: `TrajectoryPredict.hpp/.cpp` 가 이미 존재 (`PROJECT_FILE_DIAGRAM.md` §3 참조).
>          이 명세는 **새로 만드는 것이 아니라 명세에 맞춰 점검·보강·단위테스트** 함.
> **★ 호출 컨텍스트 (중요) ★**: 본 클래스는 **rt_thread 안에서 직접 호출**된다.
>          collision_avoidance 의 FormationMode 패턴과 동일한 rt_loop 1ms 주기.
>          기존 trajectory_prediction 의 wall_timer 10Hz 경로는 검증/리플레이 단계의 임시 구조였음.
>          **본 명세는 rt_thread 컨텍스트를 전제로 작성됨.**
> **범위**: 주어진 (PredictState, PredictInput) → 미래 궤적 N step 예측.
>          호출자가 어떤 입력을 어디서 가져오는지, 몇 회 부르는지는 본 클래스 범위 밖.
> **금지사항**: 본 명세에 없는 기능 추가 금지. 의문 사항은 `TODO(question):` 코멘트로 남기고 진행할 것.

---

## 1. 컨텍스트 (Why this module exists)

분산 충돌회피 알고리즘은 **무인기의 미래 궤적을 짧은 horizon (1~3초) 예측**할 수 있어야 한다.
본 클래스는 그 예측기 본체이며, 호출 컨텍스트는 **rt_thread (1ms 주기)** 임을 전제로 한다.

호출자(예: `FormationMode::rt_loop`)가 다음을 제공:
- 현재 상태 `PredictState`
- 입력(setpoint) `PredictInput` (zero-order hold 가정)
- horizon step 수 (템플릿 N), step 크기 dt

본 클래스는 stateless / heap-free / lock-free 로 동작.

---

## 2. 모델 — Beard & McLain (2012) 식 (9.19) 변형판

### 2.1 상태 (7개)

```cpp
struct PredictState {
    double p_n;     // [m]    NED 북쪽 위치
    double p_e;     // [m]    NED 동쪽 위치
    double h;       // [m]    고도 (위로 양수)
    double V;       // [m/s]  속력
    double psi;     // [rad]  방위각
    double h_dot;   // [m/s]  현재 상승률 (자동조종 추종 중)
    double a_lat;   // [m/s²] 현재 횡가속도 (자동조종 추종 중)
};
// sizeof = 56 byte = 1 cache line (64B) 안에 들어감 ✓
```

### 2.2 입력 (3개) — `FwSetpointOutput_rt2mt` 와 동일 단위

```cpp
struct PredictInput {
    double V_cmd;        // [m/s]   ↔ FwSetpointOutput_rt2mt::airspeed
    double h_dot_cmd;    // [m/s]   ↔ FwSetpointOutput_rt2mt::height_rate
    double a_lat_cmd;    // [m/s²]  ↔ FwSetpointOutput_rt2mt::lateral_acceleration
};
```

**중요**: 입력 단위가 collision_avoidance 의 출력 자료형 `StateType::FwSetpointOutput_rt2mt` 와 완전 일치.
이 매칭을 깨지 마라.

### 2.3 연속시간 ODE

```
p_n_dot   = sqrt(V^2 - h_dot^2) * cos(psi)
p_e_dot   = sqrt(V^2 - h_dot^2) * sin(psi)
h_dot_pos = h_dot                                  // 위치 h의 미분 = 상태 h_dot
V_dot     = (V_cmd - V) / tau_V
psi_dot   = a_lat / sqrt(V^2 - h_dot^2)            // 조정선회 가정
h_ddot    = (h_dot_cmd - h_dot) / tau_hdot
a_lat_dot = (a_lat_cmd - a_lat) / tau_a
```

### 2.4 시정수 (초기값; PX4 SITL 식별로 교체 예정)

| 채널 | 초기값 | PX4 대응 파라미터 |
|---|---|---|
| `tau_V`    | 4.0 s | `FW_T_TAS_TC` (airspeed error TC) |
| `tau_hdot` | 2.0 s | `FW_T_ALT_TC` (altitude error TC)  |
| `tau_a`    | 0.5 s | `FW_R_TC` (roll TC, default 0.5s) |

값은 `config/airframe_spec.yaml` 또는 `config/replay_params.yaml` 에서 외부 주입.
**rt_thread 가 도는 중에는 값이 바뀌지 않는다고 가정** (initialization-time only).

### 2.5 입력 saturation (ODE 평가 전에 적용)

```
V_cmd      ← clamp(V_cmd, V_min, V_max)
h_dot_cmd  ← clamp(h_dot_cmd, -h_dot_max, +h_dot_max)
a_lat_cmd  ← clamp(a_lat_cmd, -a_lat_max, +a_lat_max)

// 수치 안전:
if |h_dot| > V then h_dot ← sign(h_dot) * V * 0.99
V_h ← max(sqrt(V^2 - h_dot^2), V_h_min)   // V_h_min = 1.0 m/s
```

한계값은 `airframe_spec.yaml` 의 PX4 파라미터에서 유도:
- `V_min/V_max` ← `FW_AIRSPD_MIN/MAX`
- `h_dot_max` ← `FW_T_CLMB_MAX`, `FW_T_SINK_MAX`
- `a_lat_max` ← `g * tan(FW_R_LIM)`

**구현 권고**: 분기 (if/clamp) 를 가능한 한 분기 없는 산술 (fmin/fmax) 으로 작성.
이유: rt_thread 에서 branch predictor miss 가 jitter 원인이 될 수 있음.

---

## 3. 출처 / 정당화 (헤더 코멘트로 명시)

```cpp
// Reference:
//   Beard, R. W., & McLain, T. W. (2012).
//   "Small Unmanned Aircraft: Theory and Practice", Princeton University Press.
//   Equation (9.19) — reduced-order guidance model.
//
// Modifications from Beard-McLain (9.19):
//   1) Lateral channel: roll command (phi_c) replaced by lateral acceleration
//      command (a_lat_cmd). Equivalence: a_lat = g * tan(phi).
//      Reason: matches FwSetpointOutput_rt2mt::lateral_acceleration unit.
//   2) Altitude channel: PD form reduced to 1st-order lag (b_h = 0).
//
// PX4 parameter mapping for time constant identification:
//   tau_V    <-> FW_T_TAS_TC
//   tau_hdot <-> FW_T_ALT_TC
//   tau_a    <-> FW_R_TC (default 0.5s)
//
// Caveat: PX4 time constants are "error convergence" TCs, not strictly
//   open-loop plant TCs. Final tau values shall be identified via SITL
//   step-response data.
```

---

## 4. 아키텍처 — 사용자 기존 컨벤션 준수

### 4.1 의존성 분리 원칙 (FlockingGuidance 와 같은 패턴)

본 클래스는 **ROS2 / px4_msgs / DDS 일체 의존하지 않는다.** 순수 C++17.

이유: `FlockingGuidance` 가 정확히 같은 패턴이고, 단위테스트·이식성·RT 안전성 모두 보장됨.

- 입력: POD `PredictState`, `PredictInput`
- 출력: `std::array<PredictState, N>` (정적 크기, 호출자가 멤버에 보유)
- 호출자가 큐 pop 결과 → POD 변환 담당

### 4.2 디렉토리 (기존 그대로 유지)

```
src/trajectory_prediction/
├── include/trajectory_prediction/
│   ├── TrajectoryPredict.hpp         ← ★ 본 명세 대상 ★
│   ├── StateType.hpp                 ← PredictState/PredictInput/PredictParams 정의 위치
│   └── ...
├── src/
│   └── TrajectoryPredict.cpp         ← ★ 본 명세 대상 ★
├── config/
│   ├── airframe_spec.yaml            ← 한계값
│   └── replay_params.yaml            ← 시정수, horizon, dt
└── test/
    └── test_trajectory_predict.cpp   ← (없으면 생성)
```

### 4.3 인터페이스 (제안)

```cpp
// TrajectoryPredict.hpp
#pragma once
#include <array>
#include <cstddef>

namespace trajectory_prediction {

struct PredictState { /* §2.1 */ };
struct PredictInput { /* §2.2 */ };

struct PredictParams {
    // 시정수
    double tau_V       = 4.0;
    double tau_hdot    = 2.0;
    double tau_a       = 0.5;
    // 입력 한계
    double V_min       = 12.0;
    double V_max       = 25.0;
    double h_dot_max   = 5.0;
    double a_lat_max   = 9.8;     // g * tan(45°)
    // 수치 안전
    double V_h_min     = 1.0;
};

class TrajectoryPredict
{
public:
    explicit TrajectoryPredict(const PredictParams& params);

    /* 단일 step RK4 — input은 zero-order hold 가정 */
    PredictState stepRK4(const PredictState& x,
                         const PredictInput& u,
                         double dt) const;

    /* 다중 step 예측 — out_traj 는 호출자가 미리 정적 할당
     * 호출자(rt_thread) 가 멤버 std::array 로 보유 권장 → stack page fault 회피 */
    template <std::size_t N_STEPS>
    void predict(const PredictState& x0,
                 const PredictInput& u_zoh,
                 double dt,
                 std::array<PredictState, N_STEPS>& out_traj) const;

    /* 파라미터 갱신은 initialization-time only (rt_thread 시작 전) */
    void setParams(const PredictParams& params);

private:
    PredictParams m_params;   // rt_thread 시작 후 변경 금지

    PredictState evaluateODE(const PredictState& x,
                             const PredictInput& u) const;
    PredictState applyStateSafety(const PredictState& x) const;
    PredictInput applyInputSaturation(const PredictInput& u) const;
};

} // namespace trajectory_prediction
```

**`const` 보장**: `stepRK4`, `predict`, `evaluateODE` 모두 const.
**stateless**: 내부 state 변경 없음 → 같은 입력 → 같은 출력 (단위테스트 검증 항목).

---

## 5. 호출 컨텍스트 — rt_thread 통합

### 5.1 호출 예시

```cpp
// 호출자 (rt_loop) 예시:
m_predictor->predict(current_state, current_setpoint_input,
                     m_predict_dt, m_predicted_traj_rt);
```

**핵심**: `m_predicted_traj_rt` 는 **호출자의 멤버 std::array** 로 보관.
**stack 할당 금지** — 함수 매 호출마다 큰 stack 사용 → page fault 위험.

### 5.2 호출자가 갖춰야 할 것 (참고; 본 클래스 책임 아님)

```cpp
// 호출자 클래스에서:
static constexpr std::size_t kPredictHorizon = 15;   // 1.5초 (dt=0.1)
static constexpr double kPredictDt = 0.1;

std::unique_ptr<TrajectoryPredict> m_predictor;
std::array<PredictState, kPredictHorizon> m_predicted_traj_rt;
```

**WCET 친화**: array 가 객체 생성 시 한 번 할당, rt_loop 호출마다 in-place 덮어쓰기.

---

## 6. RTOS 관점 — rt_thread 컨텍스트 (★ 본 명세의 핵심 절 ★)

### 6.1 한 predict 호출의 WCET

```
1 step RK4 = 4 × (ODE eval) + 1 × (가중평균)
ODE eval: ~10 floating ops (cos/sin/sqrt 포함)
1 step ≈ ~수 μs (라즈베리파이 5 추정)

horizon × step_cost = 총 predict 비용
```

**보고 의무**: 단위 테스트에서 horizon별 (5, 10, 15, 30 step) WCET 표 제공.
호출자가 자기 budget 안에서 이 표를 보고 horizon 선택.

### 6.2 결정성 보장 요구사항

| 위험 | 대응 |
|---|---|
| Heap allocation | ✓ 정적 array, POD 만 사용 |
| Lock contention | ✓ stateless 설계, 공유 자원 없음 |
| **Stack page fault** | 큰 array 는 호출자 멤버로 (본 클래스 책임 아님). rt_thread 시작 시 `mlockall(MCL_CURRENT\|MCL_FUTURE)` 권장 (호출자 책임) |
| **Branch unpredictability** | clamp 분기를 fmin/fmax 로 치환 |
| **Cache miss** | PredictState=56B → 1 cache line ✓ |
| Atomic 비용 | 본 클래스 내부에는 atomic 0개 |
| **PREEMPT_RT 미부여 (현재 상태)** | 현재 rt_thread = SCHED_OTHER → 측정한 WCET 신뢰도 낮음. PREEMPT_RT + SCHED_FIFO 부여 후 재측정 필수 |

### 6.3 일반 Linux vs PREEMPT_RT 차이 (이 클래스 관점)

| 항목 | 일반 Linux | PREEMPT_RT |
|---|---|---|
| rt_thread 우선순위 | SCHED_OTHER, nice -20 까지만 | SCHED_FIFO/RR, RT priority 1~99 |
| predict 중 preemption | 임의 시점에 발생 가능 | RT prio 낮은 thread 만 preempt |
| 인터럽트 latency | 수 ms 가능 | 수십 μs |
| mutex (만약 쓰면) | non-PI | **자동 PI mutex** |
| 측정 도구 | `time`, `chrono` | `cyclictest`, `ftrace` |

본 클래스는 mutex/atomic 을 안 쓰므로 PI mutex 이슈는 무관. 핵심은 **호출자(rt_thread) 의 스케줄 정책**.

### 6.4 실패 시나리오

- **WCET 초과**: predict가 호출자 budget 넘으면 → rt_loop 가 주기 안에 못 끝남 → setpoint stale → PX4 가 cruise fallback 또는 failsafe
  → 본 클래스는 이걸 감지하지 않음. 호출자가 deadline miss 카운터를 둠.
- **horizon 너무 길면**: 1차 지연 모델은 ~3τ 후 정확도 한계. tau_V=4s → horizon > 12s 의미 없음.
  → 본 클래스는 horizon 상한을 강제하지 않음. 호출자가 템플릿 N으로 결정.

---

## 7. 수치 적분 (RK4)

```
k1 = f(x,        u)
k2 = f(x + dt/2 * k1, u)
k3 = f(x + dt/2 * k2, u)
k4 = f(x + dt   * k3, u)
x_next = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
```

**RK4 후처리**:
- `psi` 는 [-π, π] 로 wrap
- saturation 재적용 (수치오차로 V_h_min 깨질 가능성 차단)

**dt 권장값**: 0.05~0.1 s. rt 환경에서는 0.1s 권장 (step 수 ↓ = WCET ↓).

---

## 8. 단위 테스트 (반드시 작성할 것)

`test/test_trajectory_predict.cpp`:

### 정확성 테스트

1. **정상상태 수렴**: 셋포인트 일정 → 충분한 시간 후 수렴 (오차 < 1%)
2. **1차 응답**: step 입력 후 시간 τ 시점에서 셋포인트의 63.2% 도달
3. **조정선회 원궤도**: a_lat = const, h_dot = 0, V = const → 반경 R = V²/a_lat (오차 < 5%)
4. **직선 등속**: h_dot_cmd = 0, V_cmd = const, a_lat_cmd = 0 → 직선, V 일정
5. **Saturation 작동**: V_cmd > V_max → V는 V_max 로 수렴
6. **수치 안정성**: dt = 0.01 vs dt = 0.1, 1초 horizon 결과 차이 < 5%
7. **Stateless**: 같은 입력 두 번 호출 → 같은 출력 (내부 상태 누적 없음)

### ★ RT 특성 테스트 (rt_thread 가정 시 필수) ★

8. **WCET 측정**: 100,000 회 호출 → max execution time 측정.
   horizon 별 (5, 10, 15, 30 step) 표 제공.
   기준: horizon=15, dt=0.1 에서 라즈베리파이 5 단일 호출 < 100 μs.
   호스트 PC 에서는 < 30 μs 정도면 OK (CPU 차이 보정).
9. **Heap allocation 부재 검증**: AddressSanitizer + malloc hook 으로 확인.
   결과: predict 호출 중 malloc 호출 0회.
10. **Jitter 측정**: 100,000 회 호출의 execution time histogram.
    표준편차가 평균의 10% 이내인지.

테스트 프레임워크는 colcon 통합 가능한 것 (gtest, doctest, 등). 사용자 컨벤션 미확인이므로 가벼운 것 선택.

---

## 9. 빌드 / 검증

```bash
cd /home/leedonghyuck/ros2_ws
colcon build --packages-select trajectory_prediction
source install/setup.bash

# 단위 테스트
colcon test --packages-select trajectory_prediction
colcon test-result --verbose
```

---

## 10. 보고할 것

작성 / 점검 완료 후:

1. 기존 `TrajectoryPredict.hpp/.cpp` 와 본 명세의 diff (있다면)
2. 10개 단위 테스트 PASS/FAIL
3. **WCET 측정 결과** (호스트 PC):
   - mean, median, max, p99 (단위: μs)
   - horizon별 (5, 10, 15, 30 step) 표
4. **Heap allocation 검사 결과** (malloc 호출 횟수)
5. 의문점은 `TODO(question):` 코멘트 모음

---

## 11. 하지 말 것 (Anti-requirements)

- **ROS2 메시지 직접 include 금지**
  (`px4_msgs::msg::*`, `geometry_msgs::msg::*`, `rclcpp/*` 등)
  → 이유: `FlockingGuidance` 가 같은 패턴. 호출자가 변환 책임.
- **디렉토리 구조 변경 금지** (`src/trajectory_prediction/...` 그대로)
- **시정수 하드코딩 금지** (yaml 또는 PredictParams 주입)
- **동적 horizon 금지** (`std::vector<PredictState>` 금지 → 템플릿 N 으로 고정)
- **rt_thread 가 도는 중 setParams 호출 금지** (initialization-time only)
- **함수 안에서 큰 stack array 선언 금지** (호출자가 멤버 array 제공)
- **mutex / lock 사용 금지** (stateless 설계 유지)
- **풍속 모델 추가 금지** (다음 단계 별도 명세)
- **신경망 / 학습 기반 예측기 추가 금지** (본 명세는 분석적 모델만)
- **이웃/swarm 시나리오 가정 금지** (본 클래스는 단일 무인기 1회 호출만 책임. 호출 패턴은 호출자 책임)

---

*작성 기준: Beard & McLain (2012) Ch.9, PX4 v1.15 fixed-wing TECS/NPFG 표준 스택.*
*RT 컨텍스트: rt_thread 1ms 주기, SCHED_FIFO 예정 (현재 SCHED_OTHER).*
*컨벤션 출처: FORMATION_MODE_ARCHITECTURE.md, SUBSCRIBER_DATA_FLOW.md, PROJECT_FILE_DIAGRAM.md (사용자 워크스페이스).*
