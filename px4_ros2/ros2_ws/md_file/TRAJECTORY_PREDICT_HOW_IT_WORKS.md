# TrajectoryPredict — 작동 원리 + 현재 ZOH 한계 + 개선 질문

> **문서 목적**: 현재 구현된 `TrajectoryPredict` 라이브러리의 동작을 정확히 설명하고, 사후분석에서 발견된 "예측이 segment 전환을 못 따라잡는 문제" 가 모델의 본질적 한계인지 또는 호출 패턴의 단순화인지 다른 Claude 에게 묻기 위한 자료.
>
> **수신**: 외부 Claude (또는 새 세션)
> **발신**: 현재 작업 세션
> **첨부 코드 위치**: `/home/leedonghyuck/ros2_ws/src/trajectory_prediction/`
> **명세 ground truth**: `/home/leedonghyuck/ros2_ws/md_file/TASK_trajectory_predictor.md`

---

## 1. 모델 자체 (라이브러리 구현 — Layer A)

### 1.1 자료형

```cpp
namespace trajectory_prediction {

// 7-state PredictState (sizeof == 56B = 1 cache line)
struct PredictState {
    double p_n;     // [m]    NED north
    double p_e;     // [m]    NED east
    double h;       // [m]    altitude (up positive)
    double V;       // [m/s]  airspeed magnitude
    double psi;     // [rad]  heading, wrapped to [-pi, pi]
    double h_dot;   // [m/s]  current climb rate (autopilot tracking)
    double a_lat;   // [m/s²] current lateral acceleration (autopilot tracking)
};

// 3-input PredictInput (PX4 의 FwSetpointOutput 와 단위 1:1)
struct PredictInput {
    double V_cmd;       // [m/s]
    double h_dot_cmd;   // [m/s]
    double a_lat_cmd;   // [m/s²]
};

// 1차 지연 시정수 + saturation 한계
struct PredictParams {
    double tau_V       = 4.0;     // FW_T_TAS_TC
    double tau_hdot    = 2.0;     // FW_T_ALT_TC
    double tau_a       = 0.5;     // FW_R_TC
    double V_min       = 12.0;    // FW_AIRSPD_MIN
    double V_max       = 25.0;    // FW_AIRSPD_MAX
    double h_dot_max   = 5.0;
    double a_lat_max   = 9.8;
    double V_h_min     = 1.0;
};

}
```

### 1.2 ODE — Beard-McLain (2012) 식 (9.19) 변형

```
V_h         = sqrt(V² - h_dot²)            (수평 속도, V_h_min 으로 floor)
p_n_dot     = V_h * cos(psi)               (위치 NED 적분)
p_e_dot     = V_h * sin(psi)
h_dot_pos   = h_dot                        (위치 h 의 미분 = 상태 h_dot)
V_dot       = (V_cmd     - V    ) / tau_V       ★ 1차 지연
psi_dot     = a_lat / V_h                  (조정선회)
h_ddot      = (h_dot_cmd - h_dot) / tau_hdot    ★ 1차 지연
a_lat_dot   = (a_lat_cmd - a_lat) / tau_a       ★ 1차 지연
```

### 1.3 핵심 API — `predict<N_STEPS>()`

```cpp
class TrajectoryPredict {
public:
    explicit TrajectoryPredict(const PredictParams & params);

    // 단일 RK4 step (zero-order hold input 가정)
    PredictState stepRK4(const PredictState & x,
                         const PredictInput & u,
                         double dt) const;

    // ★ N_STEPS 적분 — 단일 입력 ZOH ★
    template <std::size_t N_STEPS>
    void predict(const PredictState & x0,
                 const PredictInput & u_zoh,    // ← 한 번 호출에 입력 1개
                 double dt,
                 std::array<PredictState, N_STEPS> & out_traj) const
    {
        out_traj[0] = x0;
        for (std::size_t k = 0; k + 1 < N_STEPS; ++k) {
            out_traj[k + 1] = stepRK4(out_traj[k], u_zoh, dt);  // ★ 매 step 같은 u_zoh
        }
    }

private:
    PredictState evaluateODE(const PredictState & x, const PredictInput & u) const;
    PredictInput applyInputSaturation(const PredictInput & u) const;
    PredictState applyStateSafety(const PredictState & x) const;
    PredictParams m_params;
};
```

→ **한 번의 `predict()` 호출 = 한 개 입력 + 4.5초 적분 (45 step at dt=0.1)**.

### 1.4 보장 (단위테스트로 검증)

- **Stateless**: 같은 입력 두 번 호출 → bit-exact 같은 출력
- **Heap-free**: predict 호출 중 malloc 0 회 (operator new 카운터)
- **WCET**: horizon=15 p99 = 3.7μs (호스트 PC), horizon=30 p99 = 7.7μs
- **외부 의존 0**: Eigen / rclcpp / px4_msgs 모두 안 씀. 순수 C++17 + std::array

---

## 2. 호출 패턴 (현재 검증 단계 — `main.cpp` wall_timer)

```cpp
// trajectory_replay_node 의 main.cpp
auto predict_timer = node->create_wall_timer(
    100ms,                               // ★ 100ms 마다 1회 호출
    [predictor, logger, predicted_traj, dt]() {
        const auto m  = logger->getCurrentMeasurements();   // 현재 측정값
        const auto sp = logger->getCurrentSetpointSnapshot();  // 현재 setpoint

        // 6-state 측정 → 7-state PredictState 변환
        PredictState x0;
        x0.p_n = m.x; x0.p_e = m.y; x0.h = -m.z;
        x0.V = m.true_airspeed; x0.psi = m.yaw;
        x0.h_dot = -m.vd; x0.a_lat = sp.a_lat;

        // ★ 그 시점 setpoint 1개를 ZOH 로 4.5초 예측 ★
        PredictInput u_zoh{ sp.V, sp.h_dot, sp.a_lat };
        predictor->predict<45>(x0, u_zoh, 0.1, *predicted_traj);

        logger->pushPredictedTrajectory(*predicted_traj);
    });
```

→ 매 100ms 마다:
1. **현재** 측정값 + **현재** setpoint snapshot
2. predict 호출 (입력은 그 시점 setpoint 가 horizon 동안 유지된다는 가정)
3. 결과 45점이 다음 100ms 동안 Logger 에 잠시 저장 → CSV 에 5행 (50Hz tick × 5) 동안 같은 예측이 반복 기록됨
4. 다음 100ms 후 새 호출에서 새 setpoint snapshot → 새 4.5초 예측

**즉 매 100ms 마다 reset (rolling). 한 호출 안에서는 입력 고정 (ZOH).**

---

## 3. SITL 검증 + 발견된 발산 패턴

### 3.1 시퀀스 (사전 정의된 setpoint 변화)

`setpoint_sequence.yaml`:

| segment | 시간 | V_cmd [m/s] | h_dot_cmd [m/s] | a_lat_cmd [m/s²] |
|---|---|---|---|---|
| 1 | 0~10s | 16 | 0 | 0 (직진 cruise) |
| 2 | 10~15s | 16 | 1 | **4** (우선회+상승) |
| 3 | 15~20s | 20 | 0 | 0 (직진 가속) |
| 4 | 20~25s | 18 | -1 | **-4** (좌선회+하강) |
| 5 | 25~35s | 16 | 0 | 0 (직진 복귀) |

### 3.2 사후분석 — chunk-mode (4.5초 간격 reset)

```python
# chunk_analysis.py
# rolling-mode CSV 에서 4.5초 간격 (225 ticks) 행만 추출
# 각 chunk 의 시작 행 = i × 225 (4.5s × 50Hz)
# 그 행의 p_*_k (45점 예측) vs 행 (start + k×5) 의 측정값 비교
```

### 3.3 결과

| chunk | t_start | 발생한 것 | XY end err | h end err | psi end err |
|---|---|---|---|---|---|
| 1 | 4.5s | seg1 직진 cruise | 3.55m | 0.34m | 0.5° |
| **2** | **9.0s** | **9~13.5s = seg1→seg2 전환 (10s 우선회)** | **10.14m** | **4.86m** | **39.22°** |
| **3** | **13.5s** | **13.5~18s = seg2→seg3 전환 (15s 직진 가속)** | **10.33m** | 2.19m | **29.43°** |
| 4 | 18.0s | seg3→seg4 전환 (20s 좌선회) | 6.29m | 1.55m | 18.20° |
| 6+ | 27s~ | hold_last 직진 cruise | 0.7~2.5m | 0.01~0.1m | <1° |

→ **segment 전환 시점을 포함하는 chunk 가 큰 발산. 직진 cruise 만 있는 chunk 는 baseline 1~2m**.

### 3.4 발산 원인의 정확한 메커니즘 (chunk 2 예시)

**시퀀스 시간 t=9s 시점의 상태**:
- TrajectoryReplayMode 가 PX4 에 인가 중인 setpoint = (V=16, h=0, a=0) 직진
- Logger 의 atomic snapshot = (sp_v=16, sp_h=0, sp_a=0)

**predict 호출 시 입력**:
- PredictInput = (V_cmd=16, h_dot_cmd=0, a_lat_cmd=0)

**ODE 적분 결과 (45 step, dt=0.1)**:
- a_lat_dot = (0 - 0) / 0.5 = 0 → a_lat 영원히 0 유지
- psi_dot = a_lat / V_h = 0 → psi 영원히 0.107 rad 유지
- → **t=9~13.5s 동안 직진 예측**

**실제 PX4 비행**:
- t=9~10s: 시퀀스 segment 1 (직진) → 모델과 일치
- **t=10~13.5s: 시퀀스 segment 2 활성화, sp_a=4 인가됨 → PX4 우선회 시작**
- → 3.5초 동안 우선회 누적 → 측정값 휘어짐
- → 모델은 이 변화를 모름 → 예측 직진 vs 측정 우선회 = 오차 10m

---

## 4. 문제 — 왜 모델이 segment 전환을 모르는가?

### 4.1 단순한 답 (현재 구현)

`predict()` API 가 **입력을 한 개만 받음** (`PredictInput u_zoh`):

```cpp
template <std::size_t N_STEPS>
void predict(const PredictState & x0,
             const PredictInput & u_zoh,    // ← 하나만
             double dt,
             std::array<PredictState, N_STEPS> & out_traj) const;
```

→ 호출자가 "지금 시점 setpoint" 만 넘겨주면 모델은 그게 horizon 끝까지 유지된다고 가정 (ZOH).

### 4.2 그러나... 호출자는 시퀀스 전체를 알고 있음

`SetpointSequencer` 는 yaml 파일을 미리 다 적재 → t=0 시점에 segment 5개 모두 알고 있음:

```cpp
class SetpointSequencer {
    // 미리 적재된 시퀀스 (5 segment, 35초 분량)
    std::vector<Segment> m_segments;

    // 시간 t 에서의 setpoint 조회
    SetpointOutput lookup(double t_relative) const;
};
```

→ **호출자가 "시퀀스 전체 + 현재 시뮬레이션 시간 t" 를 안다면**, 다음 4.5초 horizon 안에서 segment 전환이 어디서 일어나는지 다 안다.

### 4.3 그럼 왜 그 정보를 predict 에 안 넘기나?

**현재 명세 §11 (Anti-requirements)**:
> - 동적 horizon 금지 (`std::vector<PredictState>` 금지 → 템플릿 N 으로 고정)
> - 풍속 모델 추가 금지
> - 신경망 / 학습 기반 예측기 추가 금지
> - **이웃/swarm 시나리오 가정 금지** (본 클래스는 단일 무인기 1회 호출만 책임. 호출 패턴은 호출자 책임)

→ 명세는 "라이브러리는 단순화 유지, 호출자 책임" 원칙. 즉 **시퀀스를 인지하는 책임은 호출자에게**.

### 4.4 명세 §1 의 본래 컨텍스트

> 분산 충돌회피 알고리즘은 무인기의 미래 궤적을 짧은 horizon (1~3초) 예측할 수 있어야 한다.
> 본 클래스는 그 예측기 본체이며, 호출 컨텍스트는 **rt_thread (1ms 주기)** 임을 전제로 한다.

→ **본래 의도**: collision_avoidance 의 rt_loop 안에서 매 1ms 새 입력으로 ZOH 예측. rt_loop 컨텍스트에서는 "다음 1ms 후 입력이 어떻게 바뀔지" 미리 알 수 없음 (충돌회피 알고리즘이 매 순간 새 입력을 산출). 그래서 ZOH 가정이 합리적.

→ **현재 검증 컨텍스트** (trajectory_replay_node) 에서는 시퀀스가 미리 정해져 있음 → ZOH 가정 부적절. 그러나 라이브러리는 양 컨텍스트 모두 지원해야 함.

---

## 5. 개선 옵션

### 옵션 A: 호출자가 segment 별로 stepRK4 직접 호출 (라이브러리 변경 0)

이미 `stepRK4(x, u, dt) const` 가 public 으로 있음 → 호출자가 직접 시간 따라가며 segment 전환 시점에 입력 바꾸면 됨:

```cpp
// 호출자가 시뮬 시간 t 부터 4.5초 horizon 의 궤적 생성
PredictState x = x0;
double t = t_now;
for (std::size_t k = 0; k < 45; ++k) {
    PredictInput u = sequencer->lookup(t);   // ★ 시간 따라가며 segment 인지
    x = predictor->stepRK4(x, u, 0.1);
    out_traj[k] = x;
    t += 0.1;
}
```

→ 라이브러리 변경 0. 호출자가 시퀀스 정보를 모델에 주입.
→ rt_loop 컨텍스트에서는 sequencer 대신 다른 input source 사용 가능.

### 옵션 B: 라이브러리에 새 API `predictWithSchedule()` 추가

```cpp
// 시간별 입력을 조회 가능한 functor 받음
template <std::size_t N_STEPS, typename InputFn>
void predictWithSchedule(const PredictState & x0,
                         InputFn input_at_time,   // (double t) -> PredictInput
                         double t0,
                         double dt,
                         std::array<PredictState, N_STEPS> & out_traj) const
{
    out_traj[0] = x0;
    for (std::size_t k = 0; k + 1 < N_STEPS; ++k) {
        const double t = t0 + k * dt;
        out_traj[k + 1] = stepRK4(out_traj[k], input_at_time(t), dt);
    }
}
```

→ 호출자가 람다 / function pointer 로 입력 schedule 전달. heap-free 보장 (functor 가 capture 하는 데이터에 따라).

### 옵션 C: 옵션 A 의 패턴을 main.cpp 에 적용 (검증 정확도 즉시 개선)

현재 main.cpp 의 wall_timer 람다를 다음으로 변경:

```cpp
auto predict_timer = node->create_wall_timer(100ms,
    [predictor, logger, sequencer, predicted_traj, dt, replay_start_time]() {
        const auto m = logger->getCurrentMeasurements();
        // ... x0 변환 (기존과 동일)

        // ★ 시뮬 시간 t 따라가며 sequencer 조회 ★
        const double t_now = (now() - replay_start_time).seconds();
        PredictState x = x0;
        for (std::size_t k = 0; k < 45; ++k) {
            const auto sp_at_t = sequencer->lookup(t_now + k * 0.1);
            PredictInput u{ sp_at_t.airspeed, sp_at_t.height_rate, sp_at_t.lateral_acceleration };
            x = predictor->stepRK4(x, u, 0.1);
            (*predicted_traj)[k] = x;
        }
        logger->pushPredictedTrajectory(*predicted_traj);
    });
```

→ chunk 2 의 발산이 사라질 것 (모델이 t=10s segment 전환을 정확히 반영).
→ **단점**: 모델 baseline 오차 (~1m) 와 segment 전환 오차 (~10m) 가 분리되어 있던 정량 검증이 합쳐져버림. 둘 다 보려면 옵션 A/C 와 기존 ZOH 둘 다 측정해야.

---

## 6. 다른 Claude 에게 묻는 질문

다음 질문에 대한 답을 받고 싶음:

### Q1. 본질적인 의문
**현재 검증 결과 (chunk 2 의 10m 발산) 가 "모델의 한계" 인지 "호출 패턴 (ZOH 단일 입력) 의 한계" 인지?**

- 내 분석: **호출 패턴의 한계**. 모델 자체는 받은 입력으로 정확히 적분. 호출자가 시퀀스 전체를 안다면 segment 별 stepRK4 로 발산 제거 가능.
- 동의하는가? 다른 시각이 있는가?

### Q2. 라이브러리 설계 결정
**옵션 A (호출자 책임) vs 옵션 B (새 API 추가) 중 어느 것이 더 적절?**

- 옵션 A 장점: 라이브러리 단순 유지. 명세 §11 ("호출 패턴은 호출자 책임") 정신 준수.
- 옵션 B 장점: rt_loop 컨텍스트에서도 functor 로 미래 입력 주입 가능 (충돌회피 알고리즘이 다음 N step 의 입력을 미리 plan 한 경우).
- 어느 것이 collision_avoidance 통합 + 검증 정확도 둘 다 만족?

### Q3. 검증 메트릭의 의미
**현재 chunk-mode 검증 (rolling-mode CSV 에서 4.5초 간격 추출) 이 모델 정확도를 정확히 측정하는가?**

- chunk 2 의 10m 발산 = "ZOH 가정이 segment 전환 동안 깨짐" 의 정량화.
- 만약 옵션 C 적용 후 다시 검증 → 발산이 ~1m baseline 까지 줄어들 것 예상.
- 두 측정값 (ZOH vs schedule-aware) 의 비교가 의미 있는가? 아니면 둘 중 하나만 측정해야 하는가?

### Q4. rt_loop 통합 시 (collision_avoidance::FormationMode)
**rt_loop 컨텍스트에서는 시퀀스 전체를 모르고 매 ms 새 입력 산출. 그러면 ZOH 외에 옵션이 없는가?**

- 충돌회피 알고리즘 (FlockingGuidance) 이 매 cycle 산출하는 입력은 단일 시점 값.
- 그러나 충돌회피 자체가 "다음 N step 의 plan" 을 만드는 경우도 있음 (MPC 같은).
- collision_avoidance 가 "다음 1초의 입력 schedule" 을 산출한다면 옵션 B 의 functor API 가 유용.
- 그렇지 않다면 ZOH 단일 호출이 적절. 어느 쪽인가?

---

## 7. 추가 정보

### 코드 위치
- 라이브러리: `src/trajectory_prediction/include/trajectory_prediction/{StateType,TrajectoryPredict}.hpp`
- 라이브러리 구현: `src/trajectory_prediction/src/TrajectoryPredict.cpp`
- 호출자 (검증): `src/trajectory_prediction/src/main.cpp`, `TrajectoryLogger.cpp`
- 사후분석: `src/trajectory_prediction/scripts/chunk_analysis.py`

### 검증 산출물
- CSV: `/tmp/trajectory_20260511_132823.csv` (24.9 MB, 53초 비행)
- Plot: `/tmp/chunk_error_curves.png`, `/tmp/chunk_trajectories.png`

### 명세
- `~/ros2_ws/md_file/TASK_trajectory_predictor.md` (가이던스 알고리즘 설계자 발행)

### 워크스페이스 컨벤션
- `~/ros2_ws/PROJECT_FILE_DIAGRAM.md`
- `~/ros2_ws/FORMATION_MODE_ARCHITECTURE.md`
- `~/ros2_ws/SUBSCRIBER_DATA_FLOW.md`
