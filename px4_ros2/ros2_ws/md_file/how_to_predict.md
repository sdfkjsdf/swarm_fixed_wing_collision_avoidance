# 궤적 예측 모델 — 설정 + 적분 방식

> **범위**: `trajectory_prediction` 패키지의 *예측 모델 설정* + *RK4 적분 방식* 만.
> **제외**: 검증 / 결과 / 한계 / 시각화. *모델 자체* 설명에 한정.

---

## 1. 모델 개요

**7-state, 4-input, 연속시간 ODE + RK4 적분**.

수학적 형식:

$$
\dot{\mathbf{x}}(t) = f(\mathbf{x}(t),\; \mathbf{u}(t)), \qquad
\mathbf{x} \in \mathbb{R}^7, \quad \mathbf{u} \in \mathbb{R}^4
$$

- 식 출처: Beard-McLain *Small Unmanned Aircraft* (9.19) 의 fixed-wing 정형 모델
- 좌표계: NED 위치 + altitude (`h = -z_NED`, +up) 혼합
- 적분 horizon: $T_h = 4.5$ s, step $\Delta t = 0.1$ s → **45 step RK4**
- 적분 호출 주기: 100 ms (10 Hz). 매 호출마다 새로운 4.5 초 미래 trajectory 계산.

---

## 2. 상태 변수 (7-state)

### 수학 표기

$$
\mathbf{x} = \begin{bmatrix} p_n \\ p_e \\ h \\ V \\ \psi \\ \dot{h} \\ \phi \end{bmatrix}
= \begin{bmatrix} \text{north position [m]} \\ \text{east position [m]} \\ \text{altitude (up) [m]} \\ \text{airspeed [m/s]} \\ \text{heading [rad]} \\ \text{climb rate [m/s]} \\ \text{roll angle [rad]} \end{bmatrix}
$$

### 코드 정의 (`include/trajectory_prediction/StateType.hpp:66-74`)

```cpp
struct PredictState {
    double p_n;     // [m]    NED north
    double p_e;     // [m]    NED east
    double h;       // [m]    altitude (up positive, = -z_NED)
    double V;       // [m/s]  airspeed magnitude
    double psi;     // [rad]  heading, wrapped to [-pi, pi]
    double h_dot;   // [m/s]  current climb rate (autopilot tracking)
    double phi;     // [rad]  current roll angle (autopilot tracking)
};
```

크기 = **56 byte** = cache-line aligned (`static_assert(sizeof(PredictState) == 56)`).

### 상태 초기화 (예측 시작 시점)

매 predict 호출마다 *측정값 (PX4 EKF)* 으로 초기화. `src/main.cpp:216-223`:

$$
\mathbf{x}_0 = \begin{bmatrix} p_n \\ p_e \\ h \\ V \\ \psi \\ \dot{h} \\ \phi \end{bmatrix}_0
= \begin{bmatrix} m.x \\ m.y \\ -m.z \\ m.V_{true} \\ m.\psi_{meas} \\ -m.v_d \\ m.\phi_{meas} \end{bmatrix}
$$

```cpp
PredictState x0;
x0.p_n   = m.x;                  // PX4 vehicle_local_position.x  (NED north)
x0.p_e   = m.y;                  // PX4 vehicle_local_position.y  (NED east)
x0.h     = -m.z;                 // NED z(down) → altitude(up)
x0.V     = m.true_airspeed;      // PX4 airspeed_validated.true_airspeed_m_s
x0.psi   = m.yaw;                // PX4 vehicle_attitude (quaternion → yaw)
x0.h_dot = -m.vd;                // NED vd(down) → climb-rate(up)
x0.phi   = m.roll;               // ★ PX4 vehicle_attitude (quaternion → roll)
```

★ **`phi` 는 *명령* (`atan2(a_lat_cmd, g)`) 이 아니라 *측정 roll***. Beard-McLain 7-state 명세 정합.

---

## 3. 입력 변수 (4-input)

### 수학 표기

$$
\mathbf{u} = \begin{bmatrix} V_{cmd} \\ h_{cmd} \\ \dot{h}_{cmd} \\ a_{lat,cmd} \end{bmatrix}
= \begin{bmatrix} \text{airspeed setpoint [m/s]} \\ \text{altitude setpoint [m]} \\ \text{climb-rate setpoint [m/s]} \\ \text{lateral acceleration setpoint [m/s}^2\text{]} \end{bmatrix}
$$

### 코드 정의 (`StateType.hpp:80-87`)

```cpp
struct PredictInput {
    double V_cmd;       // [m/s]   airspeed setpoint
    double h_cmd;       // [m]     altitude setpoint (NaN 허용 — NaN guard)
    double h_dot_cmd;   // [m/s]   climb-rate setpoint
    double a_lat_cmd;   // [m/s²]  lateral acceleration setpoint
};
```

★ 외부 입력은 $a_{lat,cmd}$ 로 받고, RK4 내부에서 $\phi_{cmd} = \arctan2(a_{lat,cmd}, g)$ 으로 1회 변환.

---

## 4. 파라미터

### 수학 표기

$$
\theta = \{\tau_V, \tau_{\dot{h}}, \tau_\phi, b_h, V_{\min}, V_{\max}, \dot{h}_{\max}, a_{lat,\max}, V_{h,\min}\}
$$

### 코드 정의 (`StateType.hpp:93-112`)

```cpp
struct PredictParams {
    // 1차 지연 시정수 [s]
    double tau_V       = 4.0;    // FW_T_TAS_TC
    double tau_hdot    = 2.0;    // FW_T_ALT_TC
    double tau_phi     = 0.5;    // FW_R_TC

    // ★ 종 채널 PD 의 altitude P 게인 [1/s]
    // 0.0 = 기존 1차 지연. >0 = Beard-McLain (9.19) PD 형태.
    double b_h         = 0.0;

    // 입력 saturation 한계
    double V_min       = 12.0;   // FW_AIRSPD_MIN
    double V_max       = 25.0;   // FW_AIRSPD_MAX
    double h_dot_max   = 5.0;    // FW_T_CLMB_MAX (대칭)
    double a_lat_max   = 9.8;    // g * tan(FW_R_LIM)

    // 수치 안전 floor (V_h = sqrt(V² - h_dot²) 발산 방지)
    double V_h_min     = 1.0;
};
```

### YAML 로 override (`config/airframe_spec.yaml`)

```yaml
"/**":
  ros__parameters:
    airspeed_min: 10.0
    airspeed_max: 25.0
    height_rate_max_climb: 8.0
    height_rate_min_sink: 2.7
    max_roll_deg: 50.0
    gravity: 9.80665

    tc_tas:  5.0     # → PredictParams::tau_V
    tc_alt:  5.0     # → PredictParams::tau_hdot
    tc_roll: 0.4     # → PredictParams::tau_phi
    b_h:     0.1     # ★ altitude P gain
    alt_offset: 5.0  # ★ h_cmd = baseline_alt + alt_offset (고정 target)
```

`main.cpp` 가 `node->get_parameter()` 로 받아 `PredictParams pp` 채워서 `TrajectoryPredict` 생성자 전달.

---

## 5. ODE (운동방정식)

### 수학 표기 — 핵심 식

상수 $g = 9.80665$ m/s² (`k_g` in code).

**수평속도 유도 변수**:

$$
V_h = \max\left(\sqrt{\max(V^2 - \dot{h}^2,\; 0)},\; V_{h,\min}\right)
$$

**7-state ODE** $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$:

$$
\boxed{
\begin{aligned}
\dot{p}_n   &= V_h \cos(\psi) \\
\dot{p}_e   &= V_h \sin(\psi) \\
\dot{h}     &= \dot{h} \qquad \text{(state propagation: } \tfrac{d h}{dt} = \text{state } \dot{h}\text{)} \\
\dot{V}     &= \dfrac{V_{cmd} - V}{\tau_V} \\
\dot{\psi}  &= \dfrac{g \tan(\phi)}{V_h} \qquad \text{★ 조정선회 (coordinated turn)} \\
\ddot{h}    &= \dfrac{\dot{h}_{cmd} - \dot{h}}{\tau_{\dot{h}}}
              + b_h \cdot (h_{cmd} - h) \qquad \text{★ Beard-McLain (9.19) PD form} \\
\dot{\phi}  &= \dfrac{\phi_{cmd} - \phi}{\tau_\phi}
\end{aligned}
}
$$

### 채널별 의미

| 채널 | 식 형태 | 의미 |
|---|---|---|
| $p_n, p_e$ | 운동학 (kinematic) | 위치 = 속도 적분 |
| $h$ | 상태 전파 | $\dot{h}$ 가 상태 자체 |
| $V$ | 1차 지연 (1st-order lag) | TECS airspeed loop |
| $\psi$ | algebraic (정상선회) | bank-to-turn |
| $\ddot{h}$ | **PD 형태** | rate term + altitude P |
| $\phi$ | 1차 지연 | FW roll loop |

### 코드 직접 인용 (`src/TrajectoryPredict.cpp:163-192` `evaluateODE`)

```cpp
PredictState d;
d.p_n   = V_h * cos_psi;
d.p_e   = V_h * sin_psi;
d.h     = x.h_dot;
d.V     = (u.V_cmd     - x.V    ) / m_params.tau_V;
d.psi   = k_g * std::tan(x.phi) / V_h;                       // 현재 상태 phi 사용
d.h_dot = (u.h_dot_cmd - x.h_dot) / m_params.tau_hdot
        +  m_params.b_h * (u.h_cmd - x.h);                   // ★ PD form
d.phi   = (u.phi_cmd   - x.phi  ) / m_params.tau_phi;
```

### 모델 가정 (전제)

- **No wind** (사이드슬립 $\beta = 0$). PX4 EKF 의 *ground velocity* 와 모델의 *airspeed-based velocity* 일치 가정.
- **Coordinated turn** — $\tan(\phi) = a_{lat}/g$. Transient 중 ($\phi$ 변화 중) 부정확.
- **3-DOF**: pitch/yaw rate 동역학 무시. $\psi$ 는 *algebraic* 으로.
- **완벽 대칭** — rudder/aileron trim offset, 무게중심 변동 무시.

---

## 6. 외부 입력 → 내부 입력 변환 (stepRK4 진입 시 1회)

### 수학 표기

$$
\boxed{\phi_{cmd} = \arctan2(a_{lat,cmd},\; g)}
$$

NaN guard:

$$
h_{cmd,int} = \begin{cases} h_{cmd} & \text{if } \text{isfinite}(h_{cmd}) \\ h & \text{otherwise (fallback: P-term = 0)} \end{cases}
$$

### 코드 (`TrajectoryPredict.cpp:216-220`)

```cpp
InternalInput u_int;
u_int.V_cmd     = u_sat.V_cmd;
u_int.h_cmd     = std::isfinite(u_sat.h_cmd) ? u_sat.h_cmd : x_sat.h;   // NaN guard
u_int.h_dot_cmd = u_sat.h_dot_cmd;
u_int.phi_cmd   = std::atan2(u_sat.a_lat_cmd, k_g);                     // ★ atan2 변환
```

### 두 가지 변환 의미

1. **$\phi_{cmd} = \arctan2(a_{lat,cmd}, g)$** — lateral_acceleration 명령 → roll angle 명령. 입력 saturation 으로 $|a_{lat,cmd}| \leq a_{lat,\max} = g \tan(\text{FW\_R\_LIM})$ 보장 → 자동으로 $|\phi_{cmd}| \leq \text{FW\_R\_LIM}$.
2. **$h_{cmd}$ NaN guard** — `NaN` 이면 $h_{cmd} \leftarrow h$ 로 치환 → ODE 의 $b_h (h_{cmd} - h) = 0$ → 1차 지연 fallback.

---

## 7. Saturation + State safety

### 7.1 입력 saturation (`applyInputSaturation`, `TrajectoryPredict.cpp:124-132`)

$$
\begin{aligned}
V_{cmd}      &\leftarrow \mathrm{clamp}(V_{cmd},      V_{\min},       V_{\max}) \\
\dot{h}_{cmd} &\leftarrow \mathrm{clamp}(\dot{h}_{cmd}, -\dot{h}_{\max}, +\dot{h}_{\max}) \\
a_{lat,cmd}  &\leftarrow \mathrm{clamp}(a_{lat,cmd},  -a_{lat,\max},   +a_{lat,\max}) \\
h_{cmd}      &\leftarrow h_{cmd} \qquad \text{(clamp 없음, 절대 고도)}
\end{aligned}
$$

분기 없이 `fmin/fmax` 만 사용 (real-time 안전):

$$\mathrm{clamp}(v, lo, hi) = \min(\max(v, lo),\; hi)$$

### 7.2 상태 안전성 (`applyStateSafety`, `TrajectoryPredict.cpp:142-156`)

$$
\begin{aligned}
V        &\leftarrow \max(V,\; V_{h,\min}) & \text{(V 음수/0 방지)}\\
\dot{h}  &\leftarrow \mathrm{clamp}(\dot{h},\; -0.99 V,\; +0.99 V) & (V_h \text{ 발산 방지})\\
\phi     &\leftarrow \mathrm{clamp}(\phi,\; -60°,\; +60°) & (\tan(\pm\tfrac{\pi}{2}) \text{ 발산 방지})\\
\psi     &\leftarrow \mathrm{wrap}_\pi(\psi) & (\psi \in [-\pi, \pi])
\end{aligned}
$$

$\mathrm{wrap}_\pi$ 식 (분기 없는 `fmod` 2회):

$$\mathrm{wrap}_\pi(\psi) = \big((\psi + \pi) \bmod 2\pi + 2\pi\big) \bmod 2\pi - \pi$$

---

## 8. RK4 적분 (1 step)

### 수학 표기 — classical 4th-order Runge-Kutta + Zero-Order Hold

상태 $\mathbf{x}_n$ 에서 $\Delta t$ 후 $\mathbf{x}_{n+1}$ 적분:

$$
\boxed{
\begin{aligned}
\mathbf{k}_1 &= f(\mathbf{x}_n,\;\; \mathbf{u}_{int}) \\
\mathbf{k}_2 &= f\!\left(\mathbf{x}_n + \tfrac{\Delta t}{2} \mathbf{k}_1,\;\; \mathbf{u}_{int}\right) \\
\mathbf{k}_3 &= f\!\left(\mathbf{x}_n + \tfrac{\Delta t}{2} \mathbf{k}_2,\;\; \mathbf{u}_{int}\right) \\
\mathbf{k}_4 &= f\!\left(\mathbf{x}_n + \Delta t\, \mathbf{k}_3,\;\; \mathbf{u}_{int}\right) \\
\mathbf{x}_{n+1} &= \mathbf{x}_n + \dfrac{\Delta t}{6}\big(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4\big)
\end{aligned}
}
$$

★ **Zero-Order Hold (ZOH)**: 4 stage 모두 *같은 $\mathbf{u}_{int}$* 평가 (입력이 step 사이 일정 가정).

### 전체 1 step 흐름

$$
\begin{array}{l}
\text{Input: } \mathbf{x}_n, \mathbf{u}_n, \Delta t \\
\hline
1.\; \mathbf{u}_{sat} \;\;\leftarrow\; \mathrm{applyInputSaturation}(\mathbf{u}_n) \\
2.\; \mathbf{x}_{sat} \;\;\leftarrow\; \mathrm{applyStateSafety}(\mathbf{x}_n) \\
3.\; \mathbf{u}_{int} \;\;\leftarrow\; \{V_{cmd},\, h_{cmd}^{\text{(NaN guard)}},\, \dot{h}_{cmd},\, \arctan2(a_{lat}, g)\} \\
4.\; \mathbf{k}_1 \leftarrow f(\mathbf{x}_{sat},\; \mathbf{u}_{int}) \\
5.\; \mathbf{k}_2 \leftarrow f(\mathbf{x}_{sat} + \tfrac{\Delta t}{2}\mathbf{k}_1,\; \mathbf{u}_{int}) \\
6.\; \mathbf{k}_3 \leftarrow f(\mathbf{x}_{sat} + \tfrac{\Delta t}{2}\mathbf{k}_2,\; \mathbf{u}_{int}) \\
7.\; \mathbf{k}_4 \leftarrow f(\mathbf{x}_{sat} + \Delta t\,\mathbf{k}_3,\; \mathbf{u}_{int}) \\
8.\; \mathbf{x}_{n+1}^{\text{raw}} \leftarrow \mathbf{x}_{sat} + \tfrac{\Delta t}{6}(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4) \\
9.\; \mathbf{x}_{n+1} \leftarrow \mathrm{applyStateSafety}(\mathbf{x}_{n+1}^{\text{raw}}) \quad \text{(후처리)} \\
\hline
\text{Output: } \mathbf{x}_{n+1}
\end{array}
$$

### 코드 직접 인용 (`TrajectoryPredict.cpp:200-233`)

```cpp
const PredictInput  u_sat = applyInputSaturation(u);
const PredictState  x_sat = applyStateSafety(x);

InternalInput u_int;
u_int.V_cmd     = u_sat.V_cmd;
u_int.h_cmd     = std::isfinite(u_sat.h_cmd) ? u_sat.h_cmd : x_sat.h;
u_int.h_dot_cmd = u_sat.h_dot_cmd;
u_int.phi_cmd   = std::atan2(u_sat.a_lat_cmd, k_g);

const PredictState k1 = evaluateODE(x_sat,                                u_int);
const PredictState k2 = evaluateODE(scaleAndAdd(x_sat, 0.5 * dt, k1),     u_int);
const PredictState k3 = evaluateODE(scaleAndAdd(x_sat, 0.5 * dt, k2),     u_int);
const PredictState k4 = evaluateODE(scaleAndAdd(x_sat,        dt, k3),    u_int);

const PredictState x_next_raw = rk4Combine(x_sat, dt / 6.0, k1, k2, k3, k4);
return applyStateSafety(x_next_raw);
```

---

## 9. 호출 흐름 (45-step trajectory 생성)

### 수학 표기

매 100 ms 마다, 측정 $m(t_{now})$ 으로 초기화 + sequencer 의 *미래 입력* 으로 적분:

$$
\begin{aligned}
\mathbf{x}_0 &= \text{init}(m(t_{now})) \\
\mathbf{x}_{k+1} &= \mathrm{stepRK4}\big(\mathbf{x}_k,\; \mathbf{u}(t_{now} + (k+1)\Delta t),\; \Delta t\big), \\
&\qquad k = 0, 1, \ldots, 43
\end{aligned}
$$

여기서 $\Delta t = 0.1$ s, **trajectory** = $\{\mathbf{x}_0, \mathbf{x}_1, \ldots, \mathbf{x}_{44}\}$ 45 점.

**미래 입력** $\mathbf{u}(t)$ 의 각 채널:

$$
\mathbf{u}(t) = \begin{bmatrix}
V_{cmd}^{seq}(t) \\
h_{cmd}^{future} = \text{baseline\_alt} + \text{alt\_offset} \quad \text{(★ 고정)} \\
\dot{h}_{cmd}^{seq}(t) \\
a_{lat,cmd}^{seq}(t)
\end{bmatrix}
$$

$V_{cmd}^{seq}(t), \dot{h}_{cmd}^{seq}(t), a_{lat,cmd}^{seq}(t)$ 는 `SetpointSequencer::lookup(t)` 가 반환. $h_{cmd}$ 만 **시간에 무관한 고정 상수** (Formation 식).

### 코드 (`src/main.cpp:196-260` predict wall_timer 람다)

```cpp
auto predict_timer = node->create_wall_timer(predict_period,
    [predictor, logger, sequencer_for_predict, predicted_traj, dt, k_g, alt_offset]() {
        const auto m  = logger->getCurrentMeasurements();
        const auto sp = logger->getCurrentSetpointSnapshot();

        // 6-state 측정 → 7-state PredictState 변환
        PredictState x0;
        x0.p_n   = m.x;
        x0.p_e   = m.y;
        x0.h     = -m.z;
        x0.V     = m.true_airspeed;
        x0.psi   = m.yaw;
        x0.h_dot = -m.vd;
        x0.phi   = m.roll;                  // ★ 실측 roll

        const double t_now        = logger->getReplayTime();
        const double baseline_alt = logger->getBaselineAlt();

        if (std::isfinite(t_now) && sequencer_for_predict) {
            // schedule-aware: 매 step 마다 sequencer 의 미래 입력 lookup
            auto & traj = *predicted_traj;
            traj[0] = x0;
            PredictState x = x0;
            for (std::size_t k = 0; k + 1 < TrajectoryLogger::kPredictHorizon; ++k) {
                const double t_future = t_now + (k + 1) * dt;
                const auto sp_at_t    = sequencer_for_predict->lookup(t_future);
                const double h_cmd_future = baseline_alt + alt_offset;   // ★ 고정
                PredictInput u_at_t{sp_at_t.V, h_cmd_future, sp_at_t.h_dot, sp_at_t.a_lat};
                x = predictor->stepRK4(x, u_at_t, dt);
                traj[k + 1] = x;
            }
        }
        logger->pushPredictedTrajectory(*predicted_traj);
    });
```

### 핵심 특징

- **45 점 trajectory** ($k_{\text{PredictHorizon}} = 45$, $\Delta t = 0.1$s → $t \in [0, 4.5]$ s).
- **Schedule-aware**: 매 step 마다 `sequencer->lookup(t_future)` 로 *해당 시각의 setpoint* 받음. 시퀀스 안 segment 전환이 예측에 반영.
- **$h_{cmd}$ = baseline_alt + alt_offset 고정** (★ Formation 식): 시퀀스 적분 안 함. 시퀀스의 $\dot{h}_{cmd}$ 만 rate term 으로 사용.

### Fallback 경로 (Replay 비활성 시)

$$\mathbf{u}_{ZOH} = \{V_{safe},\; \text{NaN},\; \dot{h}_{safe},\; a_{lat,safe}\}, \quad \forall t$$

```cpp
PredictInput u_zoh{V_safe, std::nan(""), h_dot_safe, a_lat_safe};
predictor->predict<45>(x0, u_zoh, dt, *predicted_traj);
```

$h_{cmd} = $ NaN → stepRK4 NaN guard 로 $h_{cmd} \leftarrow x.h$ 치환 → P-term 0 → 1차 지연 fallback.

---

## 10. 데이터 흐름

```
┌──────────────────┐   매 100ms   ┌────────────────────────────────────┐
│ PX4 EKF (측정)   │ ──────────→ │ predict_timer (main.cpp:196)       │
│ - vehicle_local  │              │                                    │
│ - airspeed       │              │  1. x0 ← 측정값                   │
│ - attitude       │              │  2. for k = 0..43:                │
│                  │              │       sp = sequencer.lookup(t+kΔt) │
│                  │              │       x  = stepRK4(x, sp, 0.1)    │
│                  │              │       traj[k+1] = x               │
│                  │              │  3. logger.push(traj)             │
│                  │              └────────────────────────────────────┘
│                  │                              │
│                  │                              ▼
│                  │              ┌────────────────────────────────────┐
│                  │              │ TrajectoryLogger CSV (50Hz)        │
│                  │              │ - 14 측정 컬럼                     │
│                  │              │ - 7 × 45 = 315 prediction 컬럼     │
└──────────────────┘              └────────────────────────────────────┘
```

---

## 11. 핵심 코드 라인 색인

| 자리 | 파일 | 라인 |
|---|---|---|
| `PredictState` 구조체 | `include/trajectory_prediction/StateType.hpp` | 66–74 |
| `PredictInput` 구조체 | 같은 파일 | 80–87 |
| `PredictParams` 구조체 | 같은 파일 | 93–112 |
| ODE (`evaluateODE`) | `src/TrajectoryPredict.cpp` | 163–192 |
| 입력 saturation | 같은 파일 | 124–132 |
| 상태 안전성 | 같은 파일 | 142–156 |
| RK4 적분 (`stepRK4`) | 같은 파일 | 200–233 |
| predict 람다 (호출 흐름) | `src/main.cpp` | 196–260 |
| 상태 초기화 (`x0`) | 같은 파일 | 216–223 |
| `h_cmd_future` 계산 | 같은 파일 | 247 |
| YAML 파라미터 | `config/airframe_spec.yaml` | 전체 |

---

## 12. 요약 — 한 페이지 view

**상태 $\mathbf{x} \in \mathbb{R}^7$**:
$$\mathbf{x} = (p_n, p_e, h, V, \psi, \dot{h}, \phi)^T$$

**입력 $\mathbf{u} \in \mathbb{R}^4$**:
$$\mathbf{u} = (V_{cmd}, h_{cmd}, \dot{h}_{cmd}, a_{lat,cmd})^T$$

**ODE**:
$$
\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}) = \begin{bmatrix}
V_h \cos\psi \\ V_h \sin\psi \\ \dot{h} \\
(V_{cmd} - V)/\tau_V \\
g \tan(\phi) / V_h \\
(\dot{h}_{cmd} - \dot{h})/\tau_{\dot{h}} + b_h (h_{cmd} - h) \\
(\phi_{cmd} - \phi)/\tau_\phi
\end{bmatrix}
$$

여기서 $V_h = \max(\sqrt{\max(V^2 - \dot{h}^2, 0)}, V_{h,\min})$, $\phi_{cmd} = \arctan2(a_{lat,cmd}, g)$.

**적분**: classical RK4 with ZOH input, $\Delta t = 0.1$ s, 45 step → 4.5 s horizon.

**예측 호출**: 매 100 ms (10 Hz). 매 호출마다 $\mathbf{x}_0$ ← PX4 EKF 측정값, $\mathbf{u}(t)$ ← `SetpointSequencer.lookup(t)` (단 $h_{cmd}$ = 고정 baseline).

---

**문서 끝.**
