# Design Audit — trajectory_prediction (2026-05-11)

4가지 디자인 의문에 대한 정직한 답변 + 코드 증거 + 영향 정량 + fix 옵션.

작성 맥락: SESSION_TRANSPLANT_2026-05-11 의 phi-PATCH + tc 동기화 + 첫 SITL 비행 검증 직후. chunk_analysis 결과 (XY end 1.7~9.7m, chunk 0 outlier 83m) 분석 중에 외부 review 가 제기한 의문.

관련 파일:
- `~/ros2_ws/src/trajectory_prediction/include/trajectory_prediction/{StateType,TrajectoryPredict,TrajectoryLogger,SetpointSequencer}.hpp`
- `~/ros2_ws/src/trajectory_prediction/src/{TrajectoryPredict,TrajectoryLogger,TrajectoryReplayMode,main}.cpp`
- `~/ros2_ws/md_file/TASK_trajectory_predictor.md` (명세)
- `~/ros2_ws/md_file/SESSION_TRANSPLANT_2026-05-11_trajectory_prediction.md`

---

## 의문 1 — φ 가 상태인가 가상변수인가? x0.phi 초기값 식 의도? 실측 phi 비교 누락 이유?

### 결론 한 줄
**φ 는 상태변수 (디자인 A) 가 명세 의도. 현재 구현은 초기값을 setpoint 등가로 잡아 부정합 (디자인 B 흉내). 실측 phi 미사용은 단순 구현 누락 (이미 알려진 TODO).**

### 코드 증거

**상태변수 의도 (디자인 A)** — `TrajectoryPredict.cpp:181`:
```cpp
d.phi = (u.phi_cmd - x.phi) / m_params.tau_phi;   // ★ roll loop 1차 지연
```
- ODE 에 명시적 미분식
- RK4 4 stages 모두 phi 적분
- `applyStateSafety` 에서 phi clamp ±60°
- `StateType.hpp` 의 PredictState 7번째 멤버 (다른 6개와 동등)
- `static_assert(sizeof(PredictState)==56)` — phi 포함 1 cache line

**초기값은 setpoint 등가 (부정합)** — `main.cpp:196`:
```cpp
x0.phi = std::atan2(a_lat_safe, k_g);   // setpoint a_lat → 등가 roll
```
- 명령 (a_lat) ↔ 자세 (phi) 혼동
- vehicle_attitude 토픽에서 quaternion → roll 추출 가능한데 안 함

**문서 자체 인정** — `SESSION_TRANSPLANT §14 #1`:
> phi 도 측정값 (vehicle_attitude quaternion → roll) 으로 바꾸는 게 정확. 추후 도입 검토.

### 부정합 A 의 영향 정량

tau_phi = 0.4s 의 1차 지연 수렴:

| 시점 | 초기 오차 잔여 |
|---|---|
| t=0.4s (1τ) | 37% |
| t=0.8s (2τ) | 14% |
| t=1.2s (3τ) | 5% |
| t=1.6s (4τ) | 2% |
| t=4.5s (chunk 끝) | < 0.001% |

→ chunk 처음 ~1.5s 만 영향. **chunk_analysis 의 "XY end" 오차 (t=4.5s) 에는 흔적 없음**. 다만 segment 전환과 chunk boundary 가 어긋날 때 초기 1.5s 누적이 XY 에 ~1~3m 부풀림.

현재 시퀀스가 우연히 4.5s boundary 와 segment boundary 가 정렬되어 영향이 작게 나옴. 일반 production 시나리오에서는 더 클 수 있음.

### 부정합 B (실측 phi 비교 누락) 의 영향

- `chunk_analysis` 표 4 채널 (XY/h/V/psi) 만 — phi 채널 없음
- 모델 roll dynamics 추적 정확도 정량 모름
- tau_phi = 0.4s 가정 검증 불가
- a_lat → phi → psi 사슬 중 a_lat → phi 부분 black box

### Fix (30~40줄)

1. `TrajectoryLogger.hpp` 에 `std::atomic<double> m_roll{0.0}` 슬롯
2. attitude callback 에 한 줄 추가:
   ```cpp
   const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
   const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
   m_roll.store(std::atan2(sinr_cosp, cosr_cosp));
   ```
3. `MeasuredSnapshot` 에 roll 필드 추가
4. `main.cpp:196`: `x0.phi = m.roll;`
5. CSV 헤더 `roll` 컬럼 추가, onTick 에서 fprintf 한 줄
6. `chunk_analysis.py` chunk_compare 에 `meas_phi = sub['roll']` + 표/plot 에 phi 채널 추가

---

## 의문 2 — τ_V, τ_hdot, τ_phi 출처? PX4 SITL fitting 인가 placeholder 인가? R² 와 데이터?

### 결론 한 줄
**Fitting 측정 아님. PX4 컨트롤러 명목 파라미터 (`param show`) 를 1차 지연 가정에 직접 대입한 값. R² 측정 안 했고 데이터는 CSV 에 있으니 fitting 가능.**

### 출처 표

| 모델 변수 | 출처 | PX4 파라미터 | 값 |
|---|---|---|---|
| `tau_V` (5.0s) | `pxh> param show FW_T_TAS_TC` | TECS airspeed error TC | 5.0 |
| `tau_hdot` (5.0s) | `pxh> param show FW_T_ALT_TC` | TECS altitude error TC | 5.0 |
| `tau_phi` (0.4s) | `pxh> param show FW_R_TC` | FW rate/attitude roll TC | 0.4 |

확인 (SESSION_TRANSPLANT §9.3):
```
pxh> param show FW_T_TAS_TC   → 5.0000
pxh> param show FW_T_ALT_TC   → 5.0000
pxh> param show FW_R_TC       → 0.4000
```

이전 값 (이전 세션, 추측 기반):
- tau_V: 4.0 (1040 .params 추측), live 5.0 → **-20%**
- tau_hdot: 2.0 (placeholder), live 5.0 → **-60% (가장 큰 불일치)**
- tau_phi: 0.5 (placeholder), live 0.4 → **+25%**

이게 2026-05-11 본 세션의 동기화 (`airframe_spec.yaml` 수정) 의 정확한 의미.

### 이 가정의 한계

PX4 의 실제 transfer function 은 1차 지연이 **아님**:

| 채널 | 우리 모델 | PX4 실제 |
|---|---|---|
| V_cmd → V | `e^(−t/5.0)` (1차) | TECS 추력+피치 결합, 풍속 보상, ramp limiter. **2차 이상 + nonlinear** |
| h_dot_cmd → h_dot | `e^(−t/5.0)` (1차) | TECS 에너지 분배, 두 채널 결합. **단일 1차 부정확** |
| a_lat_cmd → phi → psi | roll loop `e^(−t/0.4)` + 조정선회 | cascade: lateral_accel→roll_sp→att→rate→torque. **다중 1차 곱 + saturation** |

→ `FW_T_*_TC` 파라미터값과 실제 setpoint→measured 의 effective τ 일치 보장 없음.

### R² 측정 안 한 이유

- 본 단계 우선순위가 "라이브러리 재작성 + SITL 검증 + chunk_analysis 베이스라인" 까지
- fitting 은 §13 의 후순위 작업으로 미뤘음

### 측정에 필요한 데이터 (이미 있음)

| 채널 | step input | 위치 | 응답 |
|---|---|---|---|
| V | seg2→3: V 16→20 (t=15s) | `/tmp/trajectory_*.csv` | `true_airspeed` 컬럼 |
| a_lat→psi | seg1→2: a_lat 0→4 (t=10s) | 동일 | `yaw` 컬럼 |
| h_dot | (현재 시퀀스 수평 only — 없음) | §13.3 추가 필요 | `vd` 컬럼 |

scipy.optimize.curve_fit 으로:
```python
def first_order(t, y0, y_ss, tau):
    return y0 + (y_ss - y0) * (1 - np.exp(-t / tau))
popt, pcov = curve_fit(first_order, t, y_measured)
tau_fit = popt[2]
r2 = 1 - ss_res / ss_tot
```

30~50줄 Python. 결과로 effective τ 측정 + R² 도출.

### 영향 / 다음 단계

- 현재 chunk_analysis 의 baseline 오차 (1.7~9.7m) 의 한 원인이 이 가정 한계
- R² < 0.7 면 1차 지연 가정 부적절 → 2차 모델 또는 nonlinear identification
- R² > 0.9 면 effective τ 로 model 갱신 → 정확도 ↑

---

## 의문 3 — Schedule-aware 가 Replay 모드 전용? Production 에서는 ZOH 만? Backup commitment 있으면 schedule-aware 가능할 텐데?

### 결론 한 줄
**Replay 모드 전용 아님. 현재 main.cpp 람다가 sequencer 참조라서 그렇게 *보이는* 것뿐. 호출자가 미래 plan 알면 어디서든 schedule-aware 가능. 라이브러리 자체는 단일 ZOH (`predict<N>`) 와 stepRK4 (호출자 자유) 둘 다 노출.**

### 코드 증거

**라이브러리 인터페이스** — `TrajectoryPredict.hpp`:
```cpp
PredictState stepRK4(const PredictState & x, const PredictInput & u, double dt);
template<size_t N> void predict(const PredictState & x0,
                                const PredictInput & u_zoh,
                                double dt,
                                std::array<PredictState, N> & out);
```
- `predict<N>` — ZOH 패턴 (단일 u 를 N 번 반복)
- `stepRK4` — 한 step 노출. 호출자가 매 step 마다 다른 u 줄 수 있음 (★ schedule-aware 가능성)

**호출자 패턴** — `main.cpp:203~219`:
```cpp
if (std::isfinite(t_now) && sequencer_for_predict) {
    /* schedule-aware: 호출자가 sequencer 알므로 매 stage 미래 입력 lookup */
    auto & traj = *predicted_traj;
    traj[0] = x0;
    PredictState x = x0;
    for (size_t k = 0; k + 1 < 45; ++k) {
        const double t_future = t_now + (k + 1) * dt;
        const auto sp_at_t = sequencer_for_predict->lookup(t_future);
        PredictInput u_at_t{ sp_at_t.V, sp_at_t.h_dot, sp_at_t.a_lat };
        x = predictor->stepRK4(x, u_at_t, dt);
        traj[k + 1] = x;
    }
} else {
    /* ZOH fallback */
    PredictInput u_zoh{ V_safe, h_dot_safe, a_lat_safe };
    predictor->predict<45>(x0, u_zoh, dt, *predicted_traj);
}
```

→ schedule-aware 는 **호출자의 책임**. 라이브러리는 stepRK4 만 빌려줌.

### Production 시나리오 (collision_avoidance::FormationMode)

| 호출자 상태 | 사용 패턴 |
|---|---|
| 호출자가 미래 trajectory plan 보유 (MPC, RRT*, CBF 등) | schedule-aware (stepRK4 직접 호출) |
| 호출자가 plan 없음 / 단일 setpoint 만 알고 있음 | ZOH (`predict<N>`) |
| 호출자가 일부만 plan (예: 다음 1초만) | hybrid — 1초까지 schedule, 그 이후 ZOH |

**Backup commitment** = 사용자 표현. trajectory plan 의 다른 이름. 어디든 적용 가능.

### 라이브러리 명세의 정합성

`TASK_trajectory_predictor.md §11 (Anti-requirements)`:
> 이웃/swarm 시나리오 가정 금지, 단일 호출만 책임

위반 아님. **호출 패턴은 호출자 책임**이라는 원칙이 그대로 유지됨. schedule-aware 도 stepRK4 의 정상 사용 패턴 중 하나.

### collision_avoidance 통합 시 권장

```cpp
// FormationMode::rt_loop 안 (가상 예시)
PredictState x = current_state;
for (size_t k = 0; k < N; ++k) {
    PredictInput u = mpc_plan_.input_at(t + k * dt);   // ← MPC plan
    x = predictor.stepRK4(x, u, dt);
    predicted_[k] = x;
}
```

→ FormationMode 가 자체 plan 갖고 있으면 동일 패턴 적용.

### 정리

| 모드 | 현재 상태 | 가능성 |
|---|---|---|
| Replay (현재 구현) | schedule-aware | 시퀀스 yaml 알고 있음 |
| Preflight (현재 구현) | ZOH | cruise altitude hold, 시퀀스 없음 |
| Production (collision_avoidance, 미통합) | ZOH 또는 schedule-aware | 호출자의 plan 보유 여부에 따라 자유 선택 |

---

## 의문 4 — 7-state 모델 채택 근거? 본 채팅 30턴 디자인 (6-state γ-based) 과 비교?

### ★ 결정 (2026-05-11 사용자 확정)
**7-state 모델 유지 확정**. 6-state γ-based 와의 결정적 비교는 본 작업 범위 외 (사용자 결정).
아래 추정 비교는 참고용으로 보존. 향후 별도 작업으로 6-state γ 디자인의 정확한 ODE 식을 받으면 결정적 평가 가능.

### 주의
"본 채팅 30턴 디자인" 의 정확한 6-state + γ 식이 본 컨텍스트에 없음. 아래는 Beard-McLain 의 표준 6-state γ-based 가이던스 모델로 추정해서 비교한 것.

### 추정한 두 디자인

**디자인 X — 추정 6-state γ-based (Beard-McLain §9 정형 가이던스 모델)**:
```
상태: (p_n, p_e, h, V, ψ, γ)
입력: (V_cmd, γ_cmd, ψ_cmd or a_lat_cmd)

ODE:
  p_n_dot = V cos(γ) cos(ψ)
  p_e_dot = V cos(γ) sin(ψ)
  h_dot   = V sin(γ)              ← algebraic 관계 (별도 lag 없음)
  V_dot   = (V_cmd - V) / τ_V
  ψ_dot   = g tan(φ) / V          ← φ 는 algebraic = atan2(a_lat, g)
  γ_dot   = (γ_cmd - γ) / τ_γ
```

특징: φ 자체는 상태 아니고 a_lat → φ → ψ_dot 으로 algebraic 통과. γ 가 1차 lag 으로 종 채널 흡수.

**디자인 Y — 현재 7-state (Beard-McLain 변형 + 우리 PATCH)**:
```
상태: (p_n, p_e, h, V, ψ, h_dot, φ)
입력: (V_cmd, h_dot_cmd, a_lat_cmd)

ODE:
  p_n_dot = V_h cos(ψ),     V_h = sqrt(V² - h_dot²)
  p_e_dot = V_h sin(ψ)
  h_dot_pos = h_dot         ← h 의 미분 = 상태 h_dot
  V_dot   = (V_cmd - V) / τ_V
  ψ_dot   = g tan(φ) / V_h
  h_ddot  = (h_dot_cmd - h_dot) / τ_hdot   ← h_dot 도 lag 상태
  φ_dot   = (φ_cmd - φ) / τ_phi            ← φ 도 lag 상태
```

특징: h_dot 과 φ 가 각각 1차 lag 상태로 명시. γ 안 씀.

### 양자 비교

| 항목 | 디자인 X (6-state γ) | 디자인 Y (7-state, 현재) |
|---|---|---|
| 상태 수 | 6 | 7 |
| sizeof | 48B | 56B (1 cache line) |
| 상태 변수 | h, V, ψ, γ | h, V, ψ, h_dot, φ |
| 종 채널 lag | γ 가 흡수 (단일) | h_dot 가 흡수 (단일) |
| 횡 채널 lag | φ algebraic (lag 0) | φ 가 흡수 (1차 lag) |
| h_dot 표현 | V sin(γ) | 상태 변수 직접 |
| φ 표현 | a_lat / g (algebraic) | 1차 lag 상태 |
| TECS 매칭 | γ_cmd 가 자연스러움 | h_dot_cmd 가 자연스러움 (PX4 인터페이스 일치) |
| 자세 매칭 | φ algebraic — 자세 lag 무시 | φ 상태 — roll loop 명시적 |

### 7-state 채택 근거 (현재 디자인)

**근거 1 — PX4 인터페이스 1:1 매칭**

명세 §2.4 의 입력 POD `FwSetpointOutput_rt2mt`:
```cpp
struct FwSetpointOutput_rt2mt {
    float airspeed;              // V_cmd
    float height_rate;           // h_dot_cmd  ← γ_cmd 아님
    float lateral_acceleration;  // a_lat_cmd  ← ψ_cmd 아님
    bool  is_fallback;
};
```

PX4 의 `FwLateralLongitudinalSetpoint` 가 `withHeightRate()`, `withLateralAcceleration()` 메서드를 노출. 즉 PX4 의 명령 인터페이스가 (V, h_dot, a_lat) 의 직접 입력 — γ 나 ψ_cmd 가 아님.

→ 6-state γ 모델 사용 시 외부 입력 (V_cmd, h_dot_cmd) 을 (V_cmd, γ_cmd = atan2(h_dot_cmd, V)) 로 매번 변환 필요. **그리고 V 가 상태라 변환식이 *time-varying* 됨**. 추가 복잡도.

7-state 는 변환 1회 (a_lat → phi) 만 stepRK4 진입 시.

**근거 2 — h_dot lag 의 명시화**

PX4 TECS 의 altitude 채널은 명시적 1차 지연 (FW_T_ALT_TC) 가짐. 6-state γ 에서 h_dot = V sin(γ) 라 algebraic 이면 V sin(γ_cmd) 가 즉시 측정 h_dot 이 돼야 함. 실제로는 TECS lag 으로 안 그럼.

→ 7-state 의 `h_ddot = (h_dot_cmd − h_dot) / τ_hdot` 가 PX4 컨트롤러 명목 동작과 더 가까움.

**근거 3 — φ lag 의 명시화 (★ PATCH 의 핵심)**

이전 6-state 모델 (명세 §5.2, placeholder) 은 a_lat 을 직접 ψ_dot 에 넣음 (φ algebraic). small-angle 가정 (psi_dot ≈ a_lat / V_h). 큰 롤각 (40°+) 에서 부정확.

7-state PATCH 는 a_lat → φ → ψ 사슬을 정형. `psi_dot = g tan(φ) / V_h` 가 큰 롤각에서도 정확. 추가로 φ 의 1차 lag 도 흡수 (PX4 의 FW_R_TC = 0.4s).

→ 큰 롤각 시나리오에서 6-state 보다 정확. SESSION_TRANSPLANT §4.7 의 단위테스트 #12 (LargeRollAngleAccuracy) 가 이걸 직접 검증.

**근거 4 — γ_cmd 의 부재 (실용)**

ψ_cmd 같은 명시 입력이 PX4 에는 없음. course 는 lateral_acceleration 으로 간접 제어. γ_cmd 도 마찬가지 — height_rate 로 간접. → γ-based 모델의 자연스러운 입력 채널이 PX4 인터페이스와 매칭 안 됨.

### 6-state γ 가 더 좋을 시나리오

- 외부 명령이 (V, γ, ψ) flight-path-frame: 글라이더, 일반 UAV guidance 교과서, NN policy 가 path angles 출력하는 경우
- 종 채널이 instantaneous (lag 없음, 추력 비례): 임펄스 응답 시뮬레이션
- 코드 단순성 우선 (1 state 적음)
- φ 의 자세 lag 무시해도 되는 cruise/low-bandwidth 시나리오

### 평가 표

| 평가 항목 | 디자인 X (6-state γ) | 디자인 Y (7-state, 현재) |
|---|---|---|
| PX4 인터페이스 매칭 | ✗ 변환 필요 | ✓ 1:1 |
| h_dot lag 표현 | ✗ algebraic (TECS 와 부정합) | ✓ 명시 1차 lag |
| φ lag 표현 | ✗ algebraic | ✓ 명시 1차 lag |
| 큰 롤각 정확도 | ✗ small-angle 가정 | ✓ tan(φ) 정형 |
| 종 채널 결합 (V↔h_dot) | △ γ_dot 으로 일부 | △ 독립 lag (TECS 결합 무시) |
| sizeof / cache | ✓ 48B | ✓ 56B (1 line) |
| RK4 step 비용 | ✓ 적음 | △ 약간 많음 (실측 2.93μs / 5.93μs) |
| 학술 정형성 | ✓ 표준 가이던스 | △ 변형 |

### 정리

**7-state 채택의 핵심 이유**: PX4 인터페이스 정확 매칭 + TECS/roll loop 의 1차 lag 명시 + 큰 롤각 정확도.

**6-state γ 가 옳을 수도 있는 경우**: 외부 명령이 path-angle 기반, 또는 학술적 정형성 우선, 또는 TECS 의 lag 을 무시해도 되는 시나리오.

→ **단순히 "더 좋은" 모델은 없음**. 두 디자인은 다른 가정에 최적화. 현재 7-state 는 PX4 + ROS2 오프보드 + chunk_analysis 검증 파이프라인에 정합.

### 정확한 비교를 위해 필요한 것

본 채팅 30턴 디자인의:
- 정확한 ODE 식 (특히 γ_dot 정의)
- 입력 채널 (V_cmd, γ_cmd, ψ_cmd? 또는 a_lat_cmd?)
- 시정수 정의 (τ_V, τ_γ, τ_ψ?)
- 가정 (small-angle? linearization? 풍속?)

이 정보 주시면 위 비교 표를 업데이트해서 결정적 평가 가능합니다.

---

## 정리 — 4 의문 우선순위

| # | 의문 | 본질 | 영향 | 권장 우선순위 |
|---|---|---|---|---|
| 1 | φ 상태/가상 + 초기값 + phi 비교 | 부정합 인정 (TODO 였음) | 끝점 오차 < 1%, 그러나 디자인 청결성 | **A. 30~40줄 fix** |
| 2 | τ 출처 + fitting R² | 측정 안 함, 명목값 사용 | baseline 오차의 한 원인 | **B. 30~50줄 fitting 스크립트** |
| 3 | schedule-aware production | 라이브러리는 둘 다 노출, 호출자 책임 | 명세 정합. 위반 없음 | 의문만, fix 불필요 |
| 4 | 7-state vs 6-state γ | 디자인 선택 차이, 둘 다 valid | 본질 차이. 정확한 비교는 명세 필요 | **D. 30턴 디자인 명세 필요** |

권장 다음 작업: 1 → 2 → (3 명시 update) → 4 결정적 비교 (사용자 명세 제공 후).
