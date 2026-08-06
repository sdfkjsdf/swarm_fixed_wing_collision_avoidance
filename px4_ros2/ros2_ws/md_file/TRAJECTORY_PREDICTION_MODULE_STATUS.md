# 궤적 예측 모듈 현재 구성 및 구현 현황

## 문서 기준

- 확인 날짜: 2026-08-06
- 기준 브랜치: `main`
- 기준 커밋: `6e4a3cc` (`refactor: centralize shared runtime types`)
- 대상 범위:
  - production 패키지의 궤적 예측·재구성 라이브러리
  - `trajectory_prediction_hils`의 호출·로깅·SITL/HILS 검증 경로
- 이 문서는 현재 소스 코드 기준의 상태 추적 문서다. 과거 실험·설계 문서와
  내용이 충돌하면 이 문서와 실제 코드를 우선한다.

## 1. 현재 상태 요약

| 항목 | 현재 상태 |
| --- | --- |
| 7-state 궤적 예측 모델 | 구현 완료 |
| RK4 단일 step 및 고정 horizon 예측 | 구현 완료 |
| 입력·상태 saturation과 수치 안전 처리 | 구현 완료 |
| 4개 핵심 시점 sample 추출 | 구현 완료 |
| 핵심 sample 기반 cubic spline 재구성 | 구현 완료 |
| HILS replay 노드 연결 | 구현 완료 |
| 예측·실측·spline CSV 기록 | 구현 완료 |
| key sample ROS 2 송신 | `Float32MultiArray` prototype 구현 |
| production `vtol_guidance_node` 연결 | 미구현 |
| 다기체 예측 sample 수신·동기화 | 미구현 |
| 미래 궤적 기반 충돌 판정 | 미구현 |
| 회피 기동 생성 및 formation 복귀 | 미구현 |

현재 단계는 **예측 알고리즘과 HILS 검증 파이프라인까지 구현된 상태**다.
production 비행 노드는 아직 예측기를 호출하지 않으며, 충돌 판정과 회피 의사결정
계층도 아직 없다.

## 2. 모듈 경계와 의존 방향

```text
collision_avoidance (production source of truth)
│
├── trajectory_prediction_core
│   ├── PredictTypes.hpp
│   ├── TrajectoryPredict.hpp
│   └── TrajectoryPredict.cpp
│
├── trajectory_reconstruction
│   ├── ReconstructTrajectory.hpp
│   └── ReconstructTrajectory.cpp
│
└── vtol_guidance_node
    └── 현재 trajectory_prediction_core를 링크하지 않음

trajectory_prediction_hils
├── trajectory_prediction_core 링크
├── trajectory_reconstruction 링크
└── trajectory_replay_node에서 예측·추출·재구성·기록 수행
```

의존 방향은 `testing_module → collision_avoidance` 한 방향이다. production
라이브러리는 HILS 코드, ROS 2 메시지, PX4 메시지에 의존하지 않는다.

## 3. 파일 구성

### 3.1 Production 알고리즘

| 파일 | 역할 |
| --- | --- |
| `collision_avoidance/include/collision_avoidance/estimation/trajectory_prediction/PredictTypes.hpp` | 상태·입력·파라미터·key sample POD 정의 |
| `collision_avoidance/include/collision_avoidance/estimation/trajectory_prediction/TrajectoryPredict.hpp` | 공개 API, template horizon 예측, key sample 추출 |
| `collision_avoidance/src/estimation/trajectory_prediction/TrajectoryPredict.cpp` | saturation, ODE, RK4, NED sample 변환 구현 |
| `collision_avoidance/test/test_trajectory_predict.cpp` | 정확도·상태 독립성·RT 특성 단위 테스트 |
| `collision_avoidance/include/collision_avoidance/estimation/reconstruction/ReconstructTrajectory.hpp` | spline 자료형과 재구성 API |
| `collision_avoidance/src/estimation/reconstruction/ReconstructTrajectory.cpp` | clamped cubic spline 계수 계산과 평가 |

### 3.2 HILS 검증

| 파일 | 역할 |
| --- | --- |
| `testing_module/trajectory_prediction_hils/src/nodes/trajectory_replay_main.cpp` | PX4 측정값→초기 상태 변환, 주기 예측, key sample publish |
| `testing_module/trajectory_prediction_hils/include/trajectory_prediction_hils/logging/TrajectoryLogger.hpp` | 측정·setpoint·예측 bundle 자료형과 로거 API |
| `testing_module/trajectory_prediction_hils/src/logging/TrajectoryLogger.cpp` | CSV 기록 및 46점 spline 재구성 |
| `testing_module/trajectory_prediction_hils/include/trajectory_prediction_hils/replay/SetpointSequencer.hpp` | 시간별 시험 setpoint 조회 API |
| `testing_module/trajectory_prediction_hils/src/replay/SetpointSequencer.cpp` | YAML segment 로딩과 lookup 구현 |
| `testing_module/trajectory_prediction_hils/config/replay_params.yaml` | horizon, 예측률, 호출률, 로그 주기 |
| `testing_module/trajectory_prediction_hils/config/airframe_spec.yaml` | 기체 제한, 시정수, 고도 모델 파라미터 |
| `testing_module/trajectory_prediction_hils/config/cases/*.yaml` | 채널별 시험 입력 case |

## 4. 빌드 target

### `collision_avoidance::trajectory_prediction_core`

- 실제 target: `trajectory_prediction_core`
- 형식: C++17 static library
- 구현 파일: `TrajectoryPredict.cpp`
- 공개 include: `collision_avoidance/estimation/trajectory_prediction/*`
- Eigen, ROS 2, PX4 메시지에 의존하지 않는 순수 C++ 알고리즘이다.

### `collision_avoidance::trajectory_reconstruction`

- 실제 target: `trajectory_reconstruction`
- 형식: C++17 static library
- `trajectory_prediction_core`와 `Eigen3::Eigen`에 의존한다.
- key sample을 받아 연속 위치·속도 궤적으로 복원한다.

### 현재 소비자

`trajectory_prediction_hils/trajectory_replay_node`는 두 라이브러리를 모두 링크한다.
반면 production `vtol_guidance_node`는 현재 `FlockingGuidance`만 포함하며 예측·재구성
target을 링크하지 않는다.

## 5. 자료형

모든 예측 자료형은 `collision_avoidance::estimation` 네임스페이스에 있다.

### 5.1 `PredictState`

| 필드 | 단위 | 의미 |
| --- | --- | --- |
| `p_n` | m | NED north 위치 |
| `p_e` | m | NED east 위치 |
| `h` | m | 위쪽이 양수인 고도 |
| `V` | m/s | 모델 속력 크기 |
| `psi` | rad | 수평 진행 방향, `[-π, π]` wrap |
| `h_dot` | m/s | 위쪽이 양수인 상승률 |
| `phi` | rad | roll 상태 |

- 모든 필드는 `double`이다.
- 크기는 56 byte로 고정되며 `static_assert`로 검사한다.
- 헤더 주석은 `V`를 airspeed로 설명하지만, 현재 HILS 초기화는 풍 모델을 명시적으로
  사용하지 않기 위해 `sqrt(vn² + ve² + vd²)`의 ground-speed 크기를 넣는다.

### 5.2 `PredictInput`

| 필드 | 단위 | 의미 |
| --- | --- | --- |
| `V_cmd` | m/s | 속력 명령 |
| `h_cmd` | m | 고도 명령 |
| `h_dot_cmd` | m/s | 상승률 명령 |
| `a_lat_cmd` | m/s² | 횡가속도 명령 |

`h_cmd`가 NaN이면 `stepRK4()`가 현재 `x.h`로 대체한다. 따라서 altitude P 항이
0이 되어 고도 채널은 상승률 1차 지연 모델로 fallback한다.

### 5.3 `PredictParams`

| 필드 | 기본값 | 역할 |
| --- | ---: | --- |
| `tau_V` | 4.0 s | 속력 응답 시정수 |
| `tau_hdot` | 2.0 s | 상승률 응답 시정수 |
| `tau_phi` | 0.5 s | roll 응답 시정수 |
| `b_h` | 0.0 1/s² | 고도 오차 항 계수 |
| `V_min` | 12.0 m/s | 속력 명령 하한 |
| `V_max` | 25.0 m/s | 속력 명령 상한 |
| `h_dot_max` | 5.0 m/s | 상승·하강률 대칭 한계 |
| `a_lat_max` | 9.8 m/s² | 횡가속도 명령 한계 |
| `V_h_min` | 1.0 m/s | 수평 속도 수치 안전 floor |

HILS에서는 기본값을 그대로 쓰지 않고 `airframe_spec.yaml`의 값을 조합한다.

- `tau_V ← tc_tas` (`5.0`)
- `tau_hdot ← tc_alt` (`5.0`)
- `tau_phi ← tc_roll` (`0.5`)
- `b_h ← b_h` (`0.02`)
- `V_min/max ← airspeed_min/max` (`10.0/25.0`)
- `h_dot_max ← min(height_rate_max_climb, abs(height_rate_min_sink))` (`2.7`)
- `a_lat_max ← gravity × tan(max_roll_deg)`

## 6. 예측 수학 모델

수평 속력은 다음과 같이 계산한다.

```text
V_h = max(sqrt(max(V² - h_dot², 0)), V_h_min)
```

현재 ODE는 다음 7개 상태 미분을 사용한다.

```text
p_n_dot = V_h cos(psi)
p_e_dot = V_h sin(psi)
h_dot_position = h_dot

V_dot = (V_cmd - V) / tau_V
psi_dot = g tan(phi) / V_h

h_ddot = (h_dot_cmd - h_dot) / tau_hdot
        + b_h (h_cmd - h)

phi_dot = (phi_cmd - phi) / tau_phi
phi_cmd = atan2(a_lat_cmd, g)
```

특징은 다음과 같다.

- 외부 인터페이스는 횡가속도 명령을 받지만 내부 모델은 roll 명령으로 변환한다.
- roll 상태를 1차 지연시킨 뒤 조정선회 식으로 `psi`를 적분한다.
- 고도 채널은 상승률 1차 지연과 절대 고도 오차 항을 함께 사용한다.
- 풍, sideslip, 6-DOF 자세·각속도, 추진계 동역학은 명시적으로 모델링하지 않는다.

## 7. 적분 API와 호출 방식

### `stepRK4(x, u, dt)`

- 입력과 상태에 안전 처리를 적용한다.
- `a_lat_cmd`를 `phi_cmd`로 한 번 변환한다.
- 동일 입력을 사용하는 RK4 `k1~k4`를 계산한다.
- 적분 결과에 상태 안전 처리를 다시 적용한다.

### `predict<N_STEPS>(x0, u_zoh, dt, out_traj)`

- `out_traj[0] = x0`
- 나머지 점은 동일한 `u_zoh`로 반복 적분한다.
- 결과 배열은 호출자가 소유한다.
- 내부 heap 할당이 없다.

### HILS의 schedule-aware 경로

HILS replay 중에는 미래 YAML segment를 알고 있으므로 단일 ZOH 입력만 사용하지 않는다.

```text
현재 측정 x0
  → 미래 시각 t + 0.1, t + 0.2, ... 에서 sequencer lookup
  → 각 step마다 서로 다른 PredictInput 생성
  → stepRK4를 45회 호출
  → 46점 trajectory 생성
```

Replay가 비활성 상태이거나 replay 시간이 유효하지 않으면 현재 setpoint를 사용하는
`predict<46>()` ZOH 경로로 fallback한다.

## 8. 입력 측정값 변환

HILS 노드는 PX4 측정 snapshot을 다음과 같이 초기 상태로 변환한다.

```text
p_n   = local_position.x
p_e   = local_position.y
h     = -local_position.z
V     = sqrt(vn² + ve² + vd²)
psi   = atan2(ve, vn)
h_dot = -vd
phi   = measured roll
```

중요한 현재 규칙:

- `psi`는 body yaw가 아니라 ground velocity의 course angle이다.
- `V`, `psi`, `h_dot`은 같은 velocity snapshot에서 유도한다.
- `phi`는 횡가속도 명령으로 역산하지 않고 vehicle attitude의 실측 roll을 사용한다.
- setpoint가 fallback 전환 중 NaN이면 HILS 호출부가 속력·상승률·횡가속도에 안전값을
  넣어 예측 전체가 NaN으로 오염되는 것을 막는다.

## 9. Horizon과 실행 주기

기본 HILS 설정은 다음과 같다.

| 항목 | 값 |
| --- | ---: |
| 마지막 예측 시각 | 4.5 s |
| 예측 sample rate | 10 Hz |
| 적분 간격 | 0.1 s |
| trajectory 점 개수 | 46 (`0.0~4.5 s`, 양 끝점 포함) |
| 예측 호출률 | 10 Hz |
| CSV 기록률 | 50 Hz |

노드 시작 시 다음 관계가 맞지 않으면 예외를 발생시킨다.

```text
kPredictHorizon = round(endpoint_s × predict_rate_hz) + 1
```

## 10. Key sample 추출과 통신 prototype

`extractKeySamples()`는 46점 중 다음 정보만 추출한다.

| 시각 | 위치 | 속도 |
| --- | --- | --- |
| 0.0 s | 포함 | 포함 |
| 1.5 s | 포함 | 제외 |
| 3.0 s | 포함 | 제외 |
| 4.5 s | 포함 | 포함 |

좌표는 NED이며 `TrajectorySample`은 총 18개의 `float` 값으로 표현된다.

현재 HILS publish 토픽:

```text
<topic_namespace_prefix>/collision_estimation/key_samples
```

- 메시지: `std_msgs/msg/Float32MultiArray`
- 크기: 18 float, 72 byte payload
- QoS: `SensorDataQoS`, best effort
- 기본 publish rate: 10 Hz

이 메시지는 다기체 통신의 prototype이다. 필드 이름, timestamp, vehicle ID, frame ID,
유효성 정보를 가진 전용 custom message는 아직 없다.

## 11. Cubic spline 재구성

`ReconstructTrajectory`는 key sample의 네 위치와 양 끝 속도를 이용한다.

```text
P1 = position(0.0 s)
P2 = position(1.5 s)
P3 = position(3.0 s)
P4 = position(4.5 s)
V1 = velocity(0.0 s)
V4 = velocity(4.5 s)
```

세 구간의 clamped cubic spline을 계산한다.

```text
[0.0, 1.5], [1.5, 3.0], [3.0, 4.5]

position(tau) = P + tau (b + tau (c + tau d))
velocity(tau) = b + tau (2c + tau 3d)
```

HILS logger는 예측 bundle을 받을 때 spline 계수를 계산하고 `0.0~4.5 s`를 0.1초
간격으로 다시 평가해 별도 CSV에 기록한다.

현재 재구성 구현의 주의점:

- `calculate_clamp_cubic_spline()` 호출 후 `reconstruct()`를 호출해야 한다.
- 범위 밖 시각은 position과 velocity에 NaN을 반환한다.
- spline sample과 segment 저장소가 헤더의 `detail` namespace에 mutable inline
  상태로 존재한다. 현재 HILS는 main executor 단일 thread 호출을 전제로 하지만,
  다중 인스턴스·동시 호출 전에 instance-local 상태로 변경할 필요가 있다.

## 12. 로깅과 출력

`TrajectoryLogger`가 기록하는 핵심 데이터는 다음과 같다.

- PX4 실측 위치·속도·course·roll·true airspeed
- 실제 적용 setpoint
- 예측 시점에 고정한 측정 snapshot
- 46점 × 7-state 예측 trajectory
- 46점 spline 위치·속도 trajectory

예측 trajectory, 예측 초기값을 만든 측정 snapshot, spline 결과는 하나의
`PredictBundle`로 묶어 mutex 안에서 함께 갱신한다. Logger timer가 서로 다른 시점의
데이터를 섞어 기록하지 않도록 하기 위한 구조다.

## 13. 자동 검증 범위

`test_trajectory_predict.cpp`에는 현재 12개 GTest가 있다.

### 정확도·수치 동작

- steady-state convergence
- 시정수에서의 1차 지연 응답
- coordinated turn radius
- straight-line cruise
- airspeed 입력 saturation
- 서로 다른 `dt`에서의 수치 안정성
- 동일 입력 반복 시 stateless 결과

### Roll 기반 모델

- 횡가속도와 roll 명령의 정상상태 등가성
- 큰 roll 각도에서의 정확도

### RT 특성

- 여러 horizon의 WCET 통계 출력
- 예측 중 heap allocation 0회 확인
- 반복 실행 jitter 확인

최근 전체 패키지 검증 결과는 `17 tests, 0 errors, 0 failures`였다. 여기에는
공통 자료형 테스트가 포함된다. 별도의 reconstruction 전용 GTest와
production/HILS end-to-end 자동 테스트는 아직 없다.

## 14. 현재 production 미통합 영역

현재 production `vtol_guidance_node` 흐름은 다음과 같다.

```text
PX4 odometry
  → 공통 좌표 변환
  → FormationMode
  → FlockingGuidance
  → fixed-wing setpoint
```

향후 목표 흐름은 다음과 같이 예상된다.

```text
ego/neighbor 상태
  → 각 기체 trajectory prediction
  → key sample 송수신
  → trajectory reconstruction
  → 충돌 판정
  → 회피 기동 결정
  → guidance/setpoint 반영
  → 충돌 해소 후 formation 복귀
```

아직 없는 주요 구성요소:

1. production에서 예측기를 호출하는 주기와 소유 객체
2. 예측 입력을 production `FwSetpoint`에서 만드는 adapter
3. 기체별 key sample custom message
4. timestamp와 frame을 포함한 다기체 sample 동기화
5. ego-neighbor 최소거리, CPA/TCPA, 안전 반경 판정
6. 충돌 위험 우선순위와 회피 상태 머신
7. 회피 setpoint 생성 및 `FormationMode` 복귀 조건
8. stale·NaN·통신 두절 시 fallback 정책

## 15. 알려진 한계와 확인 필요 사항

- 모델은 풍을 명시적으로 포함하지 않는다.
- HILS의 `V`는 현재 ground-speed 크기이므로 헤더의 airspeed 설명과 용어 정리가 필요하다.
- `h_dot_max`는 상승·하강 비대칭 한계 중 작은 값을 대칭 한계로 사용한다.
- `alt_envelope` 파라미터는 설정에 존재하지만 런타임 범위 검사가 없다.
- `setParams()`는 실행 중 동기화를 제공하지 않으므로 초기화 시점에만 호출해야 한다.
- `predict()`는 ZOH API이며 미래 입력 변화를 반영하려면 호출자가 `stepRK4()`를 반복해야 한다.
- key sample 메시지에는 timestamp, vehicle ID, frame ID, validity가 없다.
- reconstruction은 현재 mutable shared detail 상태 때문에 동시 호출에 안전하지 않다.
- 충돌 검사와 회피 알고리즘은 아직 구현되지 않았다.
- production 다기체 비행과 trajectory prediction을 함께 실행하는 end-to-end 검증은 아직 없다.

## 16. 다음 작업 권장 순서

1. `PredictionSamples.msg` 인터페이스 정의
2. reconstruction 상태를 `ReconstructTrajectory` 인스턴스 멤버로 이동
3. production용 `TrajectoryPredictionService` 또는 component 설계
4. ego 및 neighbor sample timestamp 동기화
5. 최소거리·CPA/TCPA 기반 충돌 판정 모듈 구현
6. HILS에서 2기체 crossing/head-on/overtake 시나리오 검증
7. 회피 guidance와 formation 복귀 상태 머신 연결
8. 5기체 HILS 및 실기체 배포 전 fault-injection 검증

## 17. 변경 추적 표

이 문서를 갱신할 때 아래 표에 현재 상태가 바뀐 항목만 추가한다.

| 날짜 | 기준 커밋 | 변경 내용 | 검증 결과 |
| --- | --- | --- | --- |
| 2026-08-06 | `6e4a3cc` | 현재 궤적 예측·재구성·HILS 구성 최초 정리 | 기존 Release build 및 17 tests 통과 상태 확인 |

