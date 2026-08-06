# Trajectory cone ROS bag 파이프라인

## 0. 외부 의존성 고정

trajectory belief full covariance 인터페이스는 다음 개인 fork와 커밋에 고정한다.
공식 PX4 upstream을 직접 수정하거나 해당 저장소의 `main`을 전제로 하지 않는다.

| 의존성 | fork/branch | 고정 commit |
|---|---|---|
| PX4-Autopilot | `sdfkjsdf/PX4-Autopilot` / `feature/trajectory-belief-covariance` | `f763fdd99f06fe53d72d948995a780d96f397fd1` |
| px4_msgs | `sdfkjsdf/px4_msgs` / `feature/trajectory-belief-message` | `f7741616e14c8330b54acdaf8daf382750896081` |

재현 시에는 branch 이름만 따르지 말고 위 commit을 명시적으로 checkout한다.

```bash
git clone https://github.com/sdfkjsdf/PX4-Autopilot.git
git -C PX4-Autopilot checkout f763fdd99f06fe53d72d948995a780d96f397fd1

git clone https://github.com/sdfkjsdf/px4_msgs.git
git -C px4_msgs checkout f7741616e14c8330b54acdaf8daf382750896081
```

## 1. 이번 단계의 범위

이번 단계는 다음 흐름이 Gazebo Classic headless SILS에서 끝까지 동작하는지 확인한다.

1. PX4 EKF의 동일 fusion horizon 평균과 full covariance 출력
2. EKF 9-state belief를 예측기 7-state belief로 변환
3. EKF 지연 시간을 현재 시각까지 보상
4. 4.5초, 10 Hz, 46점 nominal trajectory와 position covariance cone 생성
5. 필수 ROS 2 토픽을 rosbag2로 기록
6. bag 무결성, 시간 정렬, 공분산 PSD, Gazebo ground truth 포함률 분석

95% 경험적 포함률을 만족하도록 process noise를 보정하는 절차와 반복 시나리오의
통계적 합격 판정은 후속 검증 단계이다. 이번 단계에서는 포함률을 측정하지만 합격
조건으로 사용하지 않는다.

## 2. full covariance가 필요한 이유

PX4의 기존 `EstimatorStates`와 `VehicleOdometry` ROS 출력은 대각 분산만 제공한다.
대각 성분만 쓰면 다음 정보가 사라진다.

- 위치 오차 타원체의 회전 방향
- 위치와 속도 사이의 상관관계
- course, speed, roll로 좌표 변환할 때 필요한 교차항
- 미래 동역학을 통해 위치 불확실성으로 전달되는 상관관계

대각화는 항상 보수적인 처리도 아니다. 실제 타원체의 긴 축이 회전되어 있으면 어떤
방향에서는 위험을 과소평가하고 다른 방향에서는 과대평가할 수 있다. 따라서 PX4에
`EstimatorTrajectoryBelief` uORB/DDS 메시지를 추가했다.

이 메시지는 fusion horizon에서 다음 항목을 한 시각에 묶는다.

- body-FRD to earth-NED quaternion 4개
- NED velocity 3개
- local NED position 3개
- EKF error-state `[attitude error, velocity, position]`의 9×9 covariance 상삼각 45개
- publication timestamp와 fusion-horizon `timestamp_sample`

ROS 측은 `timestamp - timestamp_sample`만큼 7-state mean과 covariance를 먼저
전파한 뒤, 그 지점을 cone의 `t=0`으로 사용한다.

## 3. 반드시 기록할 토픽

기본 namespace는 `/px4_0`이다.

| 구분 | 토픽 | 필요한 이유 |
|---|---|---|
| 필수 | `/px4_0/fmu/out/estimator_trajectory_belief` | cone 초기 평균, full covariance, PX4 시간 기준 |
| 필수 | `/px4_0/collision_estimation/trajectory_cone` | 46점 평균, 3×3 position covariance, 미래 입력 기록 |
| 필수 | `/px4_0/fmu/out/vehicle_local_position_v1` | 현재 EKF 출력과 cone 시작점 진단 |
| 필수 | `/px4_0/fmu/out/vehicle_local_position_groundtruth_v1` | 미래 예측 위치의 Gazebo 정답 |
| 필수 | `/px4_0/fmu/out/vehicle_attitude_groundtruth` | 자세 정답과 simulator 상태 확인 |
| 권장 | `/px4_0/fmu/out/vehicle_odometry` | 기존 평균과 대각 분산 출력 비교 |
| 권장 | `/px4_0/fmu/out/vehicle_attitude` | EKF 자세 진단 |
| 권장 | `/px4_0/fmu/out/vehicle_status_v3` | arming, nav state, flight phase 분리 |
| 권장 | `/px4_0/fmu/out/airspeed_validated_v1` | airspeed 모델 오차 진단 |
| 권장 | `/px4_0/fmu/out/wind` | 풍속에 따른 모델 오차 분해 |
| 권장 | `/px4_0/collision_estimation/key_samples` | 기존 4시점 통신 표현과 cone 비교 |

`TrajectoryCone.prediction_inputs`에는 45개 적분 구간의
`[V_cmd, h_cmd, h_dot_cmd, a_lat_cmd]`가 함께 저장된다. 따라서 별도 setpoint
토픽이 누락되어도 해당 cone을 재현할 수 있다.

## 4. 파일 구조

기존 include/src 대칭 규칙을 유지한다.

```text
collision_avoidance/
├── include/collision_avoidance/estimation/trajectory_prediction/
│   ├── TrajectoryUncertainty.hpp
│   └── UncertaintyTypes.hpp
├── src/estimation/trajectory_prediction/
│   └── TrajectoryUncertainty.cpp
├── msg/
│   └── TrajectoryCone.msg
└── test/
    └── test_trajectory_uncertainty.cpp

testing_module/trajectory_prediction_hils/
├── include/trajectory_prediction_hils/estimation/
│   └── BeliefConePublisher.hpp
├── src/estimation/
│   └── BeliefConePublisher.cpp
├── scripts/
│   ├── record_trajectory_cone_bag.sh
│   ├── run_cone_scenarios.sh
│   └── process_cone_batch.sh
├── analysis/
│   ├── analyze_trajectory_cone_bag.py
│   ├── plot_trajectory_cone_results.py
│   └── summarize_cone_batch.py
└── result/
    ├── rosbag/   # 온라인 수집 데이터
    ├── raw/      # bag 사후 분석 데이터
    ├── plot/     # case별 PNG
    ├── summary/  # batch 요약
    └── log/      # launch/분석 로그
```

ROS 인터페이스 정의는 ROS 2 빌드 규칙 때문에 `msg/`에 둔다. 수학 자료형과 구현은
각각 `include/`와 대응하는 `src/`에 분리되어 있다.

## 5. 실행 방법

### 5.1 반복 시나리오: 기록과 분석 분리

`config/cone_scenario_matrix.yaml`이 case 목록, 반복 횟수, timeout의 단일 원본이다.
다음 명령은 실행 중에는 bag만 기록하고, 모든 case가 끝난 뒤에만 분석과 plot을
수행한다.

```bash
cd /home/hmcl/workspace/swarm-fixed-wing/source/swarm_fixed_wing_collision_avoidance/\
px4_ros2/ros2_ws/src/testing_module/trajectory_prediction_hils
./scripts/run_cone_scenarios.sh --profile coverage_core
```

실제 실행 전 전체 작업 목록 확인:

```bash
./scripts/run_cone_scenarios.sh --profile coverage_core --dry-run
```

수집과 분석을 서로 다른 시점에 실행할 수도 있다.

```bash
./scripts/run_cone_scenarios.sh --profile coverage_core \
  --batch-id coverage_01 --collect-only
./scripts/process_cone_batch.sh --batch-id coverage_01
```

이 분리는 Matplotlib, CSV/NPZ 변환, Mahalanobis 계산이 Gazebo/PX4 실행과 CPU를
경쟁하지 않게 한다. 단, 이것은 분석 부하를 제거하는 것이며 Gazebo의 물리 시간을
임의로 가속하는 기능은 아니다. 물리 시간 가속은 PX4 lockstep과 센서 발행 주기의
별도 재검증 후 적용해야 한다.

결과는 다음 경로에 생성된다.

```text
result/rosbag/<batch_id>/<case_id>_rNN/
result/raw/<batch_id>/<case_id>_rNN/
result/plot/<batch_id>/<case_id>_rNN/
result/summary/<batch_id>/
result/log/<batch_id>/
```

### 5.2 단일 smoke 실행

GUI를 열지 않는 통합 실행과 자동 bag 분석:

```bash
cd /home/hmcl/workspace/swarm-fixed-wing/source/swarm_fixed_wing_collision_avoidance
BAG_OUTPUT_DIR=/tmp/trajectory_cone_run \
  px4_ros2/ros2_ws/src/testing_module/trajectory_prediction_hils/scripts/launch_1vtol_replay.sh \
  --record-bag
```

bag만 별도로 분석:

```bash
source /opt/ros/humble/setup.bash
source /home/hmcl/workspace/swarm-fixed-wing/ros2_ws/install/setup.bash
ros2 run trajectory_prediction_hils analyze_trajectory_cone_bag.py \
  /tmp/trajectory_cone_run \
  --output /tmp/trajectory_cone_run_analysis \
  --namespace /px4_0
```

개별 bag 분석 산출물은 다음 파일이다.

- `summary.json`: 토픽 수, 누락, cone rate, PSD 오류, 포함률, 위치 오차 요약
- `cone_samples.csv`: cone 시각과 horizon별 예측·정답·오차·Mahalanobis 거리
- `cone_arrays.npz`: 후속 NumPy/Monte Carlo/Q 튜닝용 배열
- `result/plot/.../*.png`: horizon별 포함률·오차와 대표 cone
- `result/summary/<batch>/cases.csv`: 반복 실행 비교표
- `result/summary/<batch>/batch_summary.json`: batch 구조 무결성과 집계값

## 6. 2026-08-06 headless SILS smoke 결과

bag: `/tmp/trajectory_cone_smoke_20260806`

- 필수 토픽 누락: 0
- belief: 2,842개
- cone: 545개
- 유효하지 않은 cone: 0
- 미래 4.5초 ground truth가 모두 존재하는 cone: 501개
- 분석한 cone 단면: 23,046개
- 유한성/대칭/PSD 실패: 0
- cone 발행률: 10.004 Hz
- 전체 horizon 경험적 95% 포함률: 37.57%
- 4.5초 끝점 경험적 95% 포함률: 56.69%

생성·전달·기록·분석 smoke는 통과했다. 포함률은 아직 목표 95%보다 낮으므로 현재 Q를
비행용 안전 보장값으로 해석하면 안 된다. 다음 단계에서는 flight phase를 분리하고,
여러 입력·풍속·센서 조건의 bag을 모아 horizon별 Q 또는 모델 오차 항을 보정해야 한다.

## 7. 2026-08-06 자동 batch smoke 결과

batch ID: `cone_smoke_20260806_160203`

`smoke` 프로파일의 `S01`, `R15P`, `A02`를 GUI 없이 연속 실행했다. 시뮬레이션
중에는 rosbag만 기록하고 세 case가 모두 끝난 뒤 오프라인 분석과 plot을 수행했다.

- 수집 성공: 3/3
- rosbag 분석 및 plot 성공: 3/3
- 필수 토픽 누락: 0
- invalid cone: 0
- covariance 유한성/대칭/PSD 실패: 0
- cone 발행률: 모든 case 약 10 Hz
- 기록 용량: rosbag 약 16 MB, raw 분석 약 13 MB, plot 약 776 KB

| case | 전체 horizon 포함률 | 4.5초 포함률 | 평균 위치 오차 |
|---|---:|---:|---:|
| S01 | 25.45% | 38.46% | 6.33 m |
| R15P | 26.29% | 39.03% | 6.57 m |
| A02 | 25.50% | 34.00% | 7.26 m |

세 case 평균 포함률은 전체 horizon 25.75%, 4.5초 37.16%이다. 파이프라인 구조
검증은 통과했지만 nominal 95% cone의 통계 보정은 통과하지 않았다. 다음 데이터
수집에서는 정상상태, 기동 구간, 전환 구간을 분리하고 calibration/holdout 시나리오를
별도로 구성해야 한다.
