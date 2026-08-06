# Trajectory cone ROS bag 파이프라인

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
│   └── record_trajectory_cone_bag.sh
└── analysis/
    └── analyze_trajectory_cone_bag.py
```

ROS 인터페이스 정의는 ROS 2 빌드 규칙 때문에 `msg/`에 둔다. 수학 자료형과 구현은
각각 `include/`와 대응하는 `src/`에 분리되어 있다.

## 5. 실행 방법

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

분석 산출물은 다음 세 파일이다.

- `summary.json`: 토픽 수, 누락, cone rate, PSD 오류, 포함률, 위치 오차 요약
- `cone_samples.csv`: cone 시각과 horizon별 예측·정답·오차·Mahalanobis 거리
- `cone_arrays.npz`: 후속 NumPy/Monte Carlo/Q 튜닝용 배열

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
