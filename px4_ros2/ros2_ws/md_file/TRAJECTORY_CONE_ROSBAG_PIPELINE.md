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
| 권장 | `/px4_0/fmu/out/vehicle_gps_position` | GNSS sample 지연과 EKF 위치 추종 오차 분해 |
| 권장 | `/px4_0/fmu/out/vehicle_attitude` | EKF 자세 진단 |
| 권장 | `/px4_0/fmu/out/vehicle_status_v3` | arming, nav state, flight phase 분리 |
| 권장 | `/px4_0/fmu/out/airspeed_validated_v1` | airspeed 모델 오차 진단 |
| 권장 | `/px4_0/fmu/out/wind` | 풍속에 따른 모델 오차 분해 |
| 권장 | `/px4_0/collision_estimation/key_samples` | 기존 4시점 통신 표현과 cone 비교 |
| 전파시험 필수 | `/px4_0/testing/trajectory_prediction_debug` | 실제 입력 적용 시각, case ID, cone epoch, 46점 전체 상태와 ZOH 입력 계약 확인 |

정식 coverage 분석은 위 raw topic과 함께 기록되는 다음 공통 NED topic을 사용한다.

| 공통 좌표 topic | 내용 |
|---|---|
| `/common/px4_0/trans_estimator_trajectory_belief` | EKF reference를 공통 원점으로 평행이동한 belief |
| `/common/px4_0/trans_vehicle_local_position` | 공통 NED의 현재 EKF local position |
| `/common/px4_0/trans_vehicle_odometry` | 포메이션 제어와 공유하는 공통 odometry |
| `/common/px4_0/trans_vehicle_local_position_groundtruth` | simulator reference를 공통 원점으로 옮긴 정답 |
| `/common/px4_0/trans_trajectory_cone` | 모든 평균점을 공통 NED로 옮긴 cone |

`TrajectoryCone.prediction_inputs`에는 현재 선택한 후보 입력
`[V_cmd, h_cmd, h_dot_cmd, a_lat_cmd]`이 45개 적분 구간에 ZOH로 반복 저장된다.
predictor는 YAML의 미래 입력 전환을 미리 읽지 않는다. 다음 0.1초 계산 시점에는 새
상태와 현재 입력으로 4.5초 cone을 다시 만든다. 따라서 별도 setpoint 토픽이 누락되어도
각 시점에서 평가한 후보 입력을 재현할 수 있다.

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
├── include/collision_avoidance/coordinate/
│   ├── CommonNedTransform.hpp
│   └── TransferSameCoordinate.hpp
├── src/coordinate/
│   ├── CommonNedTransform.cpp
│   └── TransferSameCoordinate.cpp
└── test/
    ├── test_trajectory_uncertainty.cpp
    └── test_common_ned_transform.cpp

testing_module/trajectory_prediction_hils/
├── include/trajectory_prediction_hils/estimation/
│   └── BeliefConePublisher.hpp
├── include/trajectory_prediction_hils/testing/
│   ├── PropagationTestMode.hpp
│   └── PropagationTestExecutor.hpp
├── src/estimation/
│   └── BeliefConePublisher.cpp
├── src/testing/
│   ├── PropagationTestMode.cpp
│   └── PropagationTestExecutor.cpp
├── src/nodes/
│   └── trajectory_prediction_sils_test_main.cpp
├── msg/
│   └── TrajectoryPredictionDebug.msg
├── config/
│   ├── propagation_test_params.yaml
│   └── propagation_scenario_matrix.yaml
├── scripts/
│   ├── launch_propagation_test.sh
│   ├── record_trajectory_cone_bag.sh
│   ├── run_propagation_scenarios.sh
│   ├── run_cone_scenarios.sh
│   └── process_cone_batch.sh
├── analysis/
│   ├── analyze_trajectory_cone_bag.py
│   ├── expand_propagation_scenarios.py
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

기본 `--coordinate-frame common`이 정식 분석 모드이다. 이전 bag의 local-frame 결과를
진단 목적으로 다시 볼 때만 `--coordinate-frame local`을 명시한다.

### 5.3 FixedWing 트림 후 ZOH 전파시험

기존 replay case는 준비·기동·회복 구간이 섞일 수 있으므로 동역학 전파 오차를 직접
측정할 때는 전용 노드를 사용한다. 이 노드는 자동 이륙과 VTOL 천이가 완료되어 PX4가
FixedWing 상태가 되면 먼저 횡가속도와 상승률이 0인 트림 setpoint를 발행한다. 이후
실측 calibrated airspeed, 수직속도와 roll이 허용 범위에 2초간 연속으로 머문 뒤에만
수평 회피 입력과 cone 생성을 시작한다.

```bash
cd /home/hmcl/workspace/swarm-fixed-wing/source/swarm_fixed_wing_collision_avoidance
TEST_CASE_ID=ZOH_ALATP2P628_V20 \
TEST_V_CMD=20.0 TEST_ALAT_CMD=2.628 \
BAG_OUTPUT_DIR=/tmp/propagation_trim_lateral_v20 \
BAG_ANALYSIS_DIR=/tmp/propagation_trim_lateral_v20_analysis \
  px4_ros2/ros2_ws/src/testing_module/trajectory_prediction_hils/scripts/\
launch_propagation_test.sh --record-bag
```

기본 시험 계약은 다음과 같다.

1. 트림 구간에서 `a_lat_cmd=0`, `h_dot_cmd=0`을 유지한다.
2. 실측 CAS 오차 1.0 m/s 이하, `|vz|` 0.5 m/s 이하, `|roll|` 5도 이하가 2초간
   유지되어야 트림 게이트를 통과한다.
3. 첫 수평 회피 setpoint가 실제 발행된 시각을 `applied_input_timestamp`로 기록한다.
4. cone은 이 시각과 같거나 더 새로운 EKF belief만 사용한다.
5. 동일한 `[V_cmd, h_cmd, 0, a_lat_cmd]`를 45개 적분 구간에 ZOH로 넣는다.
6. 5초 동안 10 Hz로 4.5초 cone을 생성한다.
7. 마지막 cone의 ground truth가 확보되도록 같은 입력을 최소 4.5초 더 유지한다.
8. analyzer의 `--require-alignment`가 horizon 0 중앙값 0.5 m 이하, 95 percentile
   1.0 m 이하인지 검사한다.

여기서 `V_cmd`는 회피 순간에 새로 가하는 종방향 회피 입력이 아니라 시험을 시작할
운용 속도점이다. 회피 후보는 `a_lat_cmd`이며 `h_dot_cmd=0`은 전체 시험에서 강제한다.

절대 위치 오차는 `truth(t)-prediction(t)`이고, 전파 오차는 시작점 차이를 제거한
다음 값이다.

```text
e_prop(t) = [truth(t) - truth(0)] - [prediction(t) - prediction(0)]
```

따라서 horizon 0 오차는 EKF 평균·좌표·시각 정렬 진단에, `e_prop`는 predictor
동역학 진단에 사용한다. 둘을 합친 절대 종점 오차만 보고 Q를 키우지 않는다.

### 5.4 ZOH 반복 검증 matrix

`config/propagation_scenario_matrix.yaml`이 후보 입력, profile, 반복 횟수와 Gazebo seed의
단일 원본이다. 새 전파시험에서는 replay용 `cone_scenario_matrix.yaml`을 사용하지 않는다.

```bash
cd /home/hmcl/workspace/swarm-fixed-wing/source/swarm_fixed_wing_collision_avoidance/\
px4_ros2/ros2_ws/src/testing_module/trajectory_prediction_hils

# 실행 계획만 출력
./scripts/run_propagation_scenarios.sh --profile model_identification --dry-run

# 실행 중에는 bag만 수집
./scripts/run_propagation_scenarios.sh \
  --profile model_identification --batch-id propagation_model_01 --collect-only

# 수집 완료 후 별도 오프라인 분석
./scripts/process_cone_batch.sh \
  --batch-id propagation_model_01 --propagation-test
```

profile 순서는 다음과 같이 고정한다.

| profile | 실행 수 | 용도 |
|---|---:|---|
| `wiring_smoke` | 3 | 직선·좌선회·우선회 각 1회, 전체 배선 확인 |
| `model_identification` | 22 | 속도/횡가속도 단일 채널 11 case × 2 seed |
| `model_holdout` | 12 | 속도+횡가속도 결합 4 case × 3 seed |
| `cone_calibration` | 21 | 외곽 입력 7 case × 3 seed, 모델 고정 후 Q 보정 |
| `cone_holdout` | 40 | 중간값·결합 입력 8 case × 5 seed, 최종 독립 검증 |
| `regression_all` | 15 | 유지 중인 모든 case 1회 회귀 검사 |

각 반복은 manifest에 고정된 서로 다른 `gzserver --seed`를 사용한다. calibration과
holdout은 case뿐 아니라 seed 범위도 분리한다. `model_holdout`이 통과하기 전에는
`cone_calibration`을 실행하거나 Q를 변경하지 않는다.

운용 회피 후보는 속도와 횡가속도만 사용하며 모든 case에서 `h_dot_cmd=0`을 강제한다.
resolver는 0이 아닌 height-rate case를 오류로 거부한다. 이는 3차원 상태와 고도
불확실성을 없앤다는 뜻이 아니다. 선회 중 lift/에너지 결합으로 실제 고도가 변할 수
있으므로 `Down`, `h_dot`과 수직 covariance는 계속 예측·평가하되 상승을 회피 입력으로
선택하지 않는다.

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

## 8. ZOH·causal evaluator 전환과 재실행 결과

batch ID: `cone_smoke_causal_20260806`

평가 의미를 "현재 선택한 입력을 유지했을 때의 후보 trajectory"로 고정하기 위해 다음을
반영했다.

1. PX4 simulator ground-truth attitude/global/local position에 HIL
   `timestamp_sample`을 채운다.
2. replay mode 활성 전에는 `TrajectoryCone`을 발행하지 않는다.
3. 미래 YAML schedule-aware 예측을 제거하고 현재 입력 ZOH로만 4.5초를 예측한다.
4. offline evaluator는 이후 cone의 현재 입력이 달라지는 첫 시각부터 기존 cone의
   horizon을 평가에서 제외한다.
5. ground truth는 유효한 `timestamp_sample`만 정식 분석에 사용한다. 과거 bag 진단은
   명시적인 `--allow-publication-time-fallback`에서만 허용된다.

causal evaluator의 입력 동일성 허용 오차는 순서대로
`[V_cmd 0.05 m/s, h_cmd 0.10 m, h_dot_cmd 0.05 m/s, a_lat_cmd 0.05 m/s²]`이다.
대표 cone plot은 횡가속 입력, 수직속도 입력, 중앙 cone 순으로 기동 시점을 선택하며
입력 전환으로 잘린 경우 유효한 causal horizon만 그린다.

| case | cone | non-ZOH | causal censor point | full 4.5 s cone | 전체 포함률 | 4.5 s 포함률 |
|---|---:|---:|---:|---:|---:|---:|
| S01 | 200 | 0 | 0 | 157 | 29.36% | 66.88% |
| R15P | 200 | 0 | 991 | 112 | 26.18% | 74.11% |
| A02 | 200 | 0 | 1,699 | 93 | 28.44% | 68.82% |

- 수집/분석/plot 성공: 3/3
- ground-truth time source: 모든 case `timestamp_sample`
- ground-truth sample timestamp unique: 모든 메시지에서 고유, 약 42.4~42.6초 span
- preflight cone 제거: 이전 case당 약 395개에서 replay cone 200개로 감소
- invalid cone, covariance 유한성/대칭/PSD 실패: 0
- cone 발행률: 약 10 Hz

구조와 평가 의미의 smoke는 통과했다. 다만 horizon 0 평균 위치 오차가 약 2.5 m이고
포함률이 0%이므로 현재 결과를 cone 정확도 합격으로 해석하지 않는다. bag을 분해하면
EKF belief의 fusion-horizon position 자체가 같은 sample time의 simulator ground truth와
약 2.1 m 차이가 나며, 0.152초 fusion-horizon delay를 전파한 cone 시작점은 약 2.65 m
차이가 난다. 이는 이번에 제거한 publication timestamp 오류와 별개의 EKF 추정 오차 및
초기 covariance calibration 문제이다. 반복 seed 기반 Q 보정 전에는 95% 안전 보장값으로
사용하지 않는다.

## 9. 공통 NED 좌표계 적용 결과

batch ID: `cone_smoke_common_20260806`

`coordinate_transformer_node`에는 기존 포메이션의 `spawn_offset` 모드를 유지하면서,
HILS 검증용 `geodetic_reference` 모드를 추가했다. 공통 원점은
`(47.397742°, 8.545594°, 488.0 m AMSL)`이다. 각 메시지의 EKF 또는 simulator
`ref_lat/ref_lon/ref_alt`를 이 원점에 대한 NED 평행이동으로 바꾼다.

```text
p_common = p_local + translation(reference_local -> reference_common)
P_common = P_local
```

축 회전은 없으므로 belief 9×9 covariance와 cone 3×3 position covariance는 그대로
유지한다. analyzer는 raw/common 메시지를 timestamp로 짝지어 covariance가 변하지
않았는지 자동 검사한다.

- 수집/분석/plot: 3/3 성공
- 공통 원점 검사: 3/3 성공
- raw/common cone 매칭: case별 200/200
- belief/cone covariance 최대 변화: 0
- invalid cone, non-ZOH cone, covariance PSD 실패: 0

| case | local-frame t=0 Down 오차 중앙값 | common-frame 값 | common-frame t=0 오차 중앙값 |
|---|---:|---:|---:|
| S01 | 0.647 m | 0.013 m | 2.630 m |
| R15P | 0.653 m | -0.001 m | 2.650 m |
| A02 | 0.723 m | -0.110 m | 2.579 m |

좌표 원점 때문에 생긴 수직 오차는 제거되었다. 그러나 수평 오차 약 2.4~2.6 m가
남아 horizon 0 포함률은 여전히 0%이다. 따라서 남은 오차는 common/local 좌표 표현
불일치가 아니라 EKF 평균 추정 또는 fusion-horizon 지연 보상에서 분리해야 한다.

## 10. Horizon 0 분해와 예비 Q calibration

기존 common-frame bag을 다시 분해하면 timestamp 기반 fusion-horizon 지연은 중앙값
0.152초이며, 이 구간의 실제 이동과 predictor 이동 차이는 중앙값 0.024~0.040 m에
불과했다. 반면 cone 시작점은 PX4 current local-position과 약 0.02 m로 일치하지만
Gazebo truth에는 약 2.5 m 뒤처졌다. 세 시나리오의 위치 시계열 정렬 결과 이 PX4 출력
추종 지연은 0.125~0.130초로 반복되었다.

HILS 설정에 별도 `uncertainty.estimator_output_delay_s=0.13`을 추가해 belief 평균과
공분산을 더 전파했다. 이 값은 `timestamp - timestamp_sample`을 대체하지 않고 더해진다.
실기체나 다른 센서 설정에서는 반드시 다시 측정해야 하며 기본 라이브러리 의미로
일반화하지 않는다.

analyzer에는 다음 산출물을 추가했다.

- `initial_alignment.csv`: fusion 평균, timestamp 지연, 추가 출력 지연, horizon 0 오차 분해
- `coverage_by_horizon.csv`: horizon별 표본 수, 포함률, 오차, Mahalanobis 통계
- `cone_evaluation.csv`: cone별 전체 포함, 최초 이탈 시각, 최대 Mahalanobis 거리
- `trajectory_survival_by_horizon.png`: horizon까지 한 번도 이탈하지 않은 cone 비율

추가 지연 보상 후 horizon 0 오차 중앙값은 약 0.09~0.18 m로 감소했다. 이후 모든 Q를
동일 비율로 조정하는 `uncertainty.q_scale`을 배선하고 다음 smoke를 비교했다.

| Q scale | 실행 | 전체 점별 포함률 | 4.5초 포함률 | 전체 4.5초 궤적 포함률 |
|---:|---:|---:|---:|---:|
| 0.0 | 3 | 85.18% | 55.01% | 55.01% |
| 0.5 | 3 | 98.98% | 94.89% | 94.68% |
| 0.8 | 9 | 99.93% | 99.39% | 99.39% |

scale 0.8의 3 case × 3회 반복에서는 invalid/non-ZOH/covariance/common-contract 실패가
모두 0이었다. A02의 4.5초 포함률은 반복 평균 98.18%, S01과 R15P는 100%였다.
오프라인 선형 보간상 scale 0.7이 A02에서 약 96.4%이지만 holdout 여유가 없으므로
HILS 예비 기본값은 0.8로 유지한다. 이 값은 calibration 시나리오에 맞춘 결과이며,
독립 holdout과 시간축 joint covariance 없이 정식 95% trajectory-tube 보장으로
해석하지 않는다.

## 11. 2026-08-07 FixedWing ZOH 전파시험 smoke

batch ID: `propagation_v1_straight_20260807_02`

`ZOH_V20_STRAIGHT`를 GUI 없이 한 번 실행했다. PX4 FixedWing 전환 직후
`V_cmd=20 m/s`, `h_dot_cmd=0 m/s`, `a_lat_cmd=0 m/s²`를 적용했고, 같은 입력을
cone 생성 5.0초와 미래 정답 수집 4.6초 동안 유지했다.

- cone/debug message: 각각 50개
- valid 46점, ZOH, 입력 적용 선행, cone epoch 대응 계약: 50/50, 통과
- complete 4.5초 ground-truth cone: 50/50
- horizon 0 정렬 오차: 중앙값 0.471 m, 95 percentile 0.541 m, gate 통과
- start-aligned 4.5초 전파 오차: 중앙값 3.757 m, 95 percentile 7.077 m
- 대표 cone: horizon 0 0.475 m, 절대 종점 4.306 m, 전파 종점 3.963 m
- 4.5초 점별 포함률 및 전체 궤적 포함률: 각각 40%

이번 결과는 시작점 불일치와 전파 모델 오차의 분리에 성공했다는 smoke 결과다. 4.5초
전파 오차와 포함률은 합격값이 아니며, 다음 단계에서 속도·횡가속도 후보를 각각
일정하게 유지하는 사례로 확장한 뒤 모델 파라미터와 Q를 별도로 보정해야 한다.

추가된 대표 plot은 다음 두 의미를 분리한다.

- `trajectory_cone_example.png`: 공통 NED 절대 위치와 두 개의 `t=0` marker
- `trajectory_start_aligned.png`: 두 시작점을 원점으로 옮긴 순수 변위 비교
- `propagation_error_by_horizon.png`: horizon별 start-aligned 전파 오차 통계

## 12. 2026-08-07 초기 3-channel 탐색 smoke

batch ID: `propagation_wiring_smoke_20260807_01`

새 matrix runner의 최초 배선 확인에서 직선, 상승, 선회 사례를 서로 다른 Gazebo
seed에서 연속 실행했다.

- 수집 성공: 3/3
- 오프라인 분석/plot 성공: 3/3
- propagation debug 계약: 3/3 통과
- horizon 0 정렬 gate: 3/3 통과
- invalid/non-ZOH/covariance/common-coordinate 실패: 0

| case | horizon 0 중앙값 | 4.5초 전파 오차 중앙값 | 4.5초 전파 오차 p95 | 종점/전체 궤적 포함률 |
|---|---:|---:|---:|---:|
| `ZOH_V20_STRAIGHT` | 0.335 m | 3.556 m | 6.886 m | 54% |
| `ZOH_HUP2_V20` | 0.271 m | 6.654 m | 9.829 m | 0% |
| `ZOH_ALATP2P628_V20` | 0.252 m | 3.851 m | 12.165 m | 58% |

세 case 평균 horizon 0 오차는 0.286 m로 정렬 문제가 주원인이 아님을 재확인했다.
그러나 운용 회피 입력을 다시 검토한 결과 별도 상승 회피를 사용하지 않기로 확정했다.
따라서 `ZOH_HUP2_V20`은 파이프라인 탐색 기록으로만 남기며 모델 fitting,
calibration, holdout에서 제외한다. 현재 matrix의 wiring smoke는 직선과 좌·우 선회로
교체했고 모든 case에 `h_dot_cmd=0` 제약을 추가했다. 선회 중 발생하는 실제 고도 변화와
수직 오차는 계속 `trajectory_vertical_start_aligned.png`에서 감시한다.

## 13. 2026-08-07 트림 후 수평 회피 smoke

batch ID: `propagation_trim_lateral_20260807_02`

run ID: `ZOH_ALATP2P628_V20_r01`, Gazebo seed `11002`

VTOL의 FixedWing 천이 직후 데이터를 바로 평가하지 않고 `V_cmd=20 m/s`,
`a_lat_cmd=0`, `h_dot_cmd=0`으로 먼저 정착시켰다. 트림 게이트는
`CAS=19.47 m/s`, `vz=-0.149 m/s`, `roll=0.02 deg`에서 통과했고, 이후
`a_lat_cmd=2.628 m/s²`만 적용했다.

- cone/debug message: 각각 50개, valid/ZOH/lateral-only 계약 50/50
- 입력 시각보다 오래된 EKF source: 0개, 인과성 계약 50/50
- 트림 재구성 gate: 통과
  - 직전 2초 최대 CAS 오차 0.967 m/s
  - 최대 `|vz|` 0.344 m/s
  - 최대 `|roll|` 0.108 deg
- horizon 0 정렬 오차: 중앙값 0.204 m, p95 0.231 m, gate 통과
- start-aligned 4.5초 전파 오차: 중앙값 1.124 m, p95 1.274 m
- 대표 cone의 horizon 0/4.5초 전파 오차: 0.213 m / 1.206 m
- invalid/non-ZOH/covariance/common-coordinate 실패: 0

`trajectory_start_aligned.png`에서 예측과 ground truth의 수평 변위가 같은 원점에서
가깝게 겹친다. `trajectory_vertical_start_aligned.png`에는 `h_dot_cmd=0`인데도 발생한
약 0.23 m의 수직 전파 차이가 남는다. 이는 상승 회피를 시험한 결과가 아니라 선회 중
lift/에너지 결합을 상태·불확실성 차원에서 계속 감시한 결과다.
