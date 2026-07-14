# 프로젝트 파일 기능 상세 참조

> 기준: 현재 `ros2_ws` 소스 트리.  
> 이 문서는 배포·인수인계에 필요한 **프로젝트 코드와 설정 파일**을 파일 단위로 설명한다. PX4에서 그대로 가져온 `px4_msgs/msg/*.msg` 수백 개와 `px4_ros_com` 예제는 개별 분석 대상이 아니므로 패키지 단위로 정리했다.

## 1. 전체 동작 관계

```text
PX4 odometry / status
        │
        ├─ coordinate_transformer_node ──> 공통 원점 odometry
        │                                      │
        │                                      ▼
        └────────────────────────────> vtol_guidance_node
                                              │
                    Preflight → FormationMode → FlockingGuidance
                                              │
                                      PX4 fixed-wing setpoint

사전 정의 setpoint YAML → trajectory_replay_node → PX4 SITL
                                      │
                                      ├─ TrajectoryPredict (RK4)
                                      ├─ ReconstructTrajectory (cubic spline)
                                      └─ CSV → Python 분석/그래프
```

`collision_avoidance`의 `collision_estimation` 라이브러리가 궤적 예측 알고리즘의 유일한 구현이다. `trajectory_prediction` 패키지는 이를 링크해 SITL에서 파라미터를 검증하는 harness 역할을 한다.

## 2. 워크스페이스 최상위

| 경로 | 기능 | 배포 시 처리 |
|---|---|---|
| `src/` | ROS 2 패키지 소스 | 필수 전달 |
| `docker/Dockerfile` | ROS Humble 기반 collision_avoidance 컨테이너 이미지 정의 | Docker 사용 시 필수 |
| `docker/entrypoint.sh` | 컨테이너에서 ROS setup을 source하고 실행 예시를 표시 | Docker 사용 시 필수 |
| `md_file/` | 설계/인수인계/본 문서 | 전달 권장 |
| `results/` | 과거 CSV, 그래프, 분석 보고서 | 재현에 필수는 아니며 결과 공유용 |
| `build/`, `install/`, `log/` | colcon 생성물 | 전달하지 않고 대상 PC에서 재생성 |
| `terminal.txt` | 개발 중 사용한 명령어 메모 | 참고용, 배포 절차의 기준 문서는 아님 |

## 3. `src/collision_avoidance` — 다기체 편대 비행

### 빌드/패키지 정의

| 파일 | 기능 |
|---|---|
| `CMakeLists.txt` | `collision_estimation` 정적 라이브러리, `vtol_guidance_node`, `coordinate_transformer_node`를 빌드·설치한다. 외부 패키지에서 prediction 라이브러리를 링크할 수 있도록 export한다. |
| `package.xml` | ament, Eigen, rclcpp, PX4 메시지/interface 라이브러리 의존성을 선언한다. |

### 실행 파일과 구현

| 파일 | 기능 |
|---|---|
| `src/main.cpp` | `vtol_guidance_node` 진입점. ROS node, Preflight mode, Formation mode, ModeExecutor를 생성·등록하고 spin한다. |
| `src/transformer_main.cpp` | `coordinate_transformer_node` 진입점. `TransferSameCoordinate`를 생성하여 spin한다. |
| `src/modes/VtolPreflightMode.cpp` | 멀티콥터 이륙, VTOL→고정익 전환, cruise 안정화 단계를 수행한다. 모든 기체의 fixed-wing 전환 완료를 기다리는 로직도 가진다. |
| `src/modes/FormationMode.cpp` | 편대 비행 ModeBase 구현. 공통 좌표 odometry를 구독하고, 별도 실시간 루프에서 Flocking 가이던스를 계산해 fixed-wing setpoint를 갱신한다. |
| `src/modes/VtolGuidanceExecutor.cpp` | preflight와 formation mode의 실행 순서 및 전환 상태를 관리하는 ModeExecutor 구현이다. |
| `src/guidance/FlockingGuidance.cpp` | 이웃 기체 상태로 alignment/cohesion/separation 가속도를 계산하고, 기체 제한을 반영해 PX4 고정익 setpoint로 변환한다. |
| `src/utils/TransferSameCoordinate.cpp` | 각 PX4가 가진 로컬 odometry 원점에 spawn offset을 더해 하나의 공통 좌표계로 변환하여 republish한다. |

### 공통 헤더

| 파일 | 기능 |
|---|---|
| `include/collision_avoidance/modes/VtolPreflightMode.hpp` | preflight mode의 상태, PX4 setpoint 객체, 전환 관련 API를 선언한다. |
| `include/collision_avoidance/modes/FormationMode.hpp` | 편대 mode, 구독자, SPSC queue, 실시간 스레드와 가이던스 객체를 선언한다. |
| `include/collision_avoidance/modes/VtolGuidanceExecutor.hpp` | mode 실행 상태(state machine)와 executor API를 선언한다. |
| `include/collision_avoidance/guidance/FlockingGuidance.hpp` | flocking 파라미터, 2D 가속도 계산 및 fixed-wing setpoint 변환 API를 선언한다. |
| `include/collision_avoidance/utils/TransferSameCoordinate.hpp` | 원시/변환 odometry의 구독·발행자와 spawn offset 데이터를 선언한다. |
| `include/collision_avoidance/AirframeLimits.hpp` | airspeed, climb/sink, roll, 중력 등 기체 성능 제한을 담는 자료형이다. |
| `include/collision_avoidance/StateType.hpp` | 기체 상태, RT↔main thread 전달용 setpoint, fallback 표식을 정의한다. 좌표 단위는 주석의 NED/ENU 규칙을 따른다. |
| `include/collision_avoidance/spsc_queue.hpp` | RT thread와 ROS callback thread 사이 전달에 쓰는 lock-free single-producer/single-consumer 큐다. |

### 궤적 추정 라이브러리

| 파일 | 기능 |
|---|---|
| `include/.../trajectory_prediction/PredictTypes.hpp` | 7-state(`p_n`, `p_e`, `h`, `V`, `psi`, `h_dot`, `phi`) 예측 상태, 입력, 제한값, key sample 자료형을 선언한다. |
| `include/.../trajectory_prediction/TrajectoryPredict.hpp` | RK4 1-step 및 고정 horizon 예측 API를 선언한다. 입력 포화, state safety, roll/고도 동역학을 캡슐화한다. |
| `src/collision_estimation/trajectory_prediction/TrajectoryPredict.cpp` | 예측기의 실제 ODE와 RK4 적분을 구현한다. PX4 시간상수·속도·roll·상승률 제한을 적용한다. |
| `include/.../reconstruct_trajectory/ReconstructTrajectory.hpp` | key sample의 유효성 검사, clamped cubic spline 계수, 재구성 position/velocity API를 선언한다. |
| `src/collision_estimation/reconstruct_trajectory/ReconstructTrajectory.cpp` | key sample로 cubic spline을 계산하고 임의 시각의 위치·속도를 복원한다. |

### 설정과 스크립트

| 파일 | 기능 / 변경 시 주의 |
|---|---|
| `config/flocking_params.yaml` | 실제 `vtol_guidance_node`가 읽는 통합 파라미터. flocking gains, desired distance, sample period, 기체 한계, 고도 유지 gain을 설정한다. |
| `config/airframe_spec.yaml` | PX4/QGC 파라미터를 바탕으로 한 별도 기체 사양 파일. 현재 가이던스 기본 실행은 `flocking_params.yaml`을 쓰므로 둘의 값이 달라지지 않게 관리한다. |
| `config/ros_params.yaml` | coordinate transformer의 기체 수와 각 기체 spawn offset을 설정한다. |
| `config/spawn_config.yaml` | Gazebo에 생성할 기체 수와 초기 위치를 설정한다. `ros_params.yaml`과 반드시 일치해야 한다. |
| `scripts/launch_5vtol.sh` | 설정 두 파일의 일치 여부를 검사한 뒤 N대 PX4/Gazebo Classic SITL을 생성한다. `~/PX4-Autopilot` 경로를 가정한다. |
| `scripts/monitor_swarm.sh` | `/swarm_status/vehicle_N` 토픽을 한 화면에서 주기적으로 출력한다. |
| `scripts/convert_px4_params.py` | QGroundControl `.params` export를 읽어 airspeed, climb/sink, roll 제한이 들어간 YAML로 변환한다. 기본 출력은 `config/airframe_spec.yaml`이므로 덮어쓰기 주의. |

## 4. `src/trajectory_prediction` — 재생·예측 검증 harness

### 빌드/테스트

| 파일 | 기능 |
|---|---|
| `CMakeLists.txt` | `trajectory_replay_node`를 빌드하고 `collision_avoidance::collision_estimation`을 링크한다. config를 install하고 unit test도 등록한다. |
| `package.xml` | replay node에 필요한 ROS/PX4/yaml/예측 라이브러리 의존성을 선언한다. |
| `test/test_trajectory_predict.cpp` | collision_estimation prediction 라이브러리의 동작·안전성 단위 테스트다. |

### C++ 소스와 헤더

| 파일 | 기능 |
|---|---|
| `src/main.cpp` | node, preflight/replay mode, logger, predictor 및 prediction timer를 조립한다. YAML 파라미터를 읽어 예측기를 초기화하고 CSV 로깅을 시작한다. |
| `src/VtolPreflightMode.cpp` | 단일 VTOL을 이륙·고정익 전환·cruise 상태로 넘기는 preflight 구현이다. |
| `src/VtolGuidanceExecutor.cpp` | preflight 완료 후 replay mode를 실행하는 executor state machine이다. |
| `src/TrajectoryReplayMode.cpp` | setpoint 시퀀스를 시간에 따라 조회하고 PX4 setpoint로 적용한다. 별도 RT loop에서 replay 시간을 관리한다. |
| `src/SetpointSequencer.cpp` | 시퀀스 YAML을 읽고 시각별 airspeed/height rate/lateral acceleration을 ZOH 방식으로 반환한다. 안전 제한과 종료 정책도 처리한다. |
| `src/TrajectoryLogger.cpp` | PX4 odometry·airspeed·applied setpoint·예측 결과를 수집하여 timestamp CSV와 spline CSV로 기록한다. |
| `include/.../VtolPreflightMode.hpp` | 단일기 preflight mode 선언이다. |
| `include/.../VtolGuidanceExecutor.hpp` | replay 실행 executor 선언이다. |
| `include/.../TrajectoryReplayMode.hpp` | replay mode의 sequencer, logger, RT loop 선언이다. |
| `include/.../SetpointSequencer.hpp` | segment 자료형, safety limit, YAML sequence lookup API 선언이다. |
| `include/.../TrajectoryLogger.hpp` | 측정 snapshot, applied setpoint, 46점 예측/재구성 trajectory, CSV writer API 선언이다. |
| `include/.../StateType.hpp` | replay mode 내부 상태와 RT 전달용 자료형이다. |
| `include/.../spsc_queue.hpp` | replay RT thread와 main thread 사이의 lock-free 큐다. |

### 설정 파일

| 파일 | 기능 / 변경 시 주의 |
|---|---|
| `config/replay_params.yaml` | CSV prefix·로깅/예측 주기·4.5초 endpoint-inclusive horizon을 설정한다. 현재 기본은 10 Hz 예측, 46 points이다. |
| `config/airframe_spec.yaml` | SITL/실기체 airspeed·상승/하강·roll 한계와 PX4 time constant, 고도 PD 계수 `b_h`를 설정한다. 반드시 대상 기체 PX4 값으로 보정한다. |
| `config/setpoint_sequence.yaml` | 기본 시험 입력 시퀀스다. 각 segment의 시작/끝 시간, airspeed, ENU height rate, lateral acceleration과 종료 정책을 정의한다. |
| `config/case_matrix.yaml` | 모든 시험 case의 phase, 입력 채널, 기대 time constant, 분석 메타데이터를 분류한다. |
| `config/cases/*.yaml` | V/H/P/S/C/A/LH/R별 개별 자극 시퀀스다. 현재 `run_all_cases.sh`에서 활성화된 기본 묶음은 R-series 6개다. |

### 실행·분석 스크립트

| 파일 | 기능 |
|---|---|
| `scripts/launch_1vtol_replay.sh` | Agent → Gazebo → PX4 → SDF spawn → topic ready 확인 → replay node 순서로 단일 SITL 시험을 자동화하고, 종료 시 자식 프로세스를 정리한다. `PX4_DIR`, `ROS2_WS` 환경변수로 경로를 바꿀 수 있다. |
| `scripts/run_all_cases.sh` | case YAML을 하나씩 주입해 SITL을 반복 실행하고 `/tmp` CSV를 `results/cases/`로 옮긴다. |
| `scripts/analyze_cases.py` | case matrix와 CSV로 step response fitting, steady bias, coupling, avoidance 오차 그래프 및 `analysis_report.md`를 생성한다. SciPy가 필요하다. |
| `scripts/compare_single_input.py` | 특정 single-input case의 prediction과 PX4 측정값을 4.5초 window로 비교해 그래프/오차를 낸다. |
| `scripts/chunk_analysis.py` | rolling prediction CSV에서 4.5초 간격 chunk를 추출하여 trajectory/RMSE를 분석한다. |
| `scripts/compare_spline_reconstruct.py` | RK4 prediction CSV와 cubic-spline reconstruction CSV를 같은 tick 기준으로 비교한다. |
| `scripts/compare_b_h_grid.sh` | 여러 `b_h` 값으로 생성된 CSV를 찾아 chunk 분석 결과를 한 번에 비교한다. 현재 Python·workspace 절대 경로를 수정해야 할 수 있다. |

## 5. 외부/참고 패키지

| 경로 | 역할 | 관리 원칙 |
|---|---|---|
| `src/px4_msgs/` | PX4 uORB 메시지의 ROS 2 `.msg`/`.srv` 정의와 생성 규칙이다. `msg/*.msg`는 각 하나가 동일 이름 ROS message interface를 정의한다. PX4 펌웨어 버전과 동기화해야 한다. |
| `src/px4_ros_com/` | PX4↔ROS 2 통신 예제와 launch/test 코드다. 프로젝트 핵심 노드가 직접 수정·실행하는 대상은 아니며 참고/호환성 확인용이다. |
| `src/px4-ros2-interface-lib/` | `px4_ros2_cpp` 라이브러리와 mode/setpoint API, 예제다. 이 프로젝트의 `ModeBase`, `ModeExecutorBase`, VTOL/fixed-wing setpoint 클래스의 제공자다. |
| `src/example/` | interface library 사용 예시 메모/코드다. production 실행 경로에는 포함되지 않는다. |
| `src/how_to_use.txt`, `src/how_to_know_coordinate.txt` | 과거 사용법·좌표계 메모다. 최신 배포 절차는 `DEPLOYMENT_SETUP_GUIDE.md`를 우선한다. |

## 6. 결과와 기존 설계 문서

| 경로 | 기능 |
|---|---|
| `results/cases/*.csv` | case별 원시 prediction/measurement 결과다. 파일명 timestamp가 같은 prediction·spline CSV가 한 쌍이다. |
| `results/single_input/`, `results/reconstruct_spline/` | 단일 입력 및 spline 비교 그래프/summary다. |
| `results/analysis_report.md`, `phase*.png` | 전체 case 분석의 이전 산출물이다. |
| `md_file/FORMATION_MODE_ARCHITECTURE.md` | Formation mode의 설계 구조 설명이다. |
| `md_file/SUBSCRIBER_DATA_FLOW.md` | subscriber부터 RT loop까지 데이터 전달 구조를 설명한다. |
| `md_file/TRAJECTORY_PREDICT_HOW_IT_WORKS.md` | prediction 모델, ZOH 가정, 검증 결과와 한계를 상세히 설명한다. |
| `md_file/migration_guide_v2.md`, `HANDOFF_*.md`, `SESSION_*.md` | 알고리즘 이전 및 세션 인수인계 이력이다. 현재 실행 절차보다 설계 배경 확인에 적합하다. |

## 7. 수정할 때의 영향 범위

| 변경 대상 | 함께 확인할 항목 |
|---|---|
| 기체 수/초기 위치 | `spawn_config.yaml`, `ros_params.yaml`, Agent 포트 수, 기체별 `vehicle_ID`/DDS namespace |
| 실제 기체 성능 파라미터 | 양 패키지의 `airframe_spec.yaml`, `flocking_params.yaml`, PX4/QGC 파라미터 |
| 예측 horizon/rate | `replay_params.yaml`, `TrajectoryLogger.hpp`, 분석 스크립트의 `N_PREDICT`/rate 상수 |
| PX4 버전 | `px4_msgs`, `px4_ros2_cpp`, PX4 펌웨어를 함께 맞추고 message compatibility 검사 |
| CSV 출력 위치 | `replay_params.yaml`, `run_all_cases.sh`, 후처리 명령의 입력 경로 |
