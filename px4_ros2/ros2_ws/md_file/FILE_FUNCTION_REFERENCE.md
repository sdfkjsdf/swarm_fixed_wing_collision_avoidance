# 현재 파일 기능 참조

이 문서는 현재 소스 트리를 기준으로 한다. 과거 설계·세션 문서는 변경 당시의
경로를 보존하므로 현재 실행 경로는 이 문서와 루트 README를 우선한다.

## Production: `collision_avoidance`

### 궤적 예측

| 파일 | 기능 |
| --- | --- |
| `include/collision_avoidance/estimation/trajectory_prediction/PredictTypes.hpp` | 예측 상태·입력·파라미터와 key sample POD |
| `include/collision_avoidance/estimation/trajectory_prediction/TrajectoryPredict.hpp` | RK4 step, 고정 horizon 예측, key sample 추출 API |
| `src/estimation/trajectory_prediction/TrajectoryPredict.cpp` | ODE, saturation, state safety, RK4 구현 |
| `test/test_trajectory_predict.cpp` | 정확도, stateless, heap-free, WCET와 jitter 검증 |

### 궤적 재구성

| 파일 | 기능 |
| --- | --- |
| `include/collision_avoidance/estimation/reconstruction/ReconstructTrajectory.hpp` | key sample 검증과 spline 재구성 API |
| `src/estimation/reconstruction/ReconstructTrajectory.cpp` | clamped cubic spline 계수와 position/velocity 복원 |

### Guidance와 runtime

| 파일 | 기능 |
| --- | --- |
| `include/.../guidance/FlockingGuidance.hpp`, `src/guidance/FlockingGuidance.cpp` | alignment, cohesion, separation과 fixed-wing setpoint 계산 |
| `include/.../guidance/AirframeLimits.hpp` | airspeed, climb/sink, roll 등 기체 제한 |
| `include/.../common/StateType.hpp` | 다기체 상태와 main↔RT 전달 자료형 |
| `include/.../common/SpscQueue.hpp` | lock-free SPSC queue |
| `include/.../modes/VtolPreflightMode.hpp`, `src/modes/VtolPreflightMode.cpp` | 이륙·VTOL 전환·cruise 안정화 |
| `include/.../modes/FormationMode.hpp`, `src/modes/FormationMode.cpp` | 편대 mode와 RT guidance loop |
| `include/.../modes/VtolGuidanceExecutor.hpp`, `src/modes/VtolGuidanceExecutor.cpp` | preflight→formation state machine |
| `src/nodes/vtol_guidance_main.cpp` | production guidance node 진입점 |

### 좌표 변환

| 파일 | 기능 |
| --- | --- |
| `include/.../coordinate/TransferSameCoordinate.hpp` | 공통 원점 변환 ROS node API |
| `src/coordinate/TransferSameCoordinate.cpp` | spawn offset 기반 odometry 변환 |
| `src/nodes/coordinate_transformer_main.cpp` | 좌표 변환 node 진입점 |

### Production 설정 도구

| 파일 | 기능 |
| --- | --- |
| `tools/convert_px4_params.py` | QGC PX4 parameter export를 production airframe YAML로 변환 |

## Verification: `testing_module/trajectory_prediction_hils`

이 패키지는 production 예측 라이브러리를 링크하며 자체 예측 알고리즘을 갖지 않는다.

| 파일 | 기능 |
| --- | --- |
| `src/nodes/trajectory_replay_main.cpp` | replay mode, logger, predictor와 timer 조립 |
| `include/.../replay/SetpointSequencer.hpp`, `src/replay/SetpointSequencer.cpp` | YAML 시험 입력 조회 |
| `include/.../replay/TrajectoryReplayMode.hpp`, `src/replay/TrajectoryReplayMode.cpp` | setpoint replay mode |
| `include/.../logging/TrajectoryLogger.hpp`, `src/logging/TrajectoryLogger.cpp` | PX4 측정·예측·spline CSV 기록 |
| `include/.../modes/VtolPreflightMode.hpp`, `src/modes/VtolPreflightMode.cpp` | 단일 시험 기체 preflight |
| `include/.../modes/TrajectoryReplayExecutor.hpp`, `src/modes/TrajectoryReplayExecutor.cpp` | preflight→replay state machine |
| `config/cases/*.yaml` | 채널별 시험 자극 |
| `scripts/launch_1vtol_replay.sh` | 단일 SITL 실행 |
| `scripts/run_all_cases.sh` | case 반복 실행과 결과 수집 |

## Formation verification: `testing_module/formation_hils`

| 파일 | 기능 |
| --- | --- |
| `config/spawn_config.yaml` | Gazebo 기체 수와 초기 위치 |
| `scripts/launch_5vtol.sh` | 다기체 Gazebo Classic/PX4 실행과 좌표 변환 node 기동 |
| `scripts/monitor_swarm.sh` | 기체별 swarm 상태 토픽 모니터링 |

## Offline analysis: `testing_module/analysis_tools`

| 파일 | 기능 |
| --- | --- |
| `analyze_cases.py` | case matrix 기반 전체 결과 분석 |
| `chunk_analysis.py` | rolling prediction을 horizon chunk로 비교 |
| `compare_single_input.py` | 단일 입력 prediction/PX4 비교 |
| `compare_spline_reconstruct.py` | RK4와 spline reconstruction 비교 |
| `compare_b_h_grid.sh` | 여러 고도 gain 결과 일괄 비교 |

## 검증 명령

```bash
colcon build --packages-select collision_avoidance trajectory_prediction_hils \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

colcon test --packages-select collision_avoidance
colcon test-result --verbose --test-result-base build/collision_avoidance
```
