# Swarm Fixed-Wing Collision Avoidance

PX4 SITL과 ROS 2를 이용해 VTOL/fixed-wing 군집 비행, 궤적 예측 및 충돌 회피를
연구하기 위한 프로젝트다.

현재 ROS 2 개발 환경은 Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11,
PX4 SITL을 기준으로 한다.

## 연구 목표

1. Leader-follower 및 flocking 기반 편대 비행 구현
2. Closed-form Control Barrier Function(CBF)을 이용한 충돌 회피 가능성 검증
3. 지오펜싱 상황에서 고도 분리와 CBF 기반 안전 제약 검증
4. PX4 SITL에서 다기체 알고리즘의 실시간 동작 및 재현성 평가

## 주요 디렉터리

| 경로 | 설명 |
| --- | --- |
| `px4_ros2/ros2_ws/src/collision_avoidance` | 실기체 탑재 대상: 편대 guidance, 궤적 예측·재구성, PX4 mode |
| `px4_ros2/ros2_ws/src/testing_module/trajectory_prediction_hils` | 단일 VTOL SITL/HILS replay 및 CSV 검증 harness |
| `px4_ros2/ros2_ws/src/testing_module/formation_hils` | 다기체 formation SITL/HILS 실행·모니터링 |
| `px4_ros2/ros2_ws/src/testing_module/analysis_tools` | 시험 CSV 분석과 비교 도구 |
| `px4_ros2/ros2_ws/px4_dependencies.repos` | 검증된 ROS 2 외부 의존성 버전 |
| `px4_ros2/ros2_ws/md_file` | 배포, 구조, 데이터 흐름 및 알고리즘 문서 |
| `px4_ros2/RPI_5_docker_images` | Raspberry Pi 5 배포용 Docker 관련 파일 |
| `Theoretical_background` | 제어 및 충돌 회피 이론 자료 |
| `cpp`, `simulinke_prototype`, `symbolic_derivation` | 초기 구현, 프로토타입 및 수식 유도 |

ROS 2 워크스페이스의 자세한 설명은
[`px4_ros2/ros2_ws/README.md`](px4_ros2/ros2_ws/README.md)를 참고한다.

## 검증된 개발 환경

| 구성 요소 | 버전 |
| --- | --- |
| OS | Ubuntu 22.04 Jammy |
| ROS | ROS 2 Humble |
| Simulator | Gazebo Classic 11.10 |
| PX4-Autopilot | `ffd670b54cf33fe2eb3ed3f97adc40790ea05235` |
| Micro XRCE-DDS Agent | `v2.4.2` |
| `px4_msgs` | `f7741616e14c8330b54acdaf8daf382750896081` |
| `px4_ros_com` | `86e9aeb20e55a4673fa8a9f1c29ea06a6c5ad1af` |
| `px4-ros2-interface-lib` | `365dd8807869fd81813de0415ec99e85ea021d59` |

PX4와 `px4_msgs`는 메시지 정의가 일치해야 한다. 위 `px4_msgs` 커밋은
trajectory-cone 입력에 필요한 `EstimatorTrajectoryBelief` 메시지를 포함한
프로젝트 fork이므로 공식 upstream 커밋으로 임의 교체하지 않는다.

## 권장 로컬 구조

소스 저장소와 생성되는 빌드 결과를 분리하는 구성을 권장한다.

```text
~/workspace/swarm-fixed-wing/
├── source/
│   └── swarm_fixed_wing_collision_avoidance/
├── firmware/
│   └── PX4-Autopilot/
├── ros2_ws/
│   ├── src/
│   ├── build/
│   ├── install/
│   └── log/
└── tools/
```

이 구조에서는 프로젝트 저장소에 소스 변경만 남고, `colcon` 산출물은 외부
워크스페이스에 생성된다.

## 소스 준비

```bash
mkdir -p ~/workspace/swarm-fixed-wing/{source,firmware,ros2_ws/src,tools}
cd ~/workspace/swarm-fixed-wing

git clone https://github.com/sdfkjsdf/swarm_fixed_wing_collision_avoidance.git \
  source/swarm_fixed_wing_collision_avoidance

git clone https://github.com/PX4/PX4-Autopilot.git firmware/PX4-Autopilot
git -C firmware/PX4-Autopilot checkout \
  ffd670b54cf33fe2eb3ed3f97adc40790ea05235
git -C firmware/PX4-Autopilot submodule update --init --recursive

ln -s \
  ~/workspace/swarm-fixed-wing/source/swarm_fixed_wing_collision_avoidance/px4_ros2/ros2_ws/src/collision_avoidance \
  ~/workspace/swarm-fixed-wing/ros2_ws/src/collision_avoidance
ln -s \
  ~/workspace/swarm-fixed-wing/source/swarm_fixed_wing_collision_avoidance/px4_ros2/ros2_ws/src/testing_module \
  ~/workspace/swarm-fixed-wing/ros2_ws/src/testing_module

cd ~/workspace/swarm-fixed-wing/ros2_ws
vcs import src < \
  ../source/swarm_fixed_wing_collision_avoidance/px4_ros2/ros2_ws/px4_dependencies.repos
```

## ROS 2 빌드

ROS 2 Humble과 필수 빌드 도구가 설치된 터미널에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
cd ~/workspace/swarm-fixed-wing/ros2_ws

colcon build --packages-select px4_msgs \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

colcon build --packages-select px4_ros2_cpp \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

colcon build --packages-select collision_avoidance trajectory_prediction_hils \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

테스트:

```bash
colcon test --packages-select collision_avoidance
colcon test-result --verbose
```

현재 기준 테스트 결과는 `13 tests, 0 errors, 0 failures`다.

## PX4와 ROS 2 통신

ROS 2 Humble은 Micro XRCE-DDS Agent `v2.4.2`를 사용한다. PX4 SITL의 기본
UDP 포트는 `8888`이다.

```bash
MicroXRCEAgent udp4 -p 8888
```

통신 확인:

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/swarm-fixed-wing/ros2_ws/install/setup.bash

ros2 topic list | grep '^/px4_0/'
ros2 topic echo --once \
  /px4_0/fmu/out/vehicle_status_v3 \
  px4_msgs/msg/VehicleStatus \
  --qos-reliability best_effort
```

## 단일 VTOL 실행

`PX4_DIR`, `ROS2_WS`, `PX4_PYTHON`을 현재 로컬 구조에 맞게 설정한 후 실행한다.

```bash
export PX4_DIR=~/workspace/swarm-fixed-wing/firmware/PX4-Autopilot
export ROS2_WS=~/workspace/swarm-fixed-wing/ros2_ws
export PX4_PYTHON=~/workspace/swarm-fixed-wing/.envs/px4/bin/python

source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"

"${ROS2_WS}/src/testing_module/trajectory_prediction_hils/scripts/launch_1vtol_replay.sh" --no-node
```

이 실행 경로는 `standard_vtol` 모델 스폰, PX4–Gazebo 연결 및 `/px4_0` DDS
토픽 생성을 확인한다.

## 문서

- [배포 가이드](px4_ros2/ros2_ws/md_file/DEPLOYMENT_SETUP_GUIDE.md)
- [프로젝트 파일 구조](px4_ros2/ros2_ws/md_file/PROJECT_FILE_DIAGRAM.md)
- [Subscriber 데이터 흐름](px4_ros2/ros2_ws/md_file/SUBSCRIBER_DATA_FLOW.md)
- [파일 기능 참조](px4_ros2/ros2_ws/md_file/FILE_FUNCTION_REFERENCE.md)

## Git 관리 원칙

- `build/`, `install/`, `log/` 및 실험 결과 CSV/PNG는 커밋하지 않는다.
- PX4와 ROS 외부 의존성은 저장소에 복사하지 않고 커밋 해시로 고정한다.
- 기능 변경과 환경·문서 변경은 가능한 한 별도 커밋으로 관리한다.
- 실험 데이터가 필요하면 Git 대신 별도의 데이터 저장 위치를 사용한다.
