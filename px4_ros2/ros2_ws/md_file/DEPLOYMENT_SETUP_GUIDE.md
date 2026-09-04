# 배포 대상 PC 설정 및 실행 가이드

> 대상: ROS 2 Humble 기반 Ubuntu PC 또는 Raspberry Pi 5(aarch64).  
> 범위: `collision_avoidance`(다기체 편대 비행), `trajectory_prediction_hils`(단일 VTOL 궤적 재생·예측 검증), PX4 SITL.
> 기준 경로는 `/home/<사용자>/ros2_ws`이며, 문서 안의 `<WS>`는 그 경로를 뜻한다.

## 1. 배포 전에 함께 전달할 항목

다음은 **반드시 같은 버전으로** 전달한다. `build/`, `install/`, `log/`는 대상 PC에서 다시 만들므로 전달하지 않는다.

```
ros2_ws/
├── src/
│   ├── collision_avoidance/          # 자체 ROS 2 패키지
│   ├── trajectory_prediction/        # 자체 ROS 2 패키지/검증 도구
│   ├── px4_msgs/                     # PX4 메시지 정의 (PX4 버전과 반드시 일치)
│   ├── px4_ros_com/                  # PX4 ROS 2 예제/보조 패키지
│   └── px4-ros2-interface-lib/       # px4_ros2_cpp 라이브러리
├── docker/                           # 컨테이너 배포를 선택할 때 사용
├── md_file/                          # 인수인계 문서
└── results/                          # 선택: 기존 실험 결과와 그래프
```

추가로 PX4 SITL까지 실행할 PC에는 **이 워크스페이스 밖의** `PX4-Autopilot` 소스와 빌드 결과도 필요하다. 실기체 동반컴퓨터만 운용할 경우 Gazebo/PX4 SITL은 필요 없지만, PX4 펌웨어·`px4_msgs`·`px4_ros2_cpp`의 메시지 버전은 맞아야 한다.

## 2. 호스트 PC 사전 조건

권장 조합은 Ubuntu 22.04 + ROS 2 Humble + C++17이다. 먼저 ROS 2 Humble의 기본 설치 및 `colcon`을 준비한다.

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git python3-pip python3-colcon-common-extensions python3-vcstool \
  libeigen3-dev libyaml-cpp-dev \
  ros-humble-ros-base ros-humble-eigen3-cmake-module
```

분석 스크립트까지 사용할 경우 Python 패키지도 필요하다.

```bash
python3 -m pip install --user numpy pandas matplotlib pyyaml scipy
```

PX4 SITL을 사용할 경우에는 별도로 다음이 필요하다.

- `PX4-Autopilot`을 빌드한 경로. 기본값은 `~/PX4-Autopilot`.
- Gazebo Classic 및 PX4의 `gazebo-classic_standard_vtol` 모델.
- `MicroXRCEAgent` 실행 파일.
- GUI를 쓸 경우 X11 표시 권한.

PX4, `px4_msgs`, `px4-ros2-interface-lib`는 같은 release 계열을 써야 한다. 이
프로젝트의 `px4_dependencies.repos`는 `EstimatorTrajectoryBelief`가 추가된
프로젝트 `px4_msgs` fork를 고정한다. 공식 upstream 버전으로 대체하면
`collision_avoidance`가 빌드되지 않는다. 메시지 불일치가 있으면 외부 모드
등록 시 실패할 수 있다. 배포 전 아래 검사를 권장한다.

```bash
cd <WS>/src/px4-ros2-interface-lib
./scripts/check-message-compatibility.py -v <WS>/src/px4_msgs ~/PX4-Autopilot
```

## 3. 경로 의존성 정리 (필수 확인)

실행 환경이 다르면 아래 환경변수와 경로를 지정한다.

| 위치 | 기본 경로/값 | 대상 PC에서 할 일 |
|---|---|---|
| `launch_5vtol.sh` | `PX4_DIR`, `ROS2_WS`, `PX4_PYTHON` | 로컬 배치나 Python 환경이 다르면 환경변수로 지정 |
| `launch_1vtol_replay.sh` | `PX4_DIR`, `ROS2_WS`, `PX4_PYTHON`, `ANALYSIS_PYTHON` | 로컬 배치나 Python 환경이 다르면 환경변수로 지정 |
| `run_all_cases.sh` | `ROS2_WS`, `HILS_SRC`, `ANALYSIS_DIR` | 별도 소스/빌드 워크스페이스 구성이면 환경변수로 지정 |
| `compare_b_h_grid.sh` | 시스템 `python3` | 다른 환경은 `ANALYSIS_PYTHON`으로 지정 |
| Dockerfile | `src/px4-ros2-interface-lib` 포함을 전제 | 전달본에 해당 폴더가 있는지 확인 |

`testing_module/formation_hils/config/spawn_config.yaml`의 기체 수·초기 위치와 `collision_avoidance/config/ros_params.yaml`의 `total_agent_num`, `spawn_offset_*`는 반드시 동일해야 한다. `launch_5vtol.sh`가 시작 전에 이 일치 여부를 검사한다.

## 4. 최초 빌드

```bash
cd <WS>
source /opt/ros/humble/setup.bash

# 의존성 순서: 메시지 → PX4 interface library → 자체 패키지
colcon build --packages-select px4_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

colcon build --packages-select px4_ros2_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

colcon build --packages-select collision_avoidance trajectory_prediction_hils \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

빌드 확인:

```bash
ros2 pkg list | rg '^(collision_avoidance|trajectory_prediction_hils|px4_msgs|px4_ros2_cpp)$'
colcon test --packages-select collision_avoidance
colcon test-result --verbose
```

빌드가 꼬였을 때에는 소스는 지우지 말고 생성물만 제거 후 다시 빌드한다.

```bash
cd <WS>
rm -rf build install log
```

## 5. 운용 방식별 실행

### A. 다기체 SITL 편대 비행

터미널 1에서 각 PX4 인스턴스의 DDS Agent를 시작한다. 기본 5대는 포트 8888~8892를 쓴다.

```bash
MicroXRCEAgent udp4 -p 8888 &
MicroXRCEAgent udp4 -p 8889 &
MicroXRCEAgent udp4 -p 8890 &
MicroXRCEAgent udp4 -p 8891 &
MicroXRCEAgent udp4 -p 8892 &
```

터미널 2에서 SITL 기체를 올린다.

```bash
cd <WS>/src/testing_module/formation_hils/scripts
./launch_5vtol.sh noshow      # headless
# 또는 ./launch_5vtol.sh show # Gazebo GUI
```

터미널 3에서 좌표 통일 노드를 실행한다.

```bash
source <WS>/install/setup.bash
ros2 run collision_avoidance coordinate_transformer_node \
  --ros-args --params-file <WS>/src/collision_avoidance/config/ros_params.yaml
```

각 기체(또는 각 기체용 컨테이너)에서 가이던스 노드를 하나씩 실행한다. `vehicle_ID`는 0부터 `total_agent_num - 1`까지 달라야 한다.

```bash
source <WS>/install/setup.bash
ros2 run collision_avoidance vtol_guidance_node --ros-args \
  --params-file <WS>/src/collision_avoidance/config/flocking_params.yaml \
  -p vehicle_ID:=0 -p total_agent_num:=5 \
  -r __node:=vtol_guidance_0
```

상태 확인:

```bash
cd <WS>/src/testing_module/formation_hils/scripts
./monitor_swarm.sh
ros2 topic list | rg 'px4_|common|swarm_status'
```

### B. 단일 VTOL 궤적 재생 및 예측 검증

이 스크립트는 Agent, Gazebo, PX4 SITL, `trajectory_replay_node`를 순서대로 기동하고 종료 시 정리한다.

```bash
cd <WS>
PX4_DIR=~/PX4-Autopilot ROS2_WS=<WS> \
  src/testing_module/trajectory_prediction_hils/scripts/launch_1vtol_replay.sh
```

SITL만 먼저 올리고 노드는 별도 터미널에서 실행하려면 `--no-node`를 사용한다.

```bash
PX4_DIR=~/PX4-Autopilot ROS2_WS=<WS> \
  <WS>/src/testing_module/trajectory_prediction_hils/scripts/launch_1vtol_replay.sh --no-node
```

기본 시퀀스는 `testing_module/trajectory_prediction_hils/config/setpoint_sequence.yaml`이다. 특정 시험 시퀀스로 바꾸려면:

```bash
SEQUENCE_FILE=<WS>/src/testing_module/trajectory_prediction_hils/config/cases/R15P.yaml \
  PX4_DIR=~/PX4-Autopilot ROS2_WS=<WS> \
  <WS>/src/testing_module/trajectory_prediction_hils/scripts/launch_1vtol_replay.sh
```

결과 CSV는 기본적으로 `/tmp/trajectory_<timestamp>.csv` 및 spline CSV로 생성된다. `results/cases/`로 수집하여 활성 R-series를 일괄 실행하려면:

```bash
cd <WS>/src/testing_module/trajectory_prediction_hils/scripts
ROS2_WS=<WS> ./run_all_cases.sh --phase R
```

## 6. Docker 운용 (선택)

Dockerfile은 ROS 2 Humble 기반 aarch64/Raspberry Pi 5 컨테이너를 의도한다. 이미지에는 `px4_msgs`, `px4_ros2_cpp`, `collision_avoidance`가 빌드되며 `trajectory_prediction_hils`은 포함하지 않는다.

```bash
cd <WS>
docker build -t collision-avoidance:latest -f docker/Dockerfile .
docker run -it --rm --network host \
  -e ROS_DOMAIN_ID=0 -e VEHICLE_ID=0 -e TOTAL_AGENTS=5 \
  collision-avoidance:latest
```

컨테이너 내부에서 출력된 명령을 실행하거나 다음 명령을 사용한다.

```bash
ros2 run collision_avoidance vtol_guidance_node --ros-args \
  --params-file /ros2_ws/install/collision_avoidance/share/collision_avoidance/config/flocking_params.yaml \
  -p vehicle_ID:=${VEHICLE_ID} -p total_agent_num:=${TOTAL_AGENTS} \
  -r __node:=vtol_guidance_${VEHICLE_ID}
```

Gazebo GUI를 컨테이너에서 띄우는 경우에만 호스트에서 `xhost +local:docker`를 먼저 실행하고 X11 소켓/`DISPLAY`를 컨테이너에 전달한다. 실기체 가이던스 노드만 돌릴 때는 GUI 설정이 필요 없다.

## 7. 배포 전 체크리스트

- [ ] `px4_msgs`, PX4 펌웨어, `px4_ros2_cpp`의 버전/메시지 호환성을 확인했다.
- [ ] `<WS>`와 `PX4_DIR` 경로를 대상 PC에 맞게 설정했다.
- [ ] `formation_hils/config/spawn_config.yaml`과 `collision_avoidance/config/ros_params.yaml`의 기체 수·오프셋을 맞췄다.
- [ ] `airframe_spec.yaml`/`flocking_params.yaml`의 속도, climb/sink, roll 제한을 실제 기체 PX4 파라미터와 맞췄다.
- [ ] 실기체 운용 전에는 SITL에서 토픽, mode 등록, 좌표계(NED/ENU)와 failsafe를 확인했다.
- [ ] 각 기체의 `vehicle_ID`, node name, PX4 DDS namespace, Agent UDP port가 중복되지 않는다.

## 8. 빠른 장애 대응

| 증상 | 우선 확인할 내용 |
|---|---|
| `find_package(px4_ros2_cpp)` 실패 | `px4_ros2_cpp`를 먼저 빌드했고 `source install/setup.bash` 했는지 확인 |
| PX4 토픽이 없음 | 해당 UDP 포트의 `MicroXRCEAgent`가 실행 중인지, `PX4_UXRCE_DDS_NS`가 맞는지 확인 |
| 외부 mode 등록 실패 | PX4/`px4_msgs`/interface lib 메시지 버전 불일치 여부 확인 |
| 편대가 서로 다른 위치로 보임 | spawn/ROS offset YAML 불일치 및 NED↔ENU 부호 확인 |
| Gazebo/PX4 재실행 실패 | 남은 `px4`, `gzserver`, `gzclient`, `MicroXRCEAgent` 프로세스를 종료한 뒤 재시작 |
| 분석 스크립트 ImportError | `numpy pandas matplotlib pyyaml scipy`가 설치된 Python 실행 파일을 사용 |
