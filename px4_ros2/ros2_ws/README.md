# PX4 ROS 2 workspace — swarm fixed-wing collision avoidance

PX4 ROS 2 기반 VTOL/fixed-wing swarm 편대 비행과 단일 기체 궤적 예측 검증을 위한 워크스페이스다.

## 포함 패키지

- `src/collision_avoidance`: 다기체 좌표 통일, VTOL preflight, formation/flocking guidance, 궤적 예측·spline 재구성 라이브러리
- `src/trajectory_prediction`: 사전 정의 setpoint 재생, SITL 예측 검증, CSV/그래프 분석 harness
- `docker`: Raspberry Pi 5용 `collision_avoidance` 컨테이너 정의
- `md_file`: 배포, 설계, 데이터 흐름 및 알고리즘 인수인계 문서

PX4 외부 의존성은 소스에 중복 저장하지 않는다. 검증된 커밋은 `px4_dependencies.repos`에 고정되어 있다.

## 준비

Ubuntu 22.04와 ROS 2 Humble을 기준으로 한다.

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git python3-colcon-common-extensions python3-vcstool \
  libeigen3-dev libyaml-cpp-dev \
  ros-humble-ros-base ros-humble-eigen3-cmake-module

cd px4_ros2/ros2_ws
vcs import src < px4_dependencies.repos
source /opt/ros/humble/setup.bash
```

PX4 펌웨어와 `px4_msgs`/`px4_ros2_cpp` 메시지 버전은 반드시 호환되어야 한다.

## 빌드

```bash
colcon build --packages-select px4_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

colcon build --packages-select px4_ros2_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

colcon build --packages-select collision_avoidance trajectory_prediction \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

자세한 대상 PC 준비와 실행 방법은 [배포 가이드](md_file/DEPLOYMENT_SETUP_GUIDE.md)를, 소스 파일별 역할은 [파일 기능 참조](md_file/FILE_FUNCTION_REFERENCE.md)를 참고한다.

## 제외 항목

`build/`, `install/`, `log/`, CSV/PNG 실험 결과, Python cache, 편집기 설정, 압축 백업은 Git에 포함하지 않는다. 대상 PC에서 빌드와 실험 결과를 새로 생성한다.
