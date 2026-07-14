# ROS2 Humble + collision_avoidance for Raspberry Pi 5 (aarch64)
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# 빌드 의존성 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    ros-humble-eigen3-cmake-module \
    && rm -rf /var/lib/apt/lists/*

# 워크스페이스 생성
WORKDIR /ros2_ws/src

# px4_msgs 복사 & 빌드 (메시지부터 먼저)
COPY src/px4_msgs ./px4_msgs
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select px4_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release

# px4-ros2-interface-lib 복사 & 빌드
COPY src/px4-ros2-interface-lib /ros2_ws/src/px4-ros2-interface-lib
RUN . /opt/ros/humble/setup.sh && \
    . /ros2_ws/install/setup.sh && \
    colcon build --packages-select px4_ros2_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release

# collision_avoidance 복사 & 빌드
COPY src/collision_avoidance /ros2_ws/src/collision_avoidance
RUN . /opt/ros/humble/setup.sh && \
    . /ros2_ws/install/setup.sh && \
    colcon build --packages-select collision_avoidance --cmake-args -DCMAKE_BUILD_TYPE=Release

# 엔트리포인트
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
