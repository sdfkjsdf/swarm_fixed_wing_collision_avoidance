cmake_minimum_required(VERSION 3.8)
project(trajectory_prediction_hils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)             # ★ key_samples publisher 용 (Float64MultiArray)
find_package(collision_avoidance REQUIRED)

add_executable(trajectory_replay_node
    src/nodes/trajectory_replay_main.cpp
    src/modes/VtolPreflightMode.cpp
    src/modes/TrajectoryReplayExecutor.cpp
    src/replay/TrajectoryReplayMode.cpp
    src/replay/SetpointSequencer.cpp
    src/logging/TrajectoryLogger.cpp
    src/estimation/BeliefConePublisher.cpp
)
target_include_directories(trajectory_replay_node PRIVATE include)
ament_target_dependencies(trajectory_replay_node
    px4_msgs
    rclcpp
    ament_index_cpp
    std_msgs                   # ★ key_samples publisher (Float64MultiArray)
    collision_avoidance
)
target_link_libraries(trajectory_replay_node
    rclcpp::rclcpp
    px4_ros2_cpp::px4_ros2_cpp
    yaml-cpp
    collision_avoidance::trajectory_prediction_core
    collision_avoidance::trajectory_reconstruction
)

install(TARGETS
    trajectory_replay_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
    scripts/record_trajectory_cone_bag.sh
    analysis/analyze_trajectory_cone_bag.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
