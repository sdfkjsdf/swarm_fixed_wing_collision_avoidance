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
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/TrajectoryPredictionDebug.msg"
)

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
    collision_avoidance::ground_to_eas_adapter
    collision_avoidance::trajectory_prediction_core
    collision_avoidance::trajectory_reconstruction
)

add_executable(trajectory_prediction_sils_test_node
    src/nodes/trajectory_prediction_sils_test_main.cpp
    src/modes/VtolPreflightMode.cpp
    src/testing/PropagationTestMode.cpp
    src/testing/PropagationTestExecutor.cpp
    src/estimation/BeliefConePublisher.cpp
)
target_include_directories(trajectory_prediction_sils_test_node PRIVATE include)
ament_target_dependencies(trajectory_prediction_sils_test_node
    px4_msgs
    rclcpp
    collision_avoidance
)
target_link_libraries(trajectory_prediction_sils_test_node
    rclcpp::rclcpp
    px4_ros2_cpp::px4_ros2_cpp
    collision_avoidance::ground_to_eas_adapter
    collision_avoidance::trajectory_prediction_core
)
rosidl_get_typesupport_target(
    trajectory_prediction_hils_cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_prediction_sils_test_node
    "${trajectory_prediction_hils_cpp_typesupport_target}")

install(TARGETS
    trajectory_replay_node
    trajectory_prediction_sils_test_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
    scripts/launch_1vtol_replay.sh
    scripts/record_trajectory_cone_bag.sh
    scripts/process_cone_batch.sh
    scripts/run_cone_scenarios.sh
    scripts/run_propagation_scenarios.sh
    scripts/launch_propagation_test.sh
    analysis/analyze_trajectory_cone_bag.py
    analysis/expand_cone_scenarios.py
    analysis/expand_propagation_scenarios.py
    analysis/plot_trajectory_cone_results.py
    analysis/summarize_cone_batch.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
