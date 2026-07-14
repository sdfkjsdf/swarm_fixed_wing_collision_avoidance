cmake_minimum_required(VERSION 3.5)
project(trajectory_prediction)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)             # ★ key_samples publisher 용 (Float64MultiArray)
# ★ 2026-05-13 마이그레이션: TrajectoryPredict 알고리즘이 collision_avoidance 의
#   collision_estimation library 로 이동. 본 패키지는 *튜닝 test harness* 로서
#   해당 library 를 link 하여 동일 알고리즘 코드를 재사용.
find_package(collision_avoidance REQUIRED)

include_directories(include)

# ★ 2026-05-13: src/TrajectoryPredict.cpp 는 *제거됨*. 본 패키지는 더 이상 자체
#   알고리즘을 컴파일하지 않고, collision_avoidance::collision_estimation library
#   를 link 하여 사용.
add_executable(trajectory_replay_node
    src/main.cpp
    src/VtolPreflightMode.cpp
    src/VtolGuidanceExecutor.cpp
    src/TrajectoryReplayMode.cpp
    src/SetpointSequencer.cpp
    src/TrajectoryLogger.cpp
)
ament_target_dependencies(trajectory_replay_node
    px4_msgs
    rclcpp
    std_msgs                   # ★ key_samples publisher (Float64MultiArray)
    collision_avoidance        # ★ collision_estimation library export 사용
)
target_link_libraries(trajectory_replay_node
    rclcpp::rclcpp
    px4_ros2_cpp::px4_ros2_cpp
    yaml-cpp
    collision_avoidance::collision_estimation   # ★ 알고리즘 library link
)

install(TARGETS
    trajectory_replay_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

# ──────────────────────────────────────────────────────────────────
# Unit tests (gtest)  —  md_file/TASK_trajectory_predictor.md §8
# ──────────────────────────────────────────────────────────────────
# ★ 2026-05-13 마이그레이션: src/TrajectoryPredict.cpp 직접 컴파일 제거,
#   대신 collision_avoidance::collision_estimation library link 로 동일 알고리즘
#   호출. 12 tests 가 *같은 알고리즘 단일 binary* 를 검증하게 됨 — drift 위험 0.
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_trajectory_predict
        test/test_trajectory_predict.cpp
    )
    ament_target_dependencies(test_trajectory_predict
        collision_avoidance
    )
    target_link_libraries(test_trajectory_predict
        collision_avoidance::collision_estimation
    )
    target_include_directories(test_trajectory_predict PRIVATE include)
    set_target_properties(test_trajectory_predict PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
    )
endif()

ament_package()
