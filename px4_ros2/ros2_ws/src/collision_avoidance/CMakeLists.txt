cmake_minimum_required(VERSION 3.5)
project(collision_avoidance)          # ← 패키지 이름

set(CMAKE_CXX_STANDARD 17)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

include_directories(include)

# ── collision_estimation static library (2026-05-13 마이그레이션) ──
# trajectory_prediction 패키지의 *알고리즘 부분* (TrajectoryPredict, PredictTypes)
# 만 본 패키지로 이전. trajectory_prediction 패키지는 *test harness 로 retain* 하고
# 본 라이브러리를 link 하여 *동일 알고리즘 코드 재사용*. 즉 본 라이브러리가
# *유일한 알고리즘 source of truth*.
#
# 의도된 사용처:
#   - trajectory_prediction 패키지의 trajectory_replay_node (튜닝 SITL)
#   - 미래: collision_avoidance 의 vtol_guidance_node (실시간 충돌 검사)
#
# 향후 확장: collision_estimation 안에 새 기능 (CollisionChecker, ObstacleDetector 등)
# 추가 시 본 add_library 의 sources 에 한 줄씩 append. 단일 namespace
# collision_avoidance::collision_estimation 으로 모든 기능 통합.
add_library(collision_estimation STATIC
    src/collision_estimation/trajectory_prediction/TrajectoryPredict.cpp
    src/collision_estimation/reconstruct_trajectory/ReconstructTrajectory.cpp
)
target_include_directories(collision_estimation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# ★ 2026-05-13: ReconstructTrajectory.hpp 가 Eigen::Matrix4f 사용 (inverse_A 정의).
# Eigen3 는 header-only — interface target Eigen3::Eigen 으로 *include path 만* 전파.
target_link_libraries(collision_estimation PUBLIC Eigen3::Eigen)

# Export 하여 trajectory_prediction 패키지 (및 미래의 다른 패키지) 가 find_package 가능
ament_export_targets(collision_estimation_export HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)   # ★ downstream 패키지가 Eigen 자동 가져옴
install(TARGETS collision_estimation
    EXPORT collision_estimation_export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(DIRECTORY include/
    DESTINATION include
)

# ── VTOL Guidance node (px4-ros2-interface-lib 기반) ──
# 2026-05-13 구조 정리: src/ 내부를 modes / guidance / utils 로 분리.
#   - src/main.cpp        : 진입점 (루트 유지)
#   - src/modes/          : ModeBase 구현 (Preflight, Formation, GuidanceExecutor)
#   - src/guidance/       : 가이던스 알고리즘 (Flocking, 향후 Avoidance 등)
#   - src/utils/          : 좌표변환 등 유틸 (별도 노드용)
# 헤더는 include/collision_avoidance/{modes,guidance,utils}/ 로 동일하게 분리.
# StateType/spsc_queue/AirframeLimits 는 *공통 인프라* 라 include 의 *루트 유지*.
add_executable(vtol_guidance_node
    src/main.cpp
    src/modes/VtolPreflightMode.cpp
    src/modes/FormationMode.cpp
    src/modes/VtolGuidanceExecutor.cpp
    src/guidance/FlockingGuidance.cpp
)
ament_target_dependencies(vtol_guidance_node px4_msgs std_msgs)
target_link_libraries(vtol_guidance_node
    Eigen3::Eigen
    rclcpp::rclcpp
    px4_ros2_cpp::px4_ros2_cpp
)

# ── Coordinate Transformer node (기존 유지) ──
add_executable(coordinate_transformer_node
    src/transformer_main.cpp
    src/utils/TransferSameCoordinate.cpp
)
ament_target_dependencies(coordinate_transformer_node rclcpp px4_msgs)

install(TARGETS
    vtol_guidance_node
    coordinate_transformer_node
    DESTINATION lib/${PROJECT_NAME}
)

# yaml config 도 install 하면 ros2 launch 등에서 참조 가능
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
