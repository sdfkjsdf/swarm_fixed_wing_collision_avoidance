cmake_minimum_required(VERSION 3.5)
project(collision_avoidance)          # ← 패키지 이름

set(CMAKE_CXX_STANDARD 17)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

include_directories(include)

# ── VTOL Guidance node (px4-ros2-interface-lib 기반) ──
add_executable(vtol_guidance_node
    src/main.cpp
    src/VtolGuidanceMode.cpp
    src/VtolGuidanceExecutor.cpp
)
ament_target_dependencies(vtol_guidance_node px4_msgs)
target_link_libraries(vtol_guidance_node
    Eigen3::Eigen
    rclcpp::rclcpp
    px4_ros2_cpp::px4_ros2_cpp
)

# ── Coordinate Transformer node (기존 유지) ──
add_executable(coordinate_transformer_node
    src/transformer_main.cpp
    src/TransferSameCoordinate.cpp
)
ament_target_dependencies(coordinate_transformer_node rclcpp px4_msgs)

install(TARGETS
    vtol_guidance_node
    coordinate_transformer_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
