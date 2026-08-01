cmake_minimum_required(VERSION 3.8)
project(collision_avoidance)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

# Pure C++ trajectory predictor. This is the production source of truth used by
# both the onboard runtime and the HILS verification packages.
add_library(trajectory_prediction_core STATIC
    src/estimation/trajectory_prediction/TrajectoryPredict.cpp
)
add_library(collision_avoidance::trajectory_prediction_core ALIAS
    trajectory_prediction_core
)
target_include_directories(trajectory_prediction_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Trajectory reconstruction is kept as a separate target so prediction can be
# tested and deployed without pulling reconstruction code into every consumer.
add_library(trajectory_reconstruction STATIC
    src/estimation/reconstruction/ReconstructTrajectory.cpp
)
add_library(collision_avoidance::trajectory_reconstruction ALIAS
    trajectory_reconstruction
)
target_include_directories(trajectory_reconstruction PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(trajectory_reconstruction PUBLIC
    trajectory_prediction_core
    Eigen3::Eigen
)

install(TARGETS
    trajectory_prediction_core
    trajectory_reconstruction
    EXPORT collision_avoidance_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(EXPORT collision_avoidance_targets
    NAMESPACE collision_avoidance::
    DESTINATION share/${PROJECT_NAME}/cmake
)
install(DIRECTORY include/
    DESTINATION include
)

ament_export_targets(collision_avoidance_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)

add_executable(vtol_guidance_node
    src/nodes/vtol_guidance_main.cpp
    src/modes/VtolPreflightMode.cpp
    src/modes/FormationMode.cpp
    src/modes/VtolGuidanceExecutor.cpp
    src/guidance/FlockingGuidance.cpp
)
target_include_directories(vtol_guidance_node PRIVATE include)
ament_target_dependencies(vtol_guidance_node px4_msgs std_msgs)
target_link_libraries(vtol_guidance_node
    Eigen3::Eigen
    rclcpp::rclcpp
    px4_ros2_cpp::px4_ros2_cpp
)

add_executable(coordinate_transformer_node
    src/nodes/coordinate_transformer_main.cpp
    src/coordinate/TransferSameCoordinate.cpp
)
target_include_directories(coordinate_transformer_node PRIVATE include)
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

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_trajectory_predict
        test/test_trajectory_predict.cpp
    )
    target_link_libraries(test_trajectory_predict
        trajectory_prediction_core
    )
    target_include_directories(test_trajectory_predict PRIVATE include)

    ament_add_gtest(test_global_types
        test/test_global_types.cpp
    )
    target_include_directories(test_global_types PRIVATE include)
endif()

ament_package()
