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
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/TrajectoryCone.msg"
    "msg/TrajectoryIntent.msg"
)
rosidl_get_typesupport_target(collision_avoidance_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(common_coordinate_transform STATIC
    src/coordinate/CommonNedTransform.cpp
)
add_library(collision_avoidance::common_coordinate_transform ALIAS
    common_coordinate_transform
)
target_include_directories(common_coordinate_transform PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Existing ground-speed/course/wind conversion shared by PX4-facing modes.
add_library(ground_to_eas_adapter STATIC
    src/control/GroundToEasAdapter.cpp
)
add_library(collision_avoidance::ground_to_eas_adapter ALIAS
    ground_to_eas_adapter
)
target_include_directories(ground_to_eas_adapter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Pure C++ trajectory predictor. This is the production source of truth used by
# both the onboard runtime and the HILS verification packages.
add_library(trajectory_prediction_core STATIC
    src/estimation/trajectory_prediction/TrajectoryPredict.cpp
    src/estimation/trajectory_prediction/TrajectoryUncertainty.cpp
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

# ROS-independent A-to-B trajectory-intent pipeline. Transport wrappers may
# copy/serialize the fixed packet without duplicating prediction algorithms.
add_library(trajectory_intent STATIC
    src/estimation/trajectory_prediction/TrajectoryIntent.cpp
)
add_library(collision_avoidance::trajectory_intent ALIAS
    trajectory_intent
)
target_include_directories(trajectory_intent PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(trajectory_intent PUBLIC
    trajectory_prediction_core
    trajectory_reconstruction
)

# Static two-aircraft Current-Best + two-alternate evaluator. It consumes the
# 46-point mean/cone already reconstructed by trajectory_intent and adds no
# second reconstruction or covariance implementation.
add_library(maneuver_combination_evaluator STATIC
    src/selection/ManeuverCombinationEvaluator.cpp
)
add_library(collision_avoidance::maneuver_combination_evaluator ALIAS
    maneuver_combination_evaluator
)
target_include_directories(maneuver_combination_evaluator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(maneuver_combination_evaluator PUBLIC
    trajectory_intent
)

# Per-aircraft five-stage runtime worker. ROS callbacks remain in the onboard
# node; this target owns only deterministic prediction/selection state and one
# SPSC-fed computation thread.
add_library(maneuver_selection_worker STATIC
    src/selection/ManeuverSelectionWorker.cpp
)
add_library(collision_avoidance::maneuver_selection_worker ALIAS
    maneuver_selection_worker
)
target_include_directories(maneuver_selection_worker PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(maneuver_selection_worker PUBLIC
    maneuver_combination_evaluator
    Threads::Threads
)

add_library(trajectory_intent_transport STATIC
    src/communication/TrajectoryIntentTransport.cpp
)
add_library(collision_avoidance::trajectory_intent_transport ALIAS
    trajectory_intent_transport
)
target_include_directories(trajectory_intent_transport PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(trajectory_intent_transport rclcpp)
target_link_libraries(trajectory_intent_transport
    trajectory_intent
    "${collision_avoidance_typesupport_target}"
)

add_library(distributed_maneuver_selection_runtime STATIC
    src/communication/DistributedManeuverSelectionRuntime.cpp
)
add_library(collision_avoidance::distributed_maneuver_selection_runtime ALIAS
    distributed_maneuver_selection_runtime
)
target_include_directories(distributed_maneuver_selection_runtime PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(distributed_maneuver_selection_runtime rclcpp px4_msgs)
target_link_libraries(distributed_maneuver_selection_runtime
    maneuver_selection_worker
    trajectory_intent_transport
)

install(TARGETS
    common_coordinate_transform
    ground_to_eas_adapter
    trajectory_prediction_core
    trajectory_reconstruction
    trajectory_intent
    maneuver_combination_evaluator
    maneuver_selection_worker
    trajectory_intent_transport
    distributed_maneuver_selection_runtime
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
ament_export_dependencies(rclcpp)
ament_export_dependencies(px4_msgs)
ament_export_dependencies(Threads)
ament_export_dependencies(rosidl_default_runtime)

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
    ground_to_eas_adapter
    maneuver_selection_worker
    trajectory_intent_transport
    distributed_maneuver_selection_runtime
)

add_executable(coordinate_transformer_node
    src/nodes/coordinate_transformer_main.cpp
    src/coordinate/TransferSameCoordinate.cpp
)
target_include_directories(coordinate_transformer_node PRIVATE include)
ament_target_dependencies(coordinate_transformer_node rclcpp px4_msgs)
target_link_libraries(coordinate_transformer_node
    common_coordinate_transform
    "${collision_avoidance_typesupport_target}"
)

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

    ament_add_gtest(test_trajectory_uncertainty
        test/test_trajectory_uncertainty.cpp
    )
    target_link_libraries(test_trajectory_uncertainty
        trajectory_prediction_core
    )
    target_include_directories(test_trajectory_uncertainty PRIVATE include)

    ament_add_gtest(test_trajectory_intent
        test/test_trajectory_intent.cpp
    )
    target_link_libraries(test_trajectory_intent
        trajectory_intent
    )
    target_include_directories(test_trajectory_intent PRIVATE include)

    ament_add_gtest(test_trajectory_intent_transport
        test/test_trajectory_intent_transport.cpp
    )
    target_link_libraries(test_trajectory_intent_transport
        trajectory_intent_transport
    )
    target_include_directories(test_trajectory_intent_transport PRIVATE include)

    ament_add_gtest(test_maneuver_combination_evaluator
        test/test_maneuver_combination_evaluator.cpp
    )
    target_link_libraries(test_maneuver_combination_evaluator
        maneuver_combination_evaluator
    )
    target_include_directories(test_maneuver_combination_evaluator PRIVATE include)

    ament_add_gtest(test_maneuver_selection_worker
        test/test_maneuver_selection_worker.cpp
    )
    target_link_libraries(test_maneuver_selection_worker
        maneuver_selection_worker
    )
    target_include_directories(test_maneuver_selection_worker PRIVATE include)

    ament_add_gtest(test_distributed_maneuver_selection_runtime
        test/test_distributed_maneuver_selection_runtime.cpp
    )
    ament_target_dependencies(
        test_distributed_maneuver_selection_runtime rclcpp px4_msgs)
    target_link_libraries(test_distributed_maneuver_selection_runtime
        distributed_maneuver_selection_runtime
    )
    target_include_directories(
        test_distributed_maneuver_selection_runtime PRIVATE include)

    ament_add_gtest(test_global_types
        test/test_global_types.cpp
    )
    target_include_directories(test_global_types PRIVATE include)

    ament_add_gtest(test_common_ned_transform
        test/test_common_ned_transform.cpp
    )
    target_link_libraries(test_common_ned_transform
        common_coordinate_transform
    )
    target_include_directories(test_common_ned_transform PRIVATE include)

    ament_add_gtest(test_ground_to_eas_adapter
        test/test_ground_to_eas_adapter.cpp
    )
    target_link_libraries(test_ground_to_eas_adapter
        ground_to_eas_adapter
    )
    target_include_directories(test_ground_to_eas_adapter PRIVATE include)
endif()

ament_package()
