cmake_minimum_required(VERSION 3.5)
  project(collision_avoidance)          # ← 패키지 이름

  set(CMAKE_CXX_STANDARD 17)

  find_package(eigen3_cmake_module REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(px4_ros2_cpp REQUIRED)

  include_directories(include)
  add_executable(collision_avoidance_node
    src/main.cpp
    src/CollisionAvoidanceControl.cpp
  )
  find_package(px4_msgs REQUIRED)
  ament_target_dependencies(collision_avoidance_node rclcpp px4_msgs px4_ros2_cpp)
  target_link_libraries(collision_avoidance_node Eigen3::Eigen)

  install(TARGETS collision_avoidance_node DESTINATION lib/${PROJECT_NAME})

  ament_package()
