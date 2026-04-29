cmake_minimum_required(VERSION 3.8)
project(ur_automata_scan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ── Libreria geometrica pura (no ROS, solo Eigen + geometry_msgs) ─────────────
add_library(sphere_waypoint_generator
  src/sphere_waypoint_generator.cpp
)
target_include_directories(sphere_waypoint_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sphere_waypoint_generator
  geometry_msgs
  Eigen3
)

# ── Nodo esecutore ────────────────────────────────────────────────────────────
add_executable(scan_executor_node
  src/scan_executor_node.cpp
)
target_link_libraries(scan_executor_node
  sphere_waypoint_generator
)
ament_target_dependencies(scan_executor_node
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_ros_planning_interface
  Eigen3
)

# ── Install ───────────────────────────────────────────────────────────────────
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS sphere_waypoint_generator scan_executor_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
