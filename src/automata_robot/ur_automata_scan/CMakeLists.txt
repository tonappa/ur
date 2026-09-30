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
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(yaml-cpp REQUIRED)

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

# ── Libreria DP per la scelta della sequenza (pura, solo STL) ─────────────────
add_library(scan_sequence_planner
  src/scan_sequence_planner.cpp
)
target_include_directories(scan_sequence_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ── Recorded scan file (calibration run -> replay), pure yaml-cpp ─────────────
add_library(scan_recording
  src/scan_recording.cpp
)
target_include_directories(scan_recording PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(scan_recording yaml-cpp::yaml-cpp)

# ── Scan node (offline enumeration + DP + planner chain, calibration run) ─────
add_executable(scan_sequence_node
  src/scan_sequence_node.cpp
)
target_link_libraries(scan_sequence_node
  sphere_waypoint_generator
  scan_sequence_planner
  scan_recording
)
ament_target_dependencies(scan_sequence_node
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  Eigen3
  std_srvs
  geometric_shapes
)

# ── Replay of a recorded scan (work sessions, no planning) ────────────────────
add_executable(scan_replay_node
  src/scan_replay_node.cpp
)
target_link_libraries(scan_replay_node
  scan_recording
)
ament_target_dependencies(scan_replay_node
  rclcpp
  visualization_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  Eigen3
  std_srvs
)

# ── Install ───────────────────────────────────────────────────────────────────
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS sphere_waypoint_generator scan_sequence_planner scan_recording
          scan_sequence_node scan_replay_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sequence_planner test/test_sequence_planner.cpp)
  target_link_libraries(test_sequence_planner scan_sequence_planner)
  ament_add_gtest(test_sectors test/test_sectors.cpp)
  target_link_libraries(test_sectors sphere_waypoint_generator)
  ament_target_dependencies(test_sectors geometry_msgs Eigen3)
  ament_add_gtest(test_scan_recording test/test_scan_recording.cpp)
  target_link_libraries(test_scan_recording scan_recording)
  ament_add_gtest(test_frustum_markers test/test_frustum_markers.cpp)
  target_include_directories(test_frustum_markers PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_frustum_markers rclcpp visualization_msgs)
endif()

ament_package()
