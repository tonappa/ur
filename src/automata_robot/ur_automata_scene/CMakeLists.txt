cmake_minimum_required(VERSION 3.22)
project(ur_automata_scene)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Libreria che costruisce la PlanningScene (riusabile da altri nodi).
add_library(scan_scene_builder src/scan_scene_builder.cpp)
target_include_directories(scan_scene_builder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(scan_scene_builder
  rclcpp
  geometry_msgs
  shape_msgs
  moveit_msgs
)

# Eseguibile del nodo che applica la scena via /apply_planning_scene.
add_executable(scene_publisher_node src/scene_publisher_node.cpp)
target_link_libraries(scene_publisher_node scan_scene_builder)
ament_target_dependencies(scene_publisher_node
  rclcpp
  moveit_msgs
)

install(TARGETS scan_scene_builder
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS scene_publisher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(scan_scene_builder)
ament_export_dependencies(rclcpp geometry_msgs shape_msgs moveit_msgs)

ament_package()
