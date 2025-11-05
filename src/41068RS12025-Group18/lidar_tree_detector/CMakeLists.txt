cmake_minimum_required(VERSION 3.5)
project(lidar_tree_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TreeDetection.msg"
  "msg/TreeDetectionArray.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Node executable
add_executable(lidar_tree_detector_node src/lidar_tree_detector_node.cpp)
ament_target_dependencies(
  lidar_tree_detector_node
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
)

# --- Critical: ensure node depends on generated message headers ---
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_dependencies(lidar_tree_detector_node ${cpp_typesupport_target})

target_include_directories(lidar_tree_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

# Install node
install(TARGETS lidar_tree_detector_node DESTINATION lib/${PROJECT_NAME})

# Ensure messages get installed
install(DIRECTORY msg DESTINATION share/${PROJECT_NAME})

ament_package()
