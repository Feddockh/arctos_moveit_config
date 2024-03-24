cmake_minimum_required(VERSION 3.5)
project(arctos_moveit_config)

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Custom message requirements
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   msg/ArctosJointState.msg
# )
# ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
)

# Declare C++ executables
add_executable(realsense_to_moveit src/realsense_to_moveit_node.cpp)
ament_target_dependencies(realsense_to_moveit
  rclcpp
  sensor_msgs
  octomap_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  octomap_ros
  PCL
  message_filters
  pcl_msgs
  urdf
  visualization_msgs
  tf2
  tf2_ros
  geometry_msgs
)

add_executable(serial_joint_state_publisher src/serial_joint_state_publisher.cpp)
ament_target_dependencies(serial_joint_state_publisher
  rclcpp
  sensor_msgs
  
)

# Install the executable
install(TARGETS
  realsense_to_moveit
  serial_joint_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(realsense_to_moveit
  ${PCL_LIBRARIES}
)

# Other installation rules for your package
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
