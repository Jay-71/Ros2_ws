cmake_minimum_required(VERSION 3.22)
project(robotic_arm_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(move_arm_node src/move_arm.cpp)
ament_target_dependencies(move_arm_node rclcpp moveit_ros_planning_interface)


install(TARGETS
  move_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

# Add the Monitor Node
add_executable(position_monitor_node src/position_monitor.cpp)
ament_target_dependencies(position_monitor_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# Install the Node so the launch file can find it
install(TARGETS position_monitor_node DESTINATION lib/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
