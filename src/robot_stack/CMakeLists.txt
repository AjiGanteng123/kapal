cmake_minimum_required(VERSION 3.8)
project(robot_stack)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── C++ Nodes ──────────────────────────────────────────

add_executable(lidar_node src/lidar_node.cpp)
ament_target_dependencies(lidar_node rclcpp sensor_msgs std_msgs)
install(TARGETS lidar_node DESTINATION lib/${PROJECT_NAME})

add_executable(motor_node src/motor_node.cpp)
ament_target_dependencies(motor_node rclcpp geometry_msgs std_msgs)
install(TARGETS motor_node DESTINATION lib/${PROJECT_NAME})

# ── Python Nodes (installed as executables) ────────────

install(PROGRAMS
  robot_stack/vision_node.py
  robot_stack/autonomous_node.py
  DESTINATION lib/${PROJECT_NAME})

# ── Resources, config, launch ──────────────────────────

install(FILES resource/robot_stack
  DESTINATION share/ament_index/resource_index/packages)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# ── Finish ─────────────────────────────────────────────

ament_package()
