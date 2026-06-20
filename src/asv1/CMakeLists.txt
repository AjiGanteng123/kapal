cmake_minimum_required(VERSION 3.8)
project(asv1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(node_navigasi src/node_navigasi.cpp)
ament_target_dependencies(node_navigasi rclcpp std_msgs geometry_msgs sensor_msgs)
install(TARGETS node_navigasi DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  asv1/node_kamera.py
  asv1/node_deteksi.py
  asv1/node_lidar.py
  asv1/node_misi.py
  asv1/node_motor.py
  asv1/bide_trainer.py
  asv1/sim_bridge.py
  scripts/viewer.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES resource/asv1
  DESTINATION share/ament_index/resource_index/packages)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
