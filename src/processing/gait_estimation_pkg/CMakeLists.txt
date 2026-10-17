cmake_minimum_required(VERSION 3.8)
project(gait_estimation_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

include_directories(
  include
  ${ZMQ_INCLUDE_DIRS}
)

# Gait Estimation Node
add_executable(gait_estimation_node src/gait_estimation_node.cpp)
ament_target_dependencies(gait_estimation_node
  rclcpp
  interfaces
  sensor_msgs
  std_msgs
)

# Visualization Node com ZeroMQ e RViz
add_executable(visualization_node src/visualization_node.cpp)
ament_target_dependencies(visualization_node
  rclcpp
  interfaces
  sensor_msgs
  std_msgs
)
target_link_libraries(visualization_node ${ZMQ_LIBRARIES})

# Sensor RViz Visualizer Node (Palmilhas e IMUs com monitoramento em Hz)
add_executable(sensor_visualizer_node src/sensor_visualizer_node.cpp)
ament_target_dependencies(sensor_visualizer_node
  rclcpp
  interfaces
  sensor_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
)

install(TARGETS
  gait_estimation_node
  visualization_node
  sensor_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
