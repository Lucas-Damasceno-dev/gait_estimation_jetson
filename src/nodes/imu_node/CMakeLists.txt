cmake_minimum_required(VERSION 3.8)
project(imu_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  include/imu_node
)

# Compilação unificada do nó principal da IMU diretamente em executável
add_executable(imu_node
  src/main.cpp
  src/node/imu_node.cpp
  src/node/imu_manager.cpp
  src/driver/imu.cpp
  src/driver/bno055imu.cpp
  src/control/node/parameter_manager.cpp
)

target_include_directories(imu_node PUBLIC
  include
  include/imu_node
)

ament_target_dependencies(imu_node
  rclcpp
  interfaces
  std_msgs
  sensor_msgs
)

install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
