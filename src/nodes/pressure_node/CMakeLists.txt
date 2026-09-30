cmake_minimum_required(VERSION 3.8)
project(pressure_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_serial REQUIRED)

include_directories(
  include/pressure_node
)

# 1. Nó principal de aquisição da palmilha
add_executable(pressure_node
  src/main.cpp
  src/pressure_node.cpp
  src/driver/pressure_factory.cpp
  src/driver/insole/insole_driver.cpp
  src/control/node/parameter_manager.cpp
)

target_include_directories(pressure_node PUBLIC 
  include
  include/pressure_node
)

ament_target_dependencies(pressure_node
  rclcpp
  interfaces
  std_msgs
  common_serial
)

# 2. Monitor nativo C++ de calcanhar (100% livre de problemas com Python/rclpy)
add_executable(monitor_calcanhar_node
  src/monitor_calcanhar_node.cpp
)
ament_target_dependencies(monitor_calcanhar_node
  rclcpp
  interfaces
)

# 3. Monitor completo C++ das duas palmilhas (16 sensores cada)
add_executable(monitor_palmilhas_completo_node
  src/monitor_palmilhas_completo_node.cpp
)
ament_target_dependencies(monitor_palmilhas_completo_node
  rclcpp
  interfaces
)

install(TARGETS
  pressure_node
  monitor_calcanhar_node
  monitor_palmilhas_completo_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
