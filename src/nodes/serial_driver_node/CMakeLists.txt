cmake_minimum_required(VERSION 3.8)
project(serial_driver_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(common_serial REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

add_executable(serial_driver_node src/serial_driver_node.cpp)
ament_target_dependencies(serial_driver_node
  rclcpp
  interfaces
  common_serial
  std_msgs
)

# Emulador STM32 para testes offline / sem hardware
add_executable(stm32_emulator_node src/stm32_emulator_node.cpp)
ament_target_dependencies(stm32_emulator_node
  rclcpp
  interfaces
  common_serial
  std_msgs
)

install(TARGETS
  serial_driver_node
  stm32_emulator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
