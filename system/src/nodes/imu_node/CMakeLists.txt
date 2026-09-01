cmake_minimum_required(VERSION 3.8)
project(imu_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# detecta Raspberry Pi pelo sistema
if(EXISTS "/proc/cpuinfo")
  file(READ "/proc/cpuinfo" CPUINFO)
  if(CPUINFO MATCHES "BCM|Raspberry")
    set(RASPBERRY_PI TRUE)
  else()
    set(RASPBERRY_PI FALSE)
  endif()
else()
  set(RASPBERRY_PI FALSE)
endif()

# encontra as dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include/imu_node
)

add_library(imu_driver SHARED
  src/driver/imu.cpp
  src/driver/bno055imu.cpp
)

if (RASPBERRY_PI)
  target_link_libraries(imu_driver PRIVATE wiringPi)
endif()

target_include_directories(imu_driver PUBLIC include)


add_library(imu_control SHARED
  src/control/node/parameter_manager.cpp
)
target_include_directories(imu_control PUBLIC include)

ament_target_dependencies(imu_control
  rclcpp
)

add_library(imu_node_lib SHARED
  src/node/imu_node.cpp 
  src/node/imu_manager.cpp
)

target_link_libraries(imu_node_lib
  imu_driver
  imu_control
)

ament_target_dependencies(imu_node_lib
  rclcpp
  interfaces
  sensor_msgs
)

add_executable(imu_node src/main.cpp)
target_link_libraries(imu_node PRIVATE imu_node_lib)

target_include_directories(imu_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(RASPBERRY_PI)
  message(STATUS "Compilando para Raspberry Pi: usando wiringPi")
  target_link_libraries(imu_node PRIVATE wiringPi)
else()
  message(STATUS "Fora do Raspberry Pi: usando wiringPi stub")
  target_sources(imu_node
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../wiringpi/wiringPi/wiringPi_stub.c)
endif()

install(TARGETS
  imu_node
  imu_node_lib
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
