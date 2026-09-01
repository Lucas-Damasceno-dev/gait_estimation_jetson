cmake_minimum_required(VERSION 3.8)
project(pressure_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# encontra as dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_serial REQUIRED)

include_directories(
  include/pressure_node
)

add_library(pressure_driver SHARED
  src/driver/pressure_factory.cpp
  src/driver/insole/insole_driver.cpp
)

target_include_directories(pressure_driver PUBLIC include)

ament_target_dependencies(pressure_driver
  common_serial
)

add_library(pressure_control SHARED
  src/control/node/parameter_manager.cpp
)

target_include_directories(pressure_control PUBLIC include)

ament_target_dependencies(pressure_control
  rclcpp
)

add_library(pressure_node_lib SHARED
  src/pressure_node.cpp
)

target_link_libraries(pressure_node_lib
  pressure_driver
  pressure_control
)

ament_target_dependencies(pressure_node_lib
  rclcpp
  interfaces
  common_serial
)

add_executable(pressure_node src/main.cpp)
target_link_libraries(pressure_node pressure_node_lib)

install(TARGETS
  pressure_node
  pressure_node_lib
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
