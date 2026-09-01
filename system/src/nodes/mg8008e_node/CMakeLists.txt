cmake_minimum_required(VERSION 3.8)
project(mg8008e_node)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_serial REQUIRED)

include_directories(
  include/actuator_node
)

add_library(actuator_driver SHARED
  src/driver/mg8008e/mg8008e_driver.cpp
  src/driver/actuator_factory.cpp
)
target_include_directories(actuator_driver PUBLIC include)

ament_target_dependencies(actuator_driver
  common_serial
  rclcpp
)

add_library(actuator_control SHARED
  src/control/node/parameter_manager.cpp
)
target_include_directories(actuator_control PUBLIC include)

ament_target_dependencies(actuator_control
  rclcpp
)

add_library(actuator_node_lib SHARED
  src/node/actuator_node.cpp
)

target_link_libraries(actuator_node_lib
  actuator_driver
  actuator_control
)

ament_target_dependencies(actuator_node_lib
  rclcpp
  interfaces
  common_serial
)

add_executable(actuator_node src/main.cpp)
target_link_libraries(actuator_node actuator_node_lib)

install(TARGETS
  actuator_node
  actuator_node_lib
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
