cmake_minimum_required(VERSION 3.8)
project(time_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED) 

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

include_directories(
  include/time_node
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
)

add_executable(pressure_time_node src/pressure_time_node.cpp src/csv_writer.cpp)
ament_target_dependencies(pressure_time_node rclcpp interfaces)
target_link_libraries(pressure_time_node ${Python3_LIBRARIES})

add_executable(actuator_time_node src/actuator_time_node.cpp src/csv_writer.cpp)
ament_target_dependencies(actuator_time_node rclcpp interfaces)
target_link_libraries(actuator_time_node ${Python3_LIBRARIES})

add_executable(imu_time_node src/imu_time_node.cpp src/csv_writer.cpp)
ament_target_dependencies(imu_time_node rclcpp interfaces)
target_link_libraries(imu_time_node ${Python3_LIBRARIES})

install(TARGETS
  pressure_time_node
  actuator_time_node
  imu_time_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
