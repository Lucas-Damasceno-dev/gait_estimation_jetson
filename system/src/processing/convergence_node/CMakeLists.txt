cmake_minimum_required(VERSION 3.8)
project(convergence_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED) 

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

include_directories(
  include/convergence_node
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
)

add_executable(convergence_node src/convergence_node.cpp src/csv_writer.cpp)
ament_target_dependencies(convergence_node rclcpp interfaces)
target_link_libraries(convergence_node ${Python3_LIBRARIES})

install(TARGETS
  convergence_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
