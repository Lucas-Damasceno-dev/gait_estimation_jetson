cmake_minimum_required(VERSION 3.8)
project(client)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

include_directories(
  include/client
)

add_library(client_lib SHARED
  src/client.cpp
)

ament_target_dependencies(client_lib
  rclcpp
  interfaces
)

add_executable(client src/main.cpp)
target_link_libraries(client client_lib)

install(TARGETS
  client
  client_lib
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
