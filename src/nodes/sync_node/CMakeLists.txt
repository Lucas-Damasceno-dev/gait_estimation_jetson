cmake_minimum_required(VERSION 3.8)
project(sync_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(message_filters REQUIRED)

include_directories(
  include/sync_node
)

add_library(sync_node_lib SHARED
  src/sync_node.cpp
)

ament_target_dependencies(sync_node_lib
  rclcpp
  interfaces
  message_filters
)

add_executable(sync_node src/main.cpp)
target_link_libraries(sync_node sync_node_lib)

# Instalação correta de binários e bibliotecas compartilhadas
install(TARGETS
  sync_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  sync_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(sync_node_lib)
ament_package()
