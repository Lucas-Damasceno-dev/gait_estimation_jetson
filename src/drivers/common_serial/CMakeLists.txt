cmake_minimum_required(VERSION 3.8)
project(common_serial)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(serial_handler
  src/serial_handler.cpp
)

target_include_directories(serial_handler PUBLIC include)

install(TARGETS serial_handler
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_libraries(serial_handler)
ament_export_include_directories("include")
ament_package()
