cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/IMUData.msg"
  "msg/IMUState.msg"
  "msg/PressureUnitSensor.msg"
  "msg/PressureData.msg"
  "msg/PressureState.msg"
  "msg/ActuatorCommand.msg"
  "msg/MG8008ECommand.msg"
  "msg/ActuatorState.msg"
  "msg/MG8008EState.msg"
  "msg/SyncedSensorData.msg"
  "msg/Time.msg"
)

set(srv_files
  "srv/SetTorque.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
