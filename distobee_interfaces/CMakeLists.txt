cmake_minimum_required(VERSION 3.8)
project(distobee_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PipeStates.msg"
  "msg/WheelState.msg"
  "msg/WheelStates.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
