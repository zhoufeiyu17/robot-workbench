cmake_minimum_required(VERSION 3.8)
project(workbench_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StreamRef.msg"
  "msg/SourceDescriptor.msg"
  "msg/RobotState.msg"
  "msg/Observation.msg"
  "msg/ModelRequest.msg"
  "msg/ModelInput.msg"
  "msg/ModelResult.msg"
  "msg/DataEvent.msg"
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
