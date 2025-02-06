set(msg_files
  "msg/HelloSample.msg"
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
  std_msgs)

ament_export_dependencies(rosidl_default_runtime)

add_subdirectory(nodes/hello_world_cpp)
add_subdirectory(nodes/hello_world_py)
