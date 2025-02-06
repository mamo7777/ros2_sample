rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/HelloSample.msg"
  DEPENDENCIES
  std_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rosidl_typesupport_cpp)


set_target_properties(${PROJECT_NAME}__rosidl_generator_c PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${PROJECT_NAME}__rosidl_typesupport_introspection_c PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${PROJECT_NAME}__rosidl_generator_py PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${PROJECT_NAME}_s__rosidl_typesupport_fastrtps_c PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${PROJECT_NAME}_s__rosidl_typesupport_c PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${PROJECT_NAME}_s__rosidl_typesupport_introspection_c PROPERTIES LINKER_LANGUAGE CXX)

add_subdirectory(nodes/hello_world_cpp)
add_subdirectory(nodes/hello_world_py)
