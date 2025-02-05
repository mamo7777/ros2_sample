add_executable(hello_world_cpp main.cc)

ament_target_dependencies(hello_world_cpp spdlog rclcpp geometry_msgs sensor_msgs std_srvs)

target_link_libraries(hello_world_cpp spdlog::spdlog)

install(TARGETS hello_world_cpp DESTINATION lib/${PROJECT_NAME})
