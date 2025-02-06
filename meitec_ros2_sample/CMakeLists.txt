cmake_minimum_required(VERSION 3.5)
project(meitec_ros2_sample)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O1 -Wall")
set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-Wall -Wextra -Wpedantic -Wuninitialized
                    -fdiagnostics-color=always -Werror)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(spdlog CONFIG REQUIRED)

add_subdirectory(ros2)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
