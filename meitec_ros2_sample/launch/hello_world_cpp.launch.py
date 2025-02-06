# /usr/bin/env python3
import launch
import launch_ros

def generate_launch_description() -> launch.LaunchDescription:
    ld = launch.LaunchDescription()

    ld.add_action(
        launch_ros.actions.Node(
            name="turtlesim_node",
            package="turtlesim",
            executable="turtlesim_node",
        )
    )

    ld.add_action(
        launch_ros.actions.Node(
            name="hello_world_cpp",
            package="meitec_ros2_sample",
            executable="hello_world_cpp",
        )
    )

    return ld
