from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    console_node = Node(
        package="driver_console",
        executable="driver_console",
    )

    return LaunchDescription([console_node])
