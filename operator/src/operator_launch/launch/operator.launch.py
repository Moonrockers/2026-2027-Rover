from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    console_node = Node(
        package="driver_console",
        executable="driver_console",
    )
    autonomy_node = Node(
        package="driver_console",
        executable="autonomy_controller",
        name="Autonomy_Controller",
    )

    return LaunchDescription([console_node, autonomy_node])
