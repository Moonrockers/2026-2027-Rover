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
    test_node = Node(
        package="spark_max", executable="spark_max", parameters=[{"can_id": 1}]
    )
    diff_drive_node = Node(
        package="differential_drive", executable="differential_drive"
    )

    return LaunchDescription([test_node])
