from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        parameters=[
            {
                "address": LaunchConfiguration("rosbridge_address"),
                "port": LaunchConfiguration("rosbridge_port"),
            }
        ],
    )
    web_server = ExecuteProcess(
        cmd=[
            "python3",
            "-m",
            "http.server",
            LaunchConfiguration("web_port"),
            "--directory",
            PathJoinSubstitution([FindPackageShare("operator_launch"), "web"]),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rosbridge_address",
                default_value="0.0.0.0",
                description="Address for the rosbridge WebSocket server",
            ),
            DeclareLaunchArgument(
                "rosbridge_port",
                default_value="9090",
                description="Port for the rosbridge WebSocket server",
            ),
            DeclareLaunchArgument(
                "web_port",
                default_value="8000",
                description="Port for the operator web page",
            ),
            rosbridge_node,
            web_server,
        ]
    )
