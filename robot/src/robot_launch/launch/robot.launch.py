from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def spawner(controller, ros_args=None):
    """Spawn a controller; ros_args is a single string like '-r a:=b' passed to that controller."""
    arguments = [
        controller,
        "--controller-manager",
        "/controller_manager",
        "--controller-manager-timeout",
        "30",
    ]
    if ros_args:
        arguments += ["--controller-ros-args", ros_args]
    return Node(package="controller_manager", executable="spawner", arguments=arguments)


def generate_launch_description():
    # pkg = FindPackageShare("spark_max_hardware")
    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")

    args = [
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robot_launch"), "urdf", "rover.urdf.xacro"]
            ),
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robot_launch"), "config", "controllers.yaml"]
            ),
        ),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        # simulation is safe without hardware; spark_max requires verified CAN frames.
        DeclareLaunchArgument("protocol", default_value="simulation"),
        DeclareLaunchArgument("use_cameras", default_value="false"),
    ]

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                description_file,
                " can_interface:=",
                LaunchConfiguration("can_interface"),
                " protocol:=",
                LaunchConfiguration("protocol"),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="both",
    )

    bucket_chain_visualizer = Node(
        package="robot_launch",
        executable="bucket_chain_visualizer",
        output="both",
    )

    # Replaces the eight SparkMax nodes and the diffdrive_kinematics node.
    # Jazzy's controller manager reads the description from the /robot_description topic.
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_file],
        remappings=[("~/robot_description", "/robot_description")],
        output="both",
    )

    joint_state_broadcaster = spawner("joint_state_broadcaster")

    # Remaps keep the existing command entry points.
    diff_drive = spawner(
        "diff_drive_controller", "-r /diff_drive_controller/cmd_vel:=/drive/cmd_vel"
    )
    excavation = spawner(
        "excavation_controller",
        "-r /excavation_controller/commands:=/excavation/commands",
    )
    deposition = spawner(
        "deposition_controller",
        "-r /deposition_controller/commands:=/deposition/commands",
    )

    # Start the other controllers once the broadcaster spawner has finished.
    after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diff_drive, excavation, deposition],
        )
    )

    camera_node_1 = Node(
        package="camera",
        executable="camera",
        namespace="camera1",
        condition=IfCondition(LaunchConfiguration("use_cameras")),
    )
    camera_node_2 = Node(
        package="camera",
        executable="camera",
        namespace="camera2",
        condition=IfCondition(LaunchConfiguration("use_cameras")),
    )

    return LaunchDescription(
        args
        + [
            robot_state_publisher,
            bucket_chain_visualizer,
            ros2_control_node,
            joint_state_broadcaster,
            after_broadcaster,
            camera_node_1,
            camera_node_2,
        ]
    )
