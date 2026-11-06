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
        package="spark_max",
        executable="spark_max",
        parameters=[{"can_id": 1}],
        name="SparkMax_FL",
        namespace="drive",
    )
    test_node2 = Node(
        package="spark_max",
        executable="spark_max",
        parameters=[{"can_id": 2}],
        name="SparkMax_FR",
        namespace="drive",
    )
    test_node3 = Node(
        package="spark_max",
        executable="spark_max",
        parameters=[{"can_id": 3}],
        name="SparkMax_BL",
        namespace="drive",
    )
    test_node4 = Node(
        package="spark_max",
        executable="spark_max",
        parameters=[{"can_id": 4}],
        name="SparkMax_BR",
        namespace="drive",
    )

    excavation_buckets_node = Node(
        package="spark_max",
        executable="spark_max",
        parameters=[{"can_id": 5}],
        name="Excavation_Buckets_Spark_Max",
        namespace="excavation",
    )

    excavation_actuator_node1 = Node(
        package="spark_max",
        executable="spark_max",
        parameters=[{"can_id": 6}],
        name="Excavation_Linear_Actuator_L_Spark_Max",
        namespace="excavation",
    )

    excavation_actuator_node2 = Node(
        package="spark_max",
        executable="spark_max",
        parameters=[{"can_id": 7}],
        name="Excavation_Linear_Actuator_R_Spark_Max",
        namespace="excavation",
    )

    deposition_motor_node = Node(
        package="spark_max",
        executable="spark_max",
        parameters=[{"can_id": 8}],
        name="Deposition_Spark_Max",
        namespace="deposition",
    )

    kinematics_node = Node(
        package="diffdrive_kinematics",
        executable="diffdrive_kinematics",
        namespace="drive",
    )

    camera_node_1 = Node(
        package="camera",
        executable="camera",
        namespace="camera1",
    )
    camera_node_2 = Node(
        package="camera",
        executable="camera",
        namespace="camera2",
    )

    return LaunchDescription(
        [
            test_node,
            test_node2,
            test_node3,
            test_node4,
            excavation_buckets_node,
            excavation_actuator_node1,
            excavation_actuator_node2,
            deposition_motor_node,
            kinematics_node,
            camera_node_1,
            camera_node_2,
        ]
    )
