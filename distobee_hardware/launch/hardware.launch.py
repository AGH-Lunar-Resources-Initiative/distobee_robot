from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
)
from launch_ros.actions import Node


def launch_setup(context):
    actions = []

    # ros2_control
    actions += [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace="ros2_control",
            parameters=[
                str(
                    get_package_share_path("distobee_hardware")
                    / "config"
                    / "ros2_control.yaml"
                ),
            ],
            remappings=[
                ("robot_description", "/robot_description"),
                ("ackermann_steering_controller/reference", "/cmd_vel"),
                ("ackermann_steering_controller/odometry", "/odometry/wheel"),
            ],
        ),
    ]
    actions += [
        Node(
            package="controller_manager",
            executable="spawner",
            namespace="ros2_control",
            arguments=[controller],
        )
        for controller in [
            "joint_state_broadcaster",
            "ackermann_steering_controller",
        ]
    ]

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
