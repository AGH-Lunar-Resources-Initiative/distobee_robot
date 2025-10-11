from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction, ExecuteProcess, TimerAction
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
    actions += [
        Node(
            package="odrive_can",
            executable="odrive_can_node",
            name="odrive_pipes",
            namespace="odrive_pipes",
            parameters=[{
                "node_id": 30,
                "interface": "vcan0"
            }
            ]
        ),
        Node(
            package="odrive_can",
            executable="odrive_can_node",
            name="odrive_tilt",
            namespace="odrive_tilt",
            parameters=[{
                "node_id": 22,
                "interface": "vcan0"
            }
            ]
        )
    ]
    actions += [
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                     cmd=[
                    "ros2", "service", "call",
                    "/odrive_pipes/request_axis_state",
                    "odrive_can/srv/AxisState",
                    '{"axis_requested_state": 8}'  # set CLOSED-LOOP-CONTROL on pipes
                     ],
                    shell=False
                    ),
                ExecuteProcess(
                     cmd=[
                    "ros2", "service", "call",
                    "/odrive_tilt/request_axis_state",
                    "odrive_can/srv/AxisState",
                    '{"axis_requested_state": 8}'  # set CLOSED-LOOP-CONTROL on tilt
                     ],
                    shell=False
                    )
            ]
        )
    ]

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
