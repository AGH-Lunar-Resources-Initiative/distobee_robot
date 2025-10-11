from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction, ExecuteProcess, TimerAction
)
from launch_ros.actions import Node

# Map of ODrive instances: name, namespace, node_id, interface
ODRIVE_INSTANCES = [
    ("odrive_back_left", 11, "can1"),
    ("odrive_back_right", 12, "can1"),
    ("odrive_front_left", 21, "can0"),
    ("odrive_front_right", 22, "can0"),
    ("odrive_pipes", 30, "can0"),
    ("odrive_tilt", 40, "can0"),
]

def launch_setup(context):
    actions = []

    # Create ODrive nodes
    actions += [
        Node(
            package="odrive_can",
            executable="odrive_can_node",
            name=name,
            namespace=name,
            parameters=[{
                "node_id": node_id,
                "interface": interface,
            }]
        )
        for name, node_id, interface in ODRIVE_INSTANCES
    ]

    # Wheel driver node
    actions += [
        Node(
            package="distobee_hardware",
            executable="wheel_driver",
            name="wheel_driver",
        ),
    ]

    # Initialize ODrive axis states after a delay
    actions += [
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2", "service", "call",
                        f"/{name}/request_axis_state",
                        "odrive_can/srv/AxisState",
                        '{"axis_requested_state": 8}'
                    ],
                    shell=False
                )
                for name, _, _ in ODRIVE_INSTANCES
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
