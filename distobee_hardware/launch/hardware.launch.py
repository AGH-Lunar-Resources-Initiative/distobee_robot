from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    ExecuteProcess,
    TimerAction
)
from launch_ros.actions import Node

# Map of ODrive instances: name, node_id, interface
ODRIVE_INSTANCES = [
    ("odrive_back_left", 11, "can1"),
    ("odrive_back_right", 12, "can1"),
    ("odrive_front_left", 21, "can1"),
    ("odrive_front_right", 22, "can1"),
    ("odrive_pipes", 30, "can0"),
    ("odrive_tilt", 40, "can0"),
]
FEED_GS_IP = "192.168.1.232"
FEED_GS_PORTS = [5001, 5002] #, 5003] # distobee_main, distobee_alt, sifter

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
    # Switch odrives off on exit
    actions += [
        Node(
            package="distobee_hardware",
            executable="odrive_state_switcher",
        ),
    ]

    # Wheel driver node
    actions += [
        Node(
            package="distobee_hardware",
            executable="wheel_driver",
        ),
    ]

    # Feed drivers
    actions += [
        Node(
            package="distobee_hardware",
            executable="feed_driver",
            name="feed_driver_distobee_main",
            parameters=[{
                "udp_host": FEED_GS_IP,
                "udp_port": FEED_GS_PORTS[0],
            }],
            remappings=[
                ("set_feed", "/set_feed/distobee_main"),
            ]
        ),
        Node(
            package="distobee_hardware",
            executable="feed_driver",
            name="feed_driver_distobee_alt",
            parameters=[{
                "udp_host": FEED_GS_IP,
                "udp_port": FEED_GS_PORTS[1],
            }],
            remappings=[
                ("set_feed", "/set_feed/distobee_alt"),
            ]
        ),
    ]

    return actions

def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
