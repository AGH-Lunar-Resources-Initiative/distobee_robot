from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_path


def launch_setup(context):
    actions = []

    actions += [
        Node(
            package="distobee_sifter",
            executable="sifter_driver",
            parameters=[{
                "tcp_host": '192.168.1.43',  # sifter IP
                "tcp_port": 6000,
            }]
        ),
    ]

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
