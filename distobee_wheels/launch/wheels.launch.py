from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_path


def launch_setup(context):
    actions = []

    actions += [
        # Node(
            # package="joy_linux",
            # executable="joy_linux_node",
            # parameters=[
                # {
                    # "dev_name": "Logitech Gamepad",
                # }
            # ],
        # ),
        Node(
            package="joy",
            executable="joy_node",
        ),
        Node(
            package="distobee_wheels",
            executable="gamepad_driving",
            parameters=[
                str(
                    get_package_share_path("distobee_wheels")
                    / "config"
                    / "gamepad_driving.yaml"
                    
                ),
            ],
        ),
        Node(
            package="distobee_wheels",
            executable="twist_controller",
        ),
        Node(
            package="distobee_wheels",
            executable="wheel_joints_publisher",
        ),
    ]

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
