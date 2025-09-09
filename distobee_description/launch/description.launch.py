from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context):
    actions = []

    # robot structure TF publisher
    urdf = xacro.process_file(
        str(
            get_package_share_path("distobee_description")
            / "urdf"
            / "distobee.urdf.xacro"
        )
    ).toxml()
    actions += [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf}, {"ignore_timestamp": True}],
        ),
    ]

    # joint state publisher: muxes joint states for display in RViz
    actions += [
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[
                {
                    "rate": 30,
                    "source_list": [
                        "/ros2_control/joint_states",
                    ],
                },
            ],
        ),
    ]

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
