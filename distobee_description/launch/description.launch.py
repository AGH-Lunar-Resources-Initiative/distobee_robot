from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def launch_setup(context):
    def get_bool(name):
        return LaunchConfiguration(name).perform(context).lower() == "true"

    joint_state_publisher_gui = get_bool("joint_state_publisher_gui")

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

    parameters=[
        {
            "rate": 30,
            "source_list": [
                "/ros2_control/joint_states",
            ]
        },
    ]
    if joint_state_publisher_gui:
        # alternative joint state publisher with GUI
        actions += [
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=parameters,
            ),
        ]
    else:
        # joint state publisher
        # Required for 3D model joints to show up.
        actions += [
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                parameters=parameters,
            ),
        ]

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joint_state_publisher_gui",
                default_value="false",
                choices=["true", "false"],
                description="Start the joint state publisher in GUI mode.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
