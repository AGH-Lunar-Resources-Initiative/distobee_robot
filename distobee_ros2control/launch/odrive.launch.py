from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("distobee_description"),
                 "urdf", "distobee_description.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    # Controller config file
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("distobee_ros2control"),
            "config",
            "distobee_controllers.yaml",
        ]
    )

    return [
        # Load URDF and ros2control controllers
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, robot_controllers],
            output='screen'
        ),

        # Publish Joint States from Odrive drivers.
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager"],
            output="screen"
        ),
        # Ackermann controller's parameters are in the config file
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=["ackermann_drive",
                       "--controller-manager", "/controller_manager"],
            output='screen'
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
