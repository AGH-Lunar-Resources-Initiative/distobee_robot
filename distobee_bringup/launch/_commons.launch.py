# This launch file includes all modules and it allows to selectively launch them by setting appropriate arguments.
# It allows to quickly create launch files with custom configurations and maintain their readability.
#
# Usage:
# All modules have a "{module}" argument, which is used to enable or disable the module.
# This argument is always false by default.
# If you set "{module}" to "true", please also consider all other "{module}.*" arguments.
# The default values are always empty strings, false booleans, zero numbers, etc.
# Some of them can be left out, but others might be required.
# If "...{module}" is not set, "...{module}.*" arguments should not be specified.
# When running multiple launch configurations together, please make sure that each "{module}" argument is set to "true" at most in only one of them.
#
# If any "*.composition" argument is set to "true", please ensure that "component_container" was also set to "true" in this launch file or in any other one that is currently running.
# Since any launch file based on this one is meant to be run by the user, it should only declare its own arguments if necessary. In that case, all arguments should also provide sensible default values.

from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

COMPONENT_CONTAINER_NAME = "kalman_container"


def launch_setup(context):
    def get_bool(name):
        return LaunchConfiguration(name).perform(context).lower() == "true"

    def get_str(name):
        return LaunchConfiguration(name).perform(context)

    description = []

    if get_bool("component_container"):
        description += [
            Node(
                package="rclcpp_components",
                executable="component_container_mt",
                name=COMPONENT_CONTAINER_NAME,
                arguments=["--ros-args", "--log-level", "warn"],
            )
        ]

    if get_bool("rviz"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_rviz")
                        / "launch"
                        / "rviz.launch.py"
                    )
                ),
                launch_arguments={
                    "config": get_str("rviz.config"),
                }.items(),
            ),
        ]

    if get_bool("description"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_description")
                        / "launch"
                        / "description.launch.py"
                    )
                ),
                launch_arguments={
                    "joint_state_publisher_gui": get_str(
                        "description.joint_state_publisher_gui"
                    ),
                }.items(),
            )
        ]

    if get_bool("unity_sim"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("unity_sim")
                        / "launch"
                        / "unity_sim.launch.py"
                    )
                ),
                launch_arguments={
                    "scene": get_str("unity_sim.scene"),
                }.items(),
            ),
        ]

    if get_bool("wheels"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_wheels")
                        / "launch"
                        / "wheels.launch.py"
                    )
                ),
                launch_arguments={
                    "joy": get_str("wheels.joy"),
                }.items(),
            ),
        ]

    if get_bool("gs"):
        description += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(get_package_share_path("kalman_gs") /
                        "launch" / "gs.launch.py")
                ),
            ),
        ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "component_container",
                default_value="false",
                description="Start up the main component container. Must be spawned in order to enable composition.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Launch RViz.",
            ),
            DeclareLaunchArgument(
                "rviz.config",
                default_value="",
                description="RViz configuration file.",
            ),
            DeclareLaunchArgument(
                "description",
                default_value="false",
                description="Start up state publishers for the robot description.",
            ),
            DeclareLaunchArgument(
                "description.joint_state_publisher_gui",
                default_value="false",
                description="Start up the joint state publisher with a GUI.",
            ),
            DeclareLaunchArgument(
                "unity_sim",
                default_value="false",
                description="Start up the Unity simulator with virtual sensors and actuators.",
            ),
            DeclareLaunchArgument(
                "unity_sim.scene",
                default_value="",
                description="The scene to load in Unity.",
            ),
            DeclareLaunchArgument(
                "wheels",
                default_value="false",
                description="Start up the wheel controller.",
            ),
            DeclareLaunchArgument(
                "wheels.joy",
                default_value="",
                description="Joy device to use for headless driving. Choose 'gamepad' or 'arduino'. Empty disables headless teleop.",
            ),
            DeclareLaunchArgument(
                "gs",
                default_value="false",
                description="Start up GS.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
