from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="gs_rosbridge_websocket",
                parameters=[
                    {
                        "port": 9065,
                        "send_action_goals_in_new_thread": True,
                    }
                ],
                ros_arguments=["--ros-args", "--log-level", "fatal"],
                # Bleeding edge rosbridge_server emits a lot of errors when actions are used. It works nevertheless.
            ),
            Node(
                package="distobee_gs",
                executable="gs",
            ),
        ]
    )
