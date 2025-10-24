from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Main camera feed publisher
        Node(
            package="distobee_cleverhive",
            executable="feed_publisher",
            parameters=[{
                "port": 5001,
            }],
            remappings=[
                ("image_raw", "/cleverhive/camera0/image_raw"),
            ],
        ),
        # cmd_vel to odometry integrator
        Node(
            package="distobee_cleverhive",
            executable="cmd_vel_odometry",
            remappings=[
                ("odometry", "cleverhive/odometry"),
            ],
        ),
    ])
