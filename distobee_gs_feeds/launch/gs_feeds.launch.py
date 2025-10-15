from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Main camera feed preview
        Node(
            package="distobee_gs_feeds",
            executable="feed_preview",
            name="feed_preview_main",
            parameters=[{
                "port": 5001,
            }],
            output="screen"
        ),
        # Alt camera feed preview
        Node(
            package="distobee_gs_feeds",
            executable="feed_preview",
            name="feed_preview_alt",
            parameters=[{
                "port": 5002,
            }],
            output="screen"
        ),
        # Sifter camera feed preview
        Node(
            package="distobee_gs_feeds",
            executable="feed_preview",
            name="feed_preview_sifter",
            parameters=[{
                "port": 5003,
            }],
            output="screen"
        ),
        
        # # Main camera feed publisher
        # Node(
        #     package="distobee_gs_feeds",
        #     executable="feed_publisher",
        #     name="feed_publisher_main",
        #     parameters=[{
        #         "port": 5001,
        #     }],
        #     remappings=[
        #         ("image_raw", "feed/distobee_main/image_raw"),
        #     ],
        #     output="screen"
        # ),
        # # Alt camera feed publisher
        # Node(
        #     package="distobee_gs_feeds",
        #     executable="feed_publisher",
        #     name="feed_publisher_alt",
        #     parameters=[{
        #         "port": 5002,
        #     }],
        #     remappings=[
        #         ("image_raw", "feed/distobee_alt/image_raw"),
        #     ],
        #     output="screen"
        # ),
        # # Sifter camera feed publisher
        # Node(
        #     package="distobee_gs_feeds",
        #     executable="feed_publisher",
        #     name="feed_publisher_sifter",
        #     parameters=[{
        #         "port": 5003,
        #     }],
        #     remappings=[
        #         ("image_raw", "feed/sifter/image_raw"),
        #     ],
        #     output="screen"
        # ),
    ])
