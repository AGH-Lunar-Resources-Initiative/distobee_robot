from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def launch_setup(context):
    description = []

    description += [
        Node(
            package='distobee_wheels',
            executable='gamepad_driving',
            parameters=[{
                'max_steering_angle': 30.0,
                'max_speed': 0.15
            }],
        )
    ]
    description += [
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        )
    ]

    return description

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])