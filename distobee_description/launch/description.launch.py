from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


urdf_path = get_package_share_path(
    'distobee_description').joinpath('urdf', 'description.urdf.xacro')
urdf = xacro.process(urdf_path)


def launch_setup(context):
    description = []

    description += [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf},
                        {"ignore_timestamp": True}],
        ),
    ]
    # Only to visualize the robot model in RViz.
    # description += [
    #     Node(
    #         package='joint_state_publisher_gui',
    #         executable='joint_state_publisher_gui',
    #         parameters=[{'source_list': ['/robot_description']}],
    #     )
    # ]
    # RVIz
    description += [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                "-d", str(get_package_share_path('distobee_description') / "rviz" / "view.rviz")],
        ),
    ]

    return description


def generate_launch_description():

    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
