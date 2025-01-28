import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('freedom_navigation'), 'config')
    rviz_config_dir = os.path.join(get_package_share_directory('freedom_navigation'), 'rviz/mapping.rviz')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('freedom_navigation'), 'launch', 'base.launch.py')
        ),
    )

    return LaunchDescription([
        robot_launch,

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                os.path.join(config_dir, 'slam_toolbox.yaml')
            }],
            remappings=[('odom', '/odometry/filtered')],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', rviz_config_dir],
            output='screen'),
    ])