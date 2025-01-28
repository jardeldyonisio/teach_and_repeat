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
    package_dir_navigation = get_package_share_directory('logistic_description')
    rviz_config_dir = os.path.join(get_package_share_directory('freedom_navigation'), 'rviz/mapping.rviz')
    package_dir_hokuyo = get_package_share_directory('sensors_description')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir_navigation, 'launch', 'logistic_description.launch.py')
        
        ),
        launch_arguments={
            'gui': 'false',
        }.items()
    )

    urg_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir_hokuyo, 'launch', 'urg_node.launch.py')
        
        ),
        launch_arguments={
            'use_rviz': 'false',
        }.items()
    )

    return LaunchDescription([

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',  # Use 'sync_slam_toolbox_node' if you want synchronous SLAM
            name='slam_toolbox',
            output='screen',
            parameters=[{
                os.path.join(config_dir, 'slam_toolbox.yaml')
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', rviz_config_dir],
            output='screen'),

        robot_launch,
        urg_node_launch,
            
    ])