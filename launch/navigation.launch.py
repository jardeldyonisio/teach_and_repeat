import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('freedom_navigation'), 'param')
    rviz_config_dir = os.path.join(get_package_share_directory('freedom_navigation'), 'rviz/mapping.rviz')
    param_file = os.path.join(config_dir, 'tb3_nav2_params.yaml')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('freedom_navigation'), 'launch', 'base.launch.py')
        ),
    )

    map_file = PathJoinSubstitution(
        [FindPackageShare("freedom_navigation"), "maps", "my_map.yaml"]
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'map': map_file,
            'params_file': param_file
        }.items()
    )

    return LaunchDescription([
        robot_launch,
        nav2_bringup_launch,
       
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', rviz_config_dir],
            output='screen'),
    ])

