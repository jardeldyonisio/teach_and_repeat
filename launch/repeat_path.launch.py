import os

import launch_ros

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Caminho para o diretório dos arquivos de launch
    package_dir_navigation = get_package_share_directory('freedom_navigation')
    package_dir_hover = get_package_share_directory('hoverboard_driver')

    launch_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir_navigation, 'launch', 'base_description.launch.py')
        )
    )

    launch_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir_hover, 'launch', 'diffbot.launch.py')
        )
    )

    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', os.path.join(package_dir_navigation, 'rviz', 'teach_and_repeat.rviz')],
        output='screen'
    )

    return LaunchDescription([
        launch_1,
        launch_2,
        rviz,
    ])
