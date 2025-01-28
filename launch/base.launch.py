import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('logistic_description'), 'launch', 'logistic_description.launch.py')
        ),
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sensors_description'), 'launch', 'velodyne.launch.py')
        ),
    )

    return LaunchDescription([
        robot_launch,
        velodyne_launch,
    ])