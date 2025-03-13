#!/usr/bin/env python3
#coding: utf-8

import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# TODO: Define map directory.
# TODO: Define the rviz configuration file.

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "path_name",
            default_value="path_coords",
            description="Name of the path to be saved."
        ),
        DeclareLaunchArgument(
            "map_file",
            default_value=os.path.join(get_package_share_directory("teach_and_repeat"), 'map', 'map.yaml'),
            description="Path for the map file."
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="map",
            description="Reference frame. The options are 'map' or 'odom'."
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                get_package_share_directory('teach_and_repeat'),
                'rviz',
                'teach.rviz'),
            description='Full path to the RViz config file'
        ),
    ]

    # Initialize Arguments
    path_name = LaunchConfiguration("path_name")
    map_file = LaunchConfiguration("map_file")
    frame_id = LaunchConfiguration("frame_id")
    rviz_config = LaunchConfiguration("rviz_config")

    print("rviz_config: ", rviz_config)

    turtlebot3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("turtlebot3_gazebo"), 'launch', 'turtlebot3_world.launch.py')
        ),
    )

    # # Param to not load rviz.
    # turtlebot3_navigation = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory("turtlebot3_navigation2"), 'launch', 'navigation2.launch.py')
        
    #     ),
    #     # launch_arguments={
    #     #     'rviz_config': rviz_config,
    #     # }.items()
    # )

    # teach_node = Node(
    #    package="teach_and_repeat",
    #    executable="teach_path_coords.py",
    #    name="teach_path_coords",
    #    parameters=[{
    #        "path_name": path_name,
    #        "reference_frame": frame_id,
    #    }],
    # )

    nodes = [
        turtlebot3_world,
        # turtlebot3_navigation,
        # teach_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
