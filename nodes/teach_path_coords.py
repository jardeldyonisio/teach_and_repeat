#!/usr/bin/env python3
#coding: utf-8

# Author: Jardel Dyonisio (https://github.com/jardeldyonisio)
# Official Repository: https://github.com/jardeldyonisio/teach_and_repeat

import os
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from save_coords_to_file import save_coords_to_file
from geometry_msgs.msg import Point, PoseWithCovarianceStamped

# TODO: Other way to finish the node.
# TODO: Implement a way to save the path and the map name to be easier to know wich map and path was used during the teach phase.
# TODO: Create a folder to each path, saving the path coords, wich map was used and a image of the path in the map. This is to
#       make easier to know wich path and wich map is in the folder.
# TODO: A special marker to show the start and the end of the path.
# TODO: Verify if the path name already exists, if yes and it is empty , ask if the user wants to overwrite the file or create a new one.

class TeachPathCoords(Node):
    '''
    @class TeachPathCoords

    @brief A ROS2 node that subscribes to the a topic and records the coordinates of the robot's position.
    '''

    def __init__(self):
        super().__init__('teach_path_coords')

        self.marker_pub = self.create_publisher(Marker, '/coords_marker', 10)

        # Declare parameters
        self.declare_parameter('reference_frame', 'map')
        self.declare_parameter('path_name', 'path_coords')

        # Get parameters from launch file
        reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value
        self.path_name = self.get_parameter('path_name').get_parameter_value().string_value

        # Configure the marker
        self.marker = Marker()
        self.marker.header.frame_id = reference_frame
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.01
        self.marker.scale.y = 0.1
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        if reference_frame == 'map':
            self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.callback, 10)
            self.topic_msg = PoseWithCovarianceStamped()
        elif reference_frame == 'odom':
            self.pose_sub = self.create_subscription(Odometry, '/odom', self.callback, 10)
            self.topic_msg = Odometry()
        else:
            # Some errors are happening here. The node is not shutting down properly.
            self.get_logger().error("Invalid reference frame. Please choose 'map' or 'odom'.")
            self.destroy_node()
            rclpy.try_shutdown()

        self.path_coords = []
        self.marker.points = []
        ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
        self.file_path = os.path.join(ws_dir, "src", "teach_and_repeat", "path_saves", f"{self.path_name}.txt")

        self.get_logger().info("Path coordinates saver node initialized. Saving coordinates...")
        self.get_logger().info("Publishing markers to '/coords_marker'")
        self.get_logger().info("Press CTRL + C to save path coords. Data will be saved as a .txt file to your current directory.")

    def callback(self, msg):
        self.topic_msg = msg

        # Maybe it's better create a timer to call the handling_path_coords function.
        self.handling_path_coords()

    def handling_path_coords(self):
        point = Point()

        x = self.topic_msg.pose.pose.position.x
        y = self.topic_msg.pose.pose.position.y

        point.x = x
        point.y = y
        
        self.path_coords.append(point)
        self.marker_publisher(point)

    def marker_publisher(self, point):
        self.marker.points.append(point)
        self.marker_pub.publish(self.marker)
        
def main(args=None):
    rclpy.init(args=args)
    path_coords = TeachPathCoords()

    try:
        rclpy.spin(path_coords)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Saving path and shutting down...")
    finally:
        save_coords_to_file(path_coords.file_path, path_coords.path_coords)
        print(f"\nPath saved: {path_coords.path_name}.")
        path_coords.destroy_node()
        
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
