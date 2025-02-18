#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Point, PoseWithCovarianceStamped

from save_coords_to_file import save_coords_to_file

class TeachPathCoords(Node):

    def __init__(self):
        super().__init__('teach_path_coords')

        self.sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.callback, 10)

        self.teleop_points = []
        self.file_path = '/home/freedom/lognav_ws/src/lognav/teach_and_repeat/data/teleop_data.txt'

        print("Recording... Press CTRL + C to save coords.")

    def callback(self, msg : PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        point = Point()
        point.x = x
        point.y = y
        self.teleop_points.append(point)

def main(args=None):
    rclpy.init(args=args)

    try:
        teach_path_coords = TeachPathCoords()
        rclpy.spin(teach_path_coords)
    finally:
        save_coords_to_file(teach_path_coords.file_path, teach_path_coords.teleop_points)
        teach_path_coords.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
