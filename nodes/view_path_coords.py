#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class ViewCoords(Node):

    def __init__(self):
        super().__init__('view_path_coords')

        self.marker = Marker()
        self.marker.header.frame_id = "odom"
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.05
        self.marker.scale.y = 0.05
        self.marker.color.r = 1.0
        self.marker.color.a = 1.0

        with open('./src/freedom_navigation/data/teleop_data.txt', 'r') as f:
            for line in f:
                ponto = Point()
                x, y = line.strip().split(',')
                ponto.x = float(x)
                ponto.y = float(y)
                self.marker.points.append(ponto)

        self.marker_publisher = self.create_publisher(Marker, '/view_path_coords_marker', 10)

    def publish_marker(self):
        while self.get_clock().now().to_msg().sec > 0:
            self.marker.header.stamp = self.get_clock().now().to_msg()
            self.marker_publisher.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    view_coords = ViewCoords()
    view_coords.publish_marker()
    rclpy.spin(view_coords)
    view_coords.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
