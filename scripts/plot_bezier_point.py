#!/usr/bin/env python3

import rclpy
import bezier
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from generate_bezier_curve import generate_bezier_curve

class PlotBezierPoint(Node):

    def __init__(self):
        super().__init__('draw_bezier_point')

        self.marker_pub = self.create_publisher(Marker, 'bezier_follow', 10)

        new_zero = 0
        self.num_points = 5
        self.dist_btw_points = 0.01 
        num_points_referee = self.num_points

        ctrl_pts = np.array([[-0.02780346, 0.03870414, 0.1697755, 0.2306312, 0.13624863, 0.04603491, -0.12468106, -0.20554966, -0.30316212, -0.30588269, -0.16874089, 0.01911827, 0.34452962, 0.48584107, 0.69099792, 1.03860146, 0.8180265, 0.81792381],
                             [-3.207116, -3.46167873, -4.03901895, -4.37807472, -4.96151693, -5.20991119, -5.66865206, -5.88474106, -6.41146359, -6.72947627, -7.43814356, -7.81679074, -8.60643786, -9.02458216, -9.79244157, -10.77088, -10.49220429, -10.49211049]])

        points = generate_bezier_curve(ctrl_pts, self.num_points)

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        for p in points[new_zero:num_points_referee]:
            selected_point = Point()
            selected_point.x, selected_point.y = p[0], p[1]
            marker.points.append(selected_point)

            if len(marker.points) > self.num_points:
                marker.points = marker.points[-5:]
                num_points_referee += 1
                new_zero += 1

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    plot_bezier_point = PlotBezierPoint()
    rclpy.spin(plot_bezier_point)
    plot_bezier_point.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()