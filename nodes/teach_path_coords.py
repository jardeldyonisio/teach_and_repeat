#!/usr/bin/env python3
#coding: utf-8

# Author: Jardel Dyonisio (https://github.com/jardeldyonisio)

import os
import rclpy
import threading
from utils import saveOrientedCoordsToFile

from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from save_coords_to_file import save_coords_to_file
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
# from std_srvs.srv import Trigger  # Usaremos um serviço Trigger para salvar o caminho
from teach_and_repeat.srv import SavePath

# TODO: Reset the path_coords after calling the service
# TODO: Add a keyboard to start recording the path

class OrientedPoint:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

class TeachPathCoords(Node):
    '''
    @class TeachPathCoords

    @brief A ROS2 node that subscribes to the a topic and records the coordinates of the robot's position.
    '''

    def __init__(self):
        super().__init__('teach_path_coords')

        self.marker_pub = self.create_publisher(Marker, '/teach_and_repeat/teach/path_marker', 10)

        # Declare parameters
        self.declare_parameter('reference_frame', 'map')
        self.declare_parameter('teach_orientation', True)

        # Get parameters from launch file
        reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value
        self.teach_orientation = self.get_parameter('teach_orientation').get_parameter_value().bool_value

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
            self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.callback, 10)
            self.topic_msg = PoseWithCovarianceStamped()
        elif reference_frame == 'odom':
            self.pose_sub = self.create_subscription(Odometry, 'odom', self.callback, 10)
            self.topic_msg = Odometry()
        else:
            self.get_logger().error("Invalid reference frame. Please choose 'map' or 'odom'.")
            self.destroy_node()
            rclpy.try_shutdown()

        # Create the save path service
        self.srv = self.create_service(SavePath, 'save_path', self.save_path_callback)

        self.path_coords = []
        self.marker.points = []
        self.recording = False

        self.get_logger().info("Press ENTER to start recording the path...")
        threading.Thread(target=self.wait_for_keypress, daemon=True).start()

    def wait_for_keypress(self):
        input()
        self.recording = True
        self.get_logger().info("Recording started. Saving coordinates...")
        self.get_logger().info("Publishing markers to '/coords_marker'")
        self.get_logger().info("Call the '/save_path' service to save the coordinates.")
        self.get_logger().info("Press CTRL + C to shut down.")

    def callback(self, msg):
        self.topic_msg = msg
        if self.recording:
            self.handling_path_coords()

    def handling_path_coords(self):
        if self.teach_orientation == False:
            point = Point()
            x = self.topic_msg.pose.pose.position.x
            y = self.topic_msg.pose.pose.position.y

            point.x = x
            point.y = y
            
            self.path_coords.append(point)
            self.marker_publisher(point)
        else:
            x = self.topic_msg.pose.pose.position.x
            y = self.topic_msg.pose.pose.position.y
            yaw = self.topic_msg.pose.pose.orientation.z

            marker_point = Point()
            marker_point.x = x
            marker_point.y = y
            point = OrientedPoint(x, y, yaw)

            self.path_coords.append(point)
            self.marker_publisher(marker_point)

    def marker_publisher(self, point):
        self.marker.points.append(point)
        self.marker_pub.publish(self.marker)

    def save_path_callback(self, request, response):
        try:
            if request.path_name != "":
                self.path_name = request.path_name
            else:
                self.path_name = 'path_coords'
            ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
            self.file_path = os.path.join(ws_dir, "src", "teach_and_repeat", "path_saves", f"{self.path_name}.txt")
            if self.teach_orientation == True:
                saveOrientedCoordsToFile(self.file_path, self.path_coords)
            else:
                save_coords_to_file(self.file_path, self.path_coords)
            request.path_name = self.path_name
            response.success = True
            response.message = f"Path named {self.path_name} saved successfully on {self.file_path}."
        except Exception as e:
            self.get_logger().error(f"Failed to save path: {str(e)}")
            response.success = False
            response.message = f"Error: {str(e)}"
        return response

def main(args=None):
    rclpy.init(args=args)
    path_coords = TeachPathCoords()

    try:
        rclpy.spin(path_coords)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Shutting down without saving...")
    finally:
        path_coords.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()