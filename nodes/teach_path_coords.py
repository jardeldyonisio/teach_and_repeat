#!/usr/bin/env python3
#coding: utf-8

# Author: Jardel Dyonisio (https://github.com/jardeldyonisio)

import os
import json
import rclpy
import threading
import sys
import select
from utils import saveOrientedCoordsToFile

from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from save_coords_to_file import save_coords_to_file
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
# from std_srvs.srv import Trigger  # We would use a Trigger service to save the path
from teach_and_repeat.srv import SavePath

class OrientedPoint:
    def __init__(self, 
                 x: float = 0.0, 
                 y: float = 0.0, 
                 yaw: float = 0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

class NamedDock:
    def __init__(self, 
                 name="", 
                 x: float = 0.0, 
                 y: float = 0.0, 
                 yaw: float = 0.0):
        self.name = name
        self.x = x
        self.y = y
        self.yaw = yaw
    
    def to_dict(self):
        return {
            'name': self.name,
            'x': self.x,
            'y': self.y,
            'yaw': self.yaw
        }

class TeachPathCoords(Node):
    '''
    @class TeachPathCoords

    @brief A ROS2 node that subscribes to the a topic and records the coordinates of the robot's position.
    '''

    def __init__(self):
        super().__init__('teach_path_coords')

        self.marker_pub = self.create_publisher(Marker, '/teach_and_repeat/teach/path_marker', 10)
        self.dock_marker_pub = self.create_publisher(Marker, '/teach_and_repeat/teach/dock_markers', 10)

        # Declare parameters
        self.declare_parameter('reference_frame', 'map')
        self.declare_parameter('teach_orientation', True)

        # Get parameters from launch file
        reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value
        self.teach_orientation = self.get_parameter('teach_orientation').get_parameter_value().bool_value

        # Configure the path marker (red line)
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
        
        # Configure the dock markers (blue spheres with text)
        self.dock_markers = Marker()
        self.dock_markers.header.frame_id = reference_frame
        self.dock_markers.type = Marker.SPHERE_LIST
        self.dock_markers.action = Marker.ADD
        self.dock_markers.pose.orientation.w = 1.0
        self.dock_markers.scale.x = 0.3  # Sphere size
        self.dock_markers.scale.y = 0.3
        self.dock_markers.scale.z = 0.3
        self.dock_markers.color.r = 0.0
        self.dock_markers.color.g = 0.3
        self.dock_markers.color.b = 1.0  # Bright blue
        self.dock_markers.color.a = 1.0
        self.dock_markers.points = []
        
        # Configure text markers for dock names
        self.dock_text_markers = []
        
        # Base ID for text markers
        self.dock_marker_id = 1000  

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
        self.srv = self.create_service(SavePath, 'teach_and_repeat/teach/save_path', self.save_path_callback)
        
        # Create the save docks service
        self.srv_docks = self.create_service(SavePath, 'teach_and_repeat/teach/save_docks', self.save_docks_callback)

        self.path_coords = []
        self.docks = []  # List to store named docks
        self.marker.points = []
        self.recording = False

        self.get_logger().info("Press ENTER to start recording the path...")
        self.get_logger().info("Press 'D' + ENTER to register a dock at current position...")
        threading.Thread(target=self.wait_for_keypress, daemon=True).start()

    def wait_for_keypress(self):
        while rclpy.ok():
            try:
                user_input = input().strip().upper()
                if user_input == "":
                    if not self.recording:
                        self.recording = True
                        self.get_logger().info("Recording started. Saving coordinates...")
                        self.get_logger().info("Publishing markers to '/coords_marker'")
                        self.get_logger().info("Call the '/teach_and_repeat/teach/save_path' service to save the coordinates.")
                        self.get_logger().info("Call the '/teach_and_repeat/teach/save_docks' service to save the docks.")
                        self.get_logger().info("Press CTRL + C to shut down.")
                elif user_input == "D":
                    self.register_dock()
            except (EOFError, KeyboardInterrupt):
                break

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
    
    def register_dock(self):
        '''
        @brief Registers a new dock at the current robot position
        '''
        try:
            dock_name = input("Enter the dock name: ").strip()
            if dock_name:
                x = self.topic_msg.pose.pose.position.x
                y = self.topic_msg.pose.pose.position.y
                yaw = self.topic_msg.pose.pose.orientation.z if self.teach_orientation else 0.0
                
                dock = NamedDock(dock_name, x, y, yaw)
                self.docks.append(dock)
                
                # Add visual dock point
                dock_point = Point()
                dock_point.x = x
                dock_point.y = y
                dock_point.z = 0.0
                self.dock_markers.points.append(dock_point)
                
                # Create text marker for dock name
                self.create_dock_text_marker(dock_name, x, y)
                
                # Publish updated markers
                self.publish_dock_markers()
                
                self.get_logger().info(f"Dock '{dock_name}' registered at position ({x:.2f}, {y:.2f}, {yaw:.2f})")
                self.get_logger().info(f"Total registered docks: {len(self.docks)}")
            else:
                self.get_logger().warn("Dock name cannot be empty!")
        except (EOFError, KeyboardInterrupt):
            self.get_logger().info("Dock registration cancelled.")
    
    def create_dock_text_marker(self, dock_name, x, y):
        '''
        @brief Creates a text marker for the dock name

        @param dock_name: The name of the dock
        @param x: The x coordinate of the dock
        @param y: The y coordinate of the dock
        '''
        text_marker = Marker()
        text_marker.header.frame_id = self.dock_markers.header.frame_id
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "dock_names"
        text_marker.id = self.dock_marker_id
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        text_marker.pose.position.x = x
        text_marker.pose.position.y = y
        text_marker.pose.position.z = 0.5  # Text height
        text_marker.pose.orientation.w = 1.0
        
        text_marker.scale.z = 0.2  # Text size
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0  # White text
        text_marker.color.a = 1.0
        
        text_marker.text = dock_name
        
        self.dock_text_markers.append(text_marker)
        self.dock_marker_id += 1
    
    def publish_dock_markers(self):
        '''
        @brief Publishes all dock markers
        '''
        # Publish dock spheres
        self.dock_markers.header.stamp = self.get_clock().now().to_msg()
        self.dock_marker_pub.publish(self.dock_markers)
        
        # Publish dock text
        for text_marker in self.dock_text_markers:
            text_marker.header.stamp = self.get_clock().now().to_msg()
            self.dock_marker_pub.publish(text_marker)

    def save_path_callback(self, request, response):
        '''
        @brief Callback to save the recorded path to a file
        '''
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
    
    def save_docks_callback(self, request, response):
        '''
        @brief Callback to save registered docks to JSON file
        '''
        try:
            if len(self.docks) == 0:
                response.success = False
                response.message = "No docks have been registered yet."
                return response
                
            if request.path_name != "":
                docks_filename = request.path_name
            else:
                docks_filename = 'docks'
                
            ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
            docks_file_path = os.path.join(ws_dir, "src", "teach_and_repeat", "path_saves", f"{docks_filename}.json")
            
            # Convert docks to dictionary
            docks_data = {
                'docks': [dock.to_dict() for dock in self.docks],
                'total_docks': len(self.docks),
                'reference_frame': self.get_parameter('reference_frame').get_parameter_value().string_value,
                'teach_orientation': self.teach_orientation
            }
            
            # Save to JSON file
            with open(docks_file_path, 'w', encoding='utf-8') as f:
                json.dump(docks_data, f, indent=2, ensure_ascii=False)
            
            response.success = True
            response.message = f"Docks saved successfully to {docks_file_path}. Total: {len(self.docks)} docks."
            self.get_logger().info(response.message)
            
        except Exception as e:
            self.get_logger().error(f"Failed to save docks: {str(e)}")
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