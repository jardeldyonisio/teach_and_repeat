#!/usr/bin/env python3

import sys
import time
import rclpy
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist, PoseWithCovarianceStamped

from copy_file import copy_file
from calculate_erro import calculate_erro
from compare_paths_two import compare_paths_two
from save_coords_to_file import save_coords_to_file
from tf_transformations import euler_from_quaternion
from save_variables_to_file import save_variables_to_file
from read_coords_from_file import read_points_from_file
from create_folder_with_datetime import create_folder_with_datetime

# TODO: Remover os pontos que foram atingidos
# Marcar em outra cor o atual ponto de objetivo
# Ajustar os caminhos de diretórios dos arquivos

class Navigator(Node):
    def __init__(self):
        super().__init__('repeat_path_coords')
        self.cmd_vel_pub = self.create_publisher(Twist, '/hoverboard_velocity_controller/cmd_vel', 10)
        self.position_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.odom_callback, 10)

        base_to_create_folder = "/home/lognav/lognav_ws/src/freedom_navigation/data/"
        path_folder_to_copy = "/home/lognav/lognav_ws/src/freedom_navigation/data/teleop_data.txt"

        self.folder_path = create_folder_with_datetime(base_to_create_folder)
        copy_file(path_folder_to_copy, self.folder_path)

        self.file_teleop_path = '/home/lognav/lognav_ws/src/freedom_navigation/data/teleop_data.txt'
        self.goals = read_points_from_file(self.file_teleop_path)

        self.coords_marker_pub = self.create_publisher(Marker, 'coords_marker', 10)

        self.coords_marker = Marker()
        self.coords_marker.header.frame_id = 'map'
        self.coords_marker.type = Marker.POINTS
        self.coords_marker.action = Marker.ADD
        self.coords_marker.pose.orientation.w = 1.0
        self.coords_marker.scale.x = 0.01
        self.coords_marker.scale.y = 0.1
        self.coords_marker.color.r = 1.0
        self.coords_marker.color.g = 0.0
        self.coords_marker.color.b = 0.0
        self.coords_marker.color.a = 1.0

        for p in self.goals:
            selected_point = Point()
            selected_point.x, selected_point.y = p[0], p[1]
            self.coords_marker.points.append(selected_point)

        self.file_following_path = self.folder_path + '/following_data.txt'

        self.coords_during_following = []
        
        self.t = 0.0

        self.tractor_velocity = 0.05
        self.threshold_dist = 0.6
        self.THRESHOLD_YAW = 0.4
        self.start_time = time.time()

    def odom_callback(self, data: PoseWithCovarianceStamped):
        msg = Twist()

        self.coords_marker_pub.publish(self.coords_marker)

        if len(self.goals) == 0:
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            self.cmd_vel_pub.publish(msg)
            return

        quaternion = data.pose.pose.orientation
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, self.yaw = euler_from_quaternion(quaternion_list)

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        point = Point()
        point.x = self.x
        point.y = self.y

        self.coords_during_following.append(point)

        self.pos = np.array([self.x, self.y])

        goal = self.goals[0]
        goal_vec = np.array(goal)
        self.dist_vec = goal_vec - self.pos
        self.dist = np.linalg.norm(self.dist_vec)

        self.yaw_d = np.arctan2(self.dist_vec[1], self.dist_vec[0])
        self.diff_yaw = self.yaw_d - self.yaw
        
        if self.diff_yaw > self.THRESHOLD_YAW:
            msg.linear.x = .0
            msg.angular.z = .1
        elif self.diff_yaw < -self.THRESHOLD_YAW:
            msg.linear.x = .0
            msg.angular.z = -.1
        else:
            msg.linear.x = .1
            msg.angular.z = .0

        self.cmd_vel_pub.publish(msg)

        if self.dist < self.threshold_dist:
            print("Atingiu o objetivo")
            self.goals = np.delete(self.goals, 0, 0)
            if len(self.goals) == 0:
                print("Objetivos concluídos")
                msg.angular.z = 0.0
                msg.linear.x = 0.0
                self.cmd_vel_pub.publish(msg)

                end_time = time.time()
                duration_time = (end_time - self.start_time) / 60

                variables_path = self.folder_path + '/variables.txt'

                save_coords_to_file(self.file_following_path, self.coords_during_following)
                erro_percentual = calculate_erro(self.file_teleop_path, self.file_following_path)

                self.variables = {
                    "self.tractor_velocity": self.tractor_velocity,
                    "threshold_dist": self.threshold_dist,
                    "duration_time": duration_time,
                    "erro_percentual": erro_percentual
                }
                save_variables_to_file(variables_path, **self.variables)
                compare_paths_two(self.file_teleop_path, self.file_following_path)

                sys.exit()

def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()