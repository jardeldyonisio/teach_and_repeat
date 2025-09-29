#!/usr/bin/env python3
#coding: utf-8

# Author: Jardel Dyonisio (https://github.com/jardeldyonisio)
# Official Repository: https://github.com/jardeldyonisio/teach_and_repeat

import os
import sys
import time
import rclpy
import numpy as np

from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist, PoseWithCovarianceStamped

# Add the bspline_smoother src directory to Python path
# Use absolute path to ensure it works when run via ROS2
bspline_path = "/home/jardeldyonisio/teach_repeat_ws/src/teach_and_repeat/bspline_smoother/src"
sys.path.insert(0, bspline_path)
from bspline_fit import BsplieFit

from copy_file import copy_file
from tf_transformations import euler_from_quaternion
from compare_bezier_lookahead import compare_bezier_lookahead
from create_folder_with_datetime import create_folder_with_datetime

from utils import getPointsFromFile, pointsToPoseStamped, posesToPath

# TODO: Variable linear velocity.
# TODO: Doxygen documentation.
# TODO: Add max distance from global path.
# TODO: Param "use_bezier".
# TODO: Max error tolerance on Bezier curve generation
# TODO: Automatic initial pose
# TODO: Add a flag to save some datas, by default this flag should be false

class LookaheadLocalPlanner(Node):
    '''
    Local Planner implementation
    '''
    def __init__(self):
        super().__init__('path_lookahead_window_planner')
        self.path_curve_marker_pub = self.create_publisher(Marker, 'path_curve_marker', 10)
        self.path_points_marker_pub = self.create_publisher(Marker, 'path_points_marker', 10)
        self.lookahead_paths_marker_pub = self.create_publisher(Marker, 'lookahead_paths_marker', 10)
        
        # TODO: Instead of creating a new marker to publish the local path, publish it on /local_plan topic
        self.selected_lookahead_path_marker_pub = self.create_publisher(Marker, 'selected_lookahead_path_marker', 10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.odom_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.callback_pose, 10)

        # Declare parameters
        self.declare_parameter('path_name', 'path')

        # Get parameters from launch file
        path_name = self.get_parameter('path_name').get_parameter_value().string_value

        # Frame ID. If you are using only relatyve data 
        # (IMU, Odometry, etc) you can use 'odom'.
        frame_id = 'map'

        # Robot params
        self.max_vel_x = 0.2
        self.max_vel_theta = 1.0
        self.min_vel_theta = -self.max_vel_theta
        self.max_distance_from_path = 1.0

        # threshold_dist btw robot and coord
        self.threshold_dist = 0.8

        # Future Behavior Parameters
        self.points_per_paths = 15
        self.dist_btw_points = 0.05
        self.lookahead_total_paths = 50

        # Simulation
        self.sim_steps = 30

        self.lookahead_updated = dict()
        self.robot_position = 0.0
        self.desired_steering_angle = 0.0
        
        # Cofiguração do marker dos paths do lookahead
        self.lookahead_paths_marker = Marker()
        self.lookahead_paths_marker.header.frame_id = frame_id
        self.lookahead_paths_marker.type = Marker.POINTS
        self.lookahead_paths_marker.action = Marker.ADD
        self.lookahead_paths_marker.pose.orientation.w = 1.0
        self.lookahead_paths_marker.scale.x = 0.01
        self.lookahead_paths_marker.scale.y = 0.01
        self.lookahead_paths_marker.color.r = 0.0
        self.lookahead_paths_marker.color.g = 0.0
        self.lookahead_paths_marker.color.b = 1.0
        self.lookahead_paths_marker.color.a = 1.0

        # Configuração do marker que mostra o path da curva de esterçamento selecionada
        self.selected_lookahead_path_marker = Marker()
        self.selected_lookahead_path_marker.header.frame_id = frame_id
        self.selected_lookahead_path_marker.type = Marker.LINE_LIST
        self.selected_lookahead_path_marker.action = Marker.ADD
        self.selected_lookahead_path_marker.pose.orientation.w = 1.0
        self.selected_lookahead_path_marker.scale.x = 0.01
        self.selected_lookahead_path_marker.scale.y = 0.1
        self.selected_lookahead_path_marker.color.r = 1.0
        self.selected_lookahead_path_marker.color.g = 1.0
        self.selected_lookahead_path_marker.color.b = 0.0
        self.selected_lookahead_path_marker.color.a = 1.0

        # Cofiguração do marker da curva de path
        # TODO: Add marker timer or timeout

        self.path_curve_marker = Marker()
        self.path_curve_marker.header.frame_id = frame_id
        self.path_curve_marker.type = Marker.LINE_STRIP
        self.path_curve_marker.action = Marker.ADD
        self.path_curve_marker.pose.orientation.w = 1.0
        self.path_curve_marker.scale.x = 0.01
        self.path_curve_marker.scale.y = 0.1
        self.path_curve_marker.color.r = 1.0
        self.path_curve_marker.color.g = 0.0
        self.path_curve_marker.color.b = 0.0
        self.path_curve_marker.color.a = 1.0

        # Cofiguração do marker que mostra os pontos na curva de path
        self.path_points_marker = Marker()
        self.path_points_marker.header.frame_id = frame_id
        self.path_points_marker.type = Marker.POINTS
        self.path_points_marker.action = Marker.ADD
        self.path_points_marker.pose.orientation.w = 1.0
        self.path_points_marker.scale.x = 0.05
        self.path_points_marker.scale.y = 0.05
        self.path_points_marker.color.r = 1.0
        self.path_points_marker.color.g = 1.0
        self.path_points_marker.color.b = 0.0
        self.path_points_marker.color.a = 1.0

        ##################################################################################
        # WARNING!!! This code should not be here, it is just to make it work
        # The correct way is get the path from the topic

        # When use path absolute the path is on install folder, so joint there
        # and go back to the root folder of the project
        ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
        paths = os.path.join(ws_dir, "src", "teach_and_repeat", "path_saves")
        self.file_teleop_path = os.path.join(paths, f"{path_name}.txt")
        self.path_points = getPointsFromFile(self.file_teleop_path)

        # Smooth the path
        path_smoothed = BsplieFit()
        path_smoothed.fit(data_points=self.path_points, 
                          adaptive=True)
        self.path_points = path_smoothed.getBezierPoints()

        ###################################################################################

        # Show the bspline
        self.showPath()

        # Gera lookahead
        self.futureBehavior()

        # Seta o valor inicial de algumas variáveis
        self.new_min = 0
        self.path_few_points = 0.0
        self.new_max = self.points_per_paths

        self.start_time = time.time()

        self.get_logger().info("Waiting for the 2D Pose Estimation")

    def updateReferencePathPoints():
        '''
        @brief 
        '''
        pass

    def generateLogs():
        '''
        @brief
        '''
        pass
        
    def setupLogs():
        '''
        @brief
        '''
        pass

    def callback_pose(self, msg : PoseWithCovarianceStamped):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.robot_position = [self.x, self.y]

        point = Point()
        point.x = self.x
        point.y = self.y

        self.quaternion = msg.pose.pose.orientation
        quaternion_list = [self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w]
        _, _, self.robot_yaw = euler_from_quaternion(quaternion_list)

        self.update()

    def update(self):
        '''
        Atualiza a posição do robô, seleciona o melhor path local e publica os markers.
        Usa diretamente v e omega, sem precisar da distância entre rodas.
        '''
        msg = Twist()

        self.showPath()

        min_cost = None

        # Atualiza os lookahead paths no frame global
        for steering_angle, points in self.generated_lookahead.items():
            points = np.array(points)

            # TODO: Use linalg norm
            d = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
            angle = np.arctan2(points[:, 1], points[:, 0])

            xs = self.x + d * np.cos(angle + self.robot_yaw)
            ys = self.y + d * np.sin(angle + self.robot_yaw)

            global_points = np.stack((xs, ys), axis=1)
            self.lookahead_updated[steering_angle] = global_points

        # Atualiza marker de todos os paths
        self.lookahead_paths_marker.points = []
        for points in self.lookahead_updated.values():
            for p in points:
                pt = Point()
                pt.x, pt.y = p[0], p[1]
                self.lookahead_paths_marker.points.append(pt)
        self.lookahead_paths_marker_pub.publish(self.lookahead_paths_marker)

        # Seleciona o melhor path local
        self.path_few_points = self.path_points[self.new_min:self.new_max]
        for angle, lookahead_points in self.lookahead_updated.items():
            cost = compare_bezier_lookahead(lookahead_points, self.path_few_points, self.points_per_paths)
            if min_cost is None or cost < min_cost:
                min_cost = cost
                self.desired_steering_angle = angle
                self.best_lookahead_path = lookahead_points

        # Publica comando direto com velocidade linear e angular
        msg.linear.x = self.max_vel_x
        msg.angular.z = self.desired_steering_angle
        self.cmd_vel_pub.publish(msg)

        # Atribui para a variável os pontos a frente da curva de Bézier
        for p in self.path_few_points:
            selected_point = Point()
            selected_point.x, selected_point.y = p[0], p[1]
            self.path_points_marker.points.append(selected_point)

        self.path_points_marker_pub.publish(self.path_points_marker)

        # Publica marker do path selecionado
        self.selected_lookahead_path_marker.points = []
        for p in self.best_lookahead_path:
            pt = Point()
            pt.x, pt.y = p[0], p[1]
            self.selected_lookahead_path_marker.points.append(pt)
        self.selected_lookahead_path_marker_pub.publish(self.selected_lookahead_path_marker)

        # Atualiza controle dos pontos da curva
        if np.any(self.path_few_points):
            dist_robot_from_point = np.linalg.norm(self.path_few_points[0] - self.robot_position)
            if dist_robot_from_point < self.threshold_dist:
                self.new_min += 1
                self.new_max += 1
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            sys.exit()
        self.lookahead_updated.clear()
        self.selected_lookahead_path_marker.points = []
        self.lookahead_paths_marker.points = []
        self.path_points_marker.points = []

    def updateReferencePoints(self):
        '''
        @brief Update reference points on the path based on robot's position.
        '''
        if np.any(self.path_few_points):
            dist_robot_from_point = np.linalg.norm(self.path_few_points[0] - self.robot_position)
            if dist_robot_from_point < self.threshold_dist:
                self.new_min += 1
                self.new_max += 1
        pass

    def futureBehavior(self):
        '''
        Gera os lookahead paths diretamente em coordenadas relativas do robô.
        Integra usando velocidade linear (v) e velocidade angular (omega).

        TODO: Add kinematics
        '''
        lookahead_dict = dict()

        # Define intervalo de velocidades angulares (omega)
        omegas = np.linspace(self.min_vel_theta, self.max_vel_theta, self.lookahead_total_paths)

        for omega in omegas:
            x, y, yaw = 0.0, 0.0, 0.0
            path_points = []

            for _ in range(self.points_per_paths):
                path_points.append((x, y))  # salva a posição atual

                for _ in range(self.sim_steps):
                    # distância percorrida em cada sub-passagem
                    v_sub = self.dist_btw_points / self.sim_steps

                    # integração da pose
                    x += v_sub * np.cos(yaw)
                    y += v_sub * np.sin(yaw)
                    yaw += omega * (self.dist_btw_points / self.max_vel_x) / self.sim_steps

            lookahead_dict[omega] = path_points

        self.generated_lookahead = lookahead_dict


    def showPath(self):
        # TODO: Publish on global plan topic
        for p in self.path_points:
            point = Point()
            point.x = p[0]
            point.y = p[1]
            self.path_curve_marker.points.append(point)

        self.path_curve_marker_pub.publish(self.path_curve_marker)

def main(args=None):
    rclpy.init(args=args)
    path_lookahead_window_planner = LookaheadLocalPlanner()
    rclpy.spin(path_lookahead_window_planner)
    path_lookahead_window_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()