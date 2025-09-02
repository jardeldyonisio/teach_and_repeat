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
from geometry_msgs.msg import Point, Twist, PoseWithCovarianceStamped, PoseStamped

from copy_file import copy_file
from compare_paths import compare_paths
from points_at_interval import points_at_interval
from save_coords_to_file import save_coords_to_file
from tf_transformations import euler_from_quaternion
from generate_bezier_curve import generate_bezier_curve
from save_variables_to_file import save_variables_to_file
from compare_bezier_lookahead import compare_bezier_lookahead
from create_folder_with_datetime import create_folder_with_datetime

from utils import getPointsFromFile, pointsToPoseStamped, posesToPath

# TODO: Variable linear velocity.
# TODO: Doxygen documentation.
# TODO: Remove "distance_btw_wheels".
# TODO: "robot" instead of "tractor".
# TODO: Add max distance from global path.
# TODO: Param "use_bezier".
# TODO: Max error tolerance on Bezier curve generation
# TODO: Automatic initial pose

class BezierLookaheadWindowPlanner(Node):
    '''
    Path following behavior using Bézier curves.
    '''
    def __init__(self):
        super().__init__('bezier_lookahead_window_planner')
        self.bezier_curve_marker_pub = self.create_publisher(Marker, 'bezier_curve_marker', 10)
        self.bezier_points_marker_pub = self.create_publisher(Marker, 'bezier_points_marker', 10)
        self.lookahead_paths_marker_pub = self.create_publisher(Marker, 'lookahead_paths_marker', 10)
        self.selected_lookahead_path_marker_pub = self.create_publisher(Marker, 'selected_lookahead_path_marker', 10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.odom_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.callback_pose, 10)

        # Frame ID. If you are using only relatyve data 
        # (IMU, Odometry, etc) you can use 'odom'.
        frame_id = 'map'

        # Robot params
        self.max_vel_x = 0.2
        self.max_vel_theta = 1.0
        self.min_vel_theta = -self.max_vel_theta

        # threshold_dist btw tractor and coord
        self.threshold_dist = 0.8

        # Future Behavior Parameters
        self.points_per_paths = 15
        self.dist_btw_points = 0.05
        self.lookahead_total_paths = 50

        # Simulation
        self.sim_steps = 30

        self.lookahead_updated = dict()
        self.tractor_position = 0.0
        self.desired_steering_angle = 0.0
        
        # Follow curves
        self.coords_during_following = []

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
        self.selected_lookahead_path_marker.type = Marker.LINE_STRIP
        self.selected_lookahead_path_marker.action = Marker.ADD
        self.selected_lookahead_path_marker.pose.orientation.w = 1.0
        self.selected_lookahead_path_marker.scale.x = 0.01
        self.selected_lookahead_path_marker.scale.y = 0.1
        self.selected_lookahead_path_marker.color.r = 1.0
        self.selected_lookahead_path_marker.color.g = 1.0
        self.selected_lookahead_path_marker.color.b = 0.0
        self.selected_lookahead_path_marker.color.a = 1.0

        # Cofiguração do marker da curva de bezier
        # TODO: Add marker timer or timeout

        self.bezier_curve_marker = Marker()
        self.bezier_curve_marker.header.frame_id = frame_id
        self.bezier_curve_marker.type = Marker.LINE_STRIP
        self.bezier_curve_marker.action = Marker.ADD
        self.bezier_curve_marker.pose.orientation.w = 1.0
        self.bezier_curve_marker.scale.x = 0.01
        self.bezier_curve_marker.scale.y = 0.1
        self.bezier_curve_marker.color.r = 1.0
        self.bezier_curve_marker.color.g = 0.0
        self.bezier_curve_marker.color.b = 0.0
        self.bezier_curve_marker.color.a = 1.0

        # Cofiguração do marker que mostra os pontos na curva de bezier
        self.bezier_points_marker = Marker()
        self.bezier_points_marker.header.frame_id = frame_id
        self.bezier_points_marker.type = Marker.POINTS
        self.bezier_points_marker.action = Marker.ADD
        self.bezier_points_marker.pose.orientation.w = 1.0
        self.bezier_points_marker.scale.x = 0.05
        self.bezier_points_marker.scale.y = 0.05
        self.bezier_points_marker.color.r = 1.0
        self.bezier_points_marker.color.g = 1.0
        self.bezier_points_marker.color.b = 0.0
        self.bezier_points_marker.color.a = 1.0

        # Cria a pasta com data e horário e copia os arquivos
        # necessários para essa pasta

        # When use path absolute the path is on install folder, so joint there
        # and go back to the root folder of the project
        ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
        package_src_path = os.path.join(ws_dir, "src", "teach_and_repeat")
        base_to_create_folder = os.path.join(package_src_path, "data/")

        path_folder_to_copy = os.path.join(base_to_create_folder, "path.txt")
        self.folder_path = create_folder_with_datetime(base_to_create_folder)
        copy_file(path_folder_to_copy, self.folder_path)

        # Coleta dados de posição de quando o veiculo
        # foi teleoperado.
        self.file_teleop_path = os.path.join(base_to_create_folder, "path.txt")
        # teleop_path_points = read_points_from_file(self.file_teleop_path)
        points = getPointsFromFile(self.file_teleop_path)
        poses_stamped = pointsToPoseStamped(points)
        # path = posesToPath(poses_stamped)
        self.bezier_path_coords = points

        # Mostra a curva de Bézier
        self.showPath()

        # Gera lookahead
        self.futureBehavior()

        # Seta o valor inicial de algumas variáveis
        self.new_min = 0
        self.bezier_few_points = 0.0
        self.new_max = self.points_per_paths

        self.start_time = time.time()

        self.get_logger().info("Waiting for the 2D Pose Estimation")

    def generateLogs():
        pass
        
    def setupLogs():
        pass

    def callback_pose(self, msg : PoseWithCovarianceStamped):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.tractor_position = [self.x, self.y]

        point = Point()
        point.x = self.x
        point.y = self.y

        self.quaternion = msg.pose.pose.orientation
        quaternion_list = [self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w]
        _, _, self.tractor_yaw = euler_from_quaternion(quaternion_list)

        self.coords_during_following.append(point)

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

            xs = self.x + d * np.cos(angle + self.tractor_yaw)
            ys = self.y + d * np.sin(angle + self.tractor_yaw)

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
        self.bezier_few_points = self.bezier_path_coords[self.new_min:self.new_max]
        for angle, lookahead_points in self.lookahead_updated.items():
            cost = compare_bezier_lookahead(lookahead_points, self.bezier_few_points, self.points_per_paths)
            if min_cost is None or cost < min_cost:
                min_cost = cost
                self.desired_steering_angle = angle
                self.best_lookahead_path = lookahead_points

        # Publica comando direto com velocidade linear e angular
        msg.linear.x = self.max_vel_x
        msg.angular.z = self.desired_steering_angle
        self.cmd_vel_pub.publish(msg)

        # Publica marker do path selecionado
        self.selected_lookahead_path_marker.points = []
        for p in self.best_lookahead_path:
            pt = Point()
            pt.x, pt.y = p[0], p[1]
            self.selected_lookahead_path_marker.points.append(pt)
        self.selected_lookahead_path_marker_pub.publish(self.selected_lookahead_path_marker)

        # Atualiza controle dos pontos da curva
        if np.any(self.bezier_few_points):
            dist_tractor_from_point = np.linalg.norm(self.bezier_few_points[0] - self.tractor_position)
            if dist_tractor_from_point < self.threshold_dist:
                self.new_min += 1
                self.new_max += 1
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            sys.exit()

    def futureBehavior(self):
        '''
        Gera os lookahead paths diretamente em coordenadas relativas do robô.
        Integra usando velocidade linear (v) e velocidade angular (omega).
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
        for p in self.bezier_path_coords:
            point = Point()
            point.x = p[0]
            point.y = p[1]
            self.bezier_curve_marker.points.append(point)

        self.bezier_curve_marker_pub.publish(self.bezier_curve_marker)

def main(args=None):
    rclpy.init(args=args)
    bezier_lookahead_window_planner = BezierLookaheadWindowPlanner()
    rclpy.spin(bezier_lookahead_window_planner)
    bezier_lookahead_window_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()