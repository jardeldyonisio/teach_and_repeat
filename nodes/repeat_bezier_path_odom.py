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
from get_jacobian import get_jacobian 
from BezierFitDemo import BezierFitDemo
from compare_paths import compare_paths
from calculate_erro import calculate_erro
from points_at_interval import points_at_interval
from save_coords_to_file import save_coords_to_file
from tf_transformations import euler_from_quaternion
from generate_bezier_curve import generate_bezier_curve
from read_coords_from_file import read_points_from_file
from save_variables_to_file import save_variables_to_file
from compare_bezier_lookahead import compare_bezier_lookahead
from create_folder_with_datetime import create_folder_with_datetime

class RepeatBezierPath(Node):

    def __init__(self):
        super().__init__('repeat_bezier_path')
        self.bezier_curve_marker_pub = self.create_publisher(Marker, 'bezier_curve_marker', 10)
        self.bezier_points_marker_pub = self.create_publisher(Marker, 'bezier_points_marker', 10)
        self.lookahead_paths_marker_pub = self.create_publisher(Marker, 'lookahead_paths_marker', 10)
        self.selected_lookahead_path_marker_pub = self.create_publisher(Marker, 'selected_lookahead_path_marker', 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/hoverboard_base_controller/cmd_vel_unstamped', 10)

        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.callback_odometry, 10)

        # Variáveis que podem ser alteradas abaixo
        # Frame id markers
        frame_id = 'odom'

        # Tractor configuration
        self.tyre_radius = 0.0775
        self.max_steering = 1.0
        self.min_steering = -1.0
        self.tractor_angle = 0.0
        self.tractor_velocity = 0.2
        self.tractor_wheelbase = 1.04

        self.distance_btw_wheels = 0.41

        # threshold_dist btw tractor and coord
        self.threshold_dist = 0.8

        # Simulation
        self.dt = 0.01
        self.sim_steps = 100
        
        # Points Lookahead and Bézier Curves params
        self.points_per_paths = 15
        self.dist_btw_points = 0.2
        self.lookahead_total_paths = 100

        # Não alterar variáveis abaixo
        # Lookahead params
        self.lookahead_updated = dict()
        self.steering_angle = 0.0
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
        self.lookahead_paths_marker.scale.x = 0.05
        self.lookahead_paths_marker.scale.y = 0.05
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
        base_to_create_folder = "/home/freedom/freedom_ws/src/freedom_navigation/data/"
        path_folder_to_copy = "/home/freedom/freedom_ws/src/freedom_navigation/data/teleop_data.txt"
        self.folder_path = create_folder_with_datetime(base_to_create_folder)
        copy_file(path_folder_to_copy, self.folder_path)

        # Coleta dados de posição de quando o veiculo
        # foi teleoperado.
        self.file_teleop_path = '/home/freedom/freedom_ws/src/freedom_navigation/data/teleop_data.txt'
        teleop_path_points = read_points_from_file(self.file_teleop_path)

        # Define número inicial de knots para a curva
        # de Bézier.
        self.start_num_knots = 500
        
        # Retorna os pontos de controle para os pontos
        # enviados.
        ctrl_pts = BezierFitDemo(teleop_path_points, self.start_num_knots)
        ctrl_pts = ctrl_pts.T

        # Gera curva de Bézier
        self.bezier_path_coords = generate_bezier_curve(ctrl_pts)

        # Cria e preenche o arquivo bezier_path_coords_data.txt
        bezier_coords_data = '/bezier_path_coords_data.txt'
        self.bezier_coords_data_path = self.folder_path + bezier_coords_data
        self.file_following_path = self.folder_path + '/following_data.txt'

        save_coords_to_file(self.bezier_coords_data_path, self.bezier_path_coords)
        
        # Garante que entra cada ponto tenha um distância
        # definida.
        self.bezier_path_coords = points_at_interval(self.bezier_path_coords, self.dist_btw_points)

        # Mostra a curva de Bézier
        self.plot_bezier()

        # Gera lookahead
        self.generate_lookahead()

        # Seta o valor inicial de algumas variáveis
        self.new_min = 0
        self.bezier_few_points = 0.0
        self.new_max = self.points_per_paths

        self.start_time = time.time()

    def callback_odometry(self, msg : Odometry):
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
        msg = Twist()
        
        cost = None
        min_cost = None

        # Para cada ângulo de esterçamento atualiza os pontos
        # a partir da posição do robô.
        for steer_angle, points in self.generated_lookahead.items():
            points = np.array(points)
            d = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
            angle = np.arctan2(points[:, 1], points[:, 0])
            xs = np.array(self.x + d * np.cos(angle + self.tractor_yaw)).reshape(-1, 1)
            ys = np.array(self.y + d * np.sin(angle + self.tractor_yaw)).reshape(-1, 1)
            points = np.concatenate((xs, ys), axis = 1)
            self.lookahead_updated[steer_angle] = points

        for _, points in self.lookahead_updated.items():
            for p in points:
                point = Point()
                point.x, point.y = p[0], p[1]
                self.lookahead_paths_marker.points.append(point)

        self.lookahead_paths_marker_pub.publish(self.lookahead_paths_marker)

        self.bezier_few_points = self.bezier_path_coords[self.new_min:self.new_max]

        # Verifica a melhor curva de esterçamento
        for angle, lookahead_path_points in self.lookahead_updated.items():
            cost = compare_bezier_lookahead(lookahead_path_points, self.bezier_few_points, self.points_per_paths)
            # Seleciona o melhor path
            if min_cost is None or cost < min_cost:
                min_cost = cost
                self.desired_steering_angle = angle

                # Coloca para a variável a melhor curva de esterçamento
                # que o veiculo deve seguir
                self.best_lookahead_path = lookahead_path_points

        # msg.linear.x = self.tractor_velocity
        # msg.angular.z = self.desired_steering_angle
                
        # Differential robot
        # right_wheel = self.tractor_velocity + self.distance_btw_wheels * self.desired_steering_angle
        # left_wheel = self.tractor_velocity - self.distance_btw_wheels * self.desired_steering_angle
        # print("self.desired_steering_angle: ", self.desired_steering_angle)
        # print("desired_steering_angle: ", self.desired_steering_angle)

        left_wheel_speed =  self.tractor_velocity - self.desired_steering_angle * self.distance_btw_wheels / 2
        right_wheel_speed = self.tractor_velocity + self.desired_steering_angle * self.distance_btw_wheels / 2

        msg.linear.x = (left_wheel_speed + right_wheel_speed) / 2.0  # Velocidade linear
        msg.angular.z = (right_wheel_speed - left_wheel_speed) / self.distance_btw_wheels  # Velocidade angular

        self.cmd_vel_pub.publish(msg)

        # Atribui para a variável os pontos a frente da curva de Bézier
        for p in self.bezier_few_points:
            selected_point = Point()
            selected_point.x, selected_point.y = p[0], p[1]
            self.bezier_points_marker.points.append(selected_point)

        self.bezier_points_marker_pub.publish(self.bezier_points_marker)
        
        # Calcula a distancia entre o ponto mais proximo e o veiculo
        # se ficar menor que o treshold pode atualizar as variaveis

        if np.any(self.bezier_few_points):
            dist_tractor_from_point = np.linalg.norm(self.bezier_few_points[0] - self.tractor_position)

            if dist_tractor_from_point < self.threshold_dist:
                self.new_min += 1
                self.new_max += 1
        else:
            print("Objetivo concluido")
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)

            end_time = time.time()
            duration_time = (end_time - self.start_time) / 60


            save_coords_to_file(self.file_following_path, self.coords_during_following)
            erro_percentual = calculate_erro(self.file_teleop_path, self.file_following_path)
            erro_percentual_bezier = calculate_erro(self.file_teleop_path, self.bezier_coords_data_path)

            self.variables = {
                "tyre_radius": self.tyre_radius,
                "max_steering": self.max_steering,
                "min_steering": self.min_steering,
                "tractor_velocity": self.tractor_velocity,
                "points_per_paths": self.points_per_paths,
                "dist_btw_points": self.dist_btw_points,
                "lookahead_total_paths": self.lookahead_total_paths,
                "threshold_dist": self.threshold_dist,
                "sim_steps": self.sim_steps,
                "dt": self.dt,
                "self.start_num_knots": self.start_num_knots,
                "duration_time": duration_time,
                "erro_percentual": erro_percentual,
                "erro_percentual_bezier": erro_percentual_bezier
            }

            variables_path = self.folder_path + '/variables.txt'

            save_variables_to_file(variables_path, **self.variables)
            compare_paths(self.file_teleop_path, self.file_following_path, self.bezier_coords_data_path)

            sys.exit()

        selected_lookahead_path = self.lookahead_updated[self.desired_steering_angle]

        for p in selected_lookahead_path:
                point = Point()
                point.x, point.y = p[0], p[1]
                self.selected_lookahead_path_marker.points.append(point)

        self.selected_lookahead_path_marker_pub.publish(self.selected_lookahead_path_marker)

        self.lookahead_updated.clear()
        self.selected_lookahead_path_marker.points = []
        self.lookahead_paths_marker.points = []
        self.bezier_points_marker.points = []

    def generate_lookahead(self):
        dt = 1.0 / self.sim_steps
        d = dict()

        for steering_angle in np.linspace(self.min_steering, self.max_steering, self.lookahead_total_paths):
            x = 0.0
            y = 0.0
            tractor_yaw = 0.0 
            d[steering_angle] = list()
            # Para cada ponto de cada curva
            for _ in range(self.points_per_paths):
                # Grava a coordenada atual
                d[steering_angle].append((y, x))
                # Avança a distância definida
                for _ in range(self.sim_steps):
                    step_dist = self.dist_btw_points/self.sim_steps
                    u = np.array([(step_dist / self.tyre_radius) / dt, steering_angle])
                    q = [x, y, tractor_yaw]
                    j = get_jacobian(tractor_yaw, self.tyre_radius)
                    q_dot = np.dot(j, u)
                    q += q_dot * dt
                    
                    x = q[0]
                    y = q[1]
                    tractor_yaw = q[2]
        self.generated_lookahead = d

    def plot_bezier(self):
        for p in self.bezier_path_coords:
            point = Point()
            point.x, point.y = p[0], p[1]
            self.bezier_curve_marker.points.append(point)

        self.bezier_curve_marker_pub.publish(self.bezier_curve_marker)

def main(args=None):
    rclpy.init(args=args)
    repeat_bezier_path = RepeatBezierPath()
    rclpy.spin(repeat_bezier_path)
    repeat_bezier_path.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()