#!/usr/bin/env python3

import numpy as np

from get_jacobian import get_jacobian 

def generate_lookahead(sim_steps, min_steering, max_steering, lookahead_total_paths, points_per_paths, dist_btw_points, tyre_radius, tractor_wheelbase):
        dt = 1.0 / sim_steps
        d = dict()

        for steering_angle in np.linspace(min_steering, max_steering, lookahead_total_paths):
            x = 0.0
            y = 0.0
            tractor_yaw = 0.0
            d[steering_angle] = list()
            # Para cada ponto de cada curva
            for _ in range(points_per_paths):
                # Grava a coordenada atual
                d[steering_angle].append((-x, -y))
                # Avança a distância definida
                for _ in range(sim_steps):
                    step_dist = dist_btw_points/sim_steps
                    u = np.array([(step_dist / tyre_radius) / dt, 0.0])
                    q = [x, y, steering_angle, tractor_yaw]
                    j = get_jacobian(steering_angle, tractor_yaw, tractor_wheelbase, tyre_radius)
                    q_dot = np.dot(j, u)
                    q += q_dot * dt
                    
                    x = q[0]
                    y = q[1]
                    steering_angle = q[2]
                    tractor_yaw = q[3]
        generated_lookahead = d
        return generated_lookahead