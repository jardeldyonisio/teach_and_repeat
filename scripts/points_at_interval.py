#!/usr/bin/env python3

import bezier
import numpy as np

def points_at_interval(bezier_curve, interval_distance):
    num_points = bezier_curve.shape[0]
    points_at_interval = [bezier_curve[0]]

    total_distance = 0.0
    for i in range(1, num_points):
        segment_distance = np.linalg.norm(bezier_curve[i] - bezier_curve[i - 1])
        total_distance += segment_distance

        while total_distance >= interval_distance:
            overshoot = total_distance - interval_distance
            ratio = 1.0 - (overshoot / segment_distance)
            intermediate_point = bezier_curve[i - 1] + ratio * (bezier_curve[i] - bezier_curve[i - 1])
            points_at_interval.append(intermediate_point)
            total_distance -= interval_distance

    return np.array(points_at_interval)
