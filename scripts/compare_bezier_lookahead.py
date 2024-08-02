#!/usr/bin/env python3

import numpy as np

def compare_bezier_lookahead(path1, path2, points_per_paths):
        '''
        Compare the distance between the respective
        points of two trajectories, in the same
        order as they appear in the list.
        '''
        total_cost = 0.0
        weight = points_per_paths
        for point1, point2 in zip(path1, path2):
            x1, y1 = point1
            x2, y2 = point2
            dx = x2 - x1
            dy = y2 - y1
            cost = dx**2 + dy**2
            total_cost += cost*weight
            weight -= 1.0
        return total_cost