#!/usr/bin/env python3

import numpy as np

def read_points_from_file(file_path):
    path_coords = np.empty((0, 2))
    with open(file_path, 'r') as f:
        for line in f:
            x, y = line.strip().split(',')
            point = np.array([[float(x), float(y)]])
            path_coords = np.append(path_coords, point, axis=0)
    return path_coords