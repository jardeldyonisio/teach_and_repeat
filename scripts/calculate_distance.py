#!/usr/bin/env python3

import numpy as np
from read_coords_from_file import read_points_from_file

def calculate_total_distance(path_coords):
    total_distance = 0
    for i in range(1, len(path_coords)):
        total_distance += np.linalg.norm(path_coords[i] - path_coords[i-1])
    return total_distance

file_path = "/home/lognav/lognav_ws/src/freedom_navigation/data/fixed_paths/warehouse_d.txt"
path_coords = read_points_from_file(file_path)
total_distance = calculate_total_distance(path_coords)
print("total_distance: ", total_distance)