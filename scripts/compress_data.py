#!/usr/bin/env python3

import numpy as np
from save_coords_to_file import save_coords_to_file

def compress_data(file_path, sample_ratio):
    with open(file_path, 'r') as f:
        x_teleop_data = []
        y_teleop_data = []
        for index, line in enumerate(f):
            if index % sample_ratio == 0:
                x0, y0 = line.split(',')
                x_teleop_data.append(float(x0))
                y_teleop_data.append(float(y0))
        x_teleop_data= np.array(x_teleop_data)
        y_teleop_data= np.array(y_teleop_data)
        points = np.vstack((x_teleop_data, y_teleop_data)).T
        return points

points = compress_data('/home/lognav/lognav_ws/src/freedom_navigation/data/teleop_data.txt', 4)   
save_coords_to_file('/home/lognav/lognav_ws/src/freedom_navigation/data/compressed.txt', points)