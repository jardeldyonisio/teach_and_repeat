#!/usr/bin/env python3
#coding: utf-8

import os
import numpy as np

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def save_coords_to_file(file_path, path_coords):
    """
    @brief Save a list of coordinates to a file.

    This function writes a list of coordinates to a specified file. If the file does not exist, it will be created.
    Each coordinate is written in the format "x,y" on a new line.

    @param file_path: The path to the file where coordinates will be saved.
    @param path_coords: A list of coordinates to be saved. Each coordinate can be a Point object or a numpy array.
    """
    # If the file does not exist, create it.
    if not os.path.exists(file_path):
        open(file_path, 'w').close()

    with open(file_path, 'w') as file:
        for point in path_coords:
            # Convert np.array to point if necessary
            if isinstance(point, np.ndarray):
                point = Point(point[0], point[1])
            file.write('{},{}\n'.format(point.x, point.y))