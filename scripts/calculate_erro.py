#!/usr/bin/env python3

import numpy as np

from read_coords_from_file import read_points_from_file

def calculate_minimum_distance(curva1, curva2):
    distancias = []
    for point1 in curva1:
        min_dist = np.min(np.sqrt(np.sum((point1 - curva2) ** 2, axis=1)))
        distancias.append(min_dist)
    return np.mean(distancias)

def calculate_erro(path_file_1, path_file_2):
    points_path1 = read_points_from_file(path_file_1)
    points_path2 = read_points_from_file(path_file_2)

    erro_medio = calculate_minimum_distance(points_path1, points_path2)
    
    valor_referencia = np.mean(np.sqrt(np.sum(points_path1 ** 2, axis=1)))
    
    # Erro percentual
    erro_percentual = (erro_medio / valor_referencia) * 100
    print("Erro percentual entre as curvas:", erro_percentual)
    return erro_percentual