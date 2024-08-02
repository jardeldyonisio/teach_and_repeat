#!/usr/bin/env python3

import os
import numpy as np

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def save_coords_to_file(file_path, path_coords):
    # Extrai o diretório do caminho do arquivo
    directory = os.path.dirname(file_path)

    # Cria o diretório se não existir
    if not os.path.exists(directory):
        os.makedirs(directory)

    print("Saving data...")  
    with open(file_path, 'w') as file:
        for point in path_coords:
            # Converte np.array para ponto se necessário
            if isinstance(point, np.ndarray):
                point = Point(point[0], point[1])
            file.write('{},{}\n'.format(point.x, point.y))
    print("All data saved.")