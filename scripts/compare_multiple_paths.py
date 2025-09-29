#!/usr/bin/env python3
#coding: utf-8

import os
import matplotlib.pyplot as plt
import numpy as np

def carregar_dados_do_arquivo(file_path):
    dados = np.loadtxt(file_path, delimiter=',')
    x = dados[:, 0]
    y = dados[:, 1]
    return x, y

def compare_multiple_paths(teleop_data, following_data1, following_data2, following_data3, bezier_path_coords_data):
    # Obter diretório de saída
    complete_path = os.path.dirname(following_data1)
    output_base = os.path.join(complete_path, "comparacao_multiplos_caminhos")

    # Carregar dados
    x1, y1 = carregar_dados_do_arquivo(teleop_data)
    x2, y2 = carregar_dados_do_arquivo(following_data1)
    x3, y3 = carregar_dados_do_arquivo(following_data2)
    x4, y4 = carregar_dados_do_arquivo(following_data3)
    x_bezier, y_bezier = carregar_dados_do_arquivo(bezier_path_coords_data)

    # Plotar gráfico
    plt.figure(figsize=(10, 10))
    plt.plot(x1, y1, label='Original Path')
    plt.plot(x2, y2, label='Path Following 1', linestyle='--', color='b')
    plt.plot(x3, y3, label='Path Following 2', linestyle='-.', color='g')
    plt.plot(x4, y4, label='Path Following 3', linestyle=':', color='m')
    plt.plot(x_bezier, y_bezier, label='Bézier Path', linestyle=':', color='r')

    plt.title('Circular Path Comparison')
    plt.xlabel('Axis X (m)')
    plt.ylabel('Axis Y (m)')
    plt.legend()
    plt.grid(True)
    plt.xlim(0, 30)
    plt.ylim(-5, 5)
    plt.axis('equal')  # Garante proporção quadrada
    plt.savefig(output_base + '.png')
    plt.savefig(output_base + '.pdf')
    plt.show()

if __name__ == "__main__":
    # Exemplo de uso
    teleop_data = '/home/jardeldyonisio/teach_repeat_ws/src/teach_and_repeat/data/path_eight.txt'
    following_data1 = '/home/jardeldyonisio/teach_repeat_ws/src/teach_and_repeat/data/29-09-2025_18-58-18/following_data.txt'
    following_data2 = '/home/jardeldyonisio/teach_repeat_ws/src/teach_and_repeat/data/29-09-2025_19-01-44/following_data.txt'
    following_data3 = '/home/jardeldyonisio/teach_repeat_ws/src/teach_and_repeat/data/29-09-2025_19-04-36/following_data.txt'
    bezier_path_coords_data = '/home/jardeldyonisio/teach_repeat_ws/src/teach_and_repeat/data/29-09-2025_19-04-36/bezier_path_coords_data.txt'

    compare_multiple_paths(teleop_data, following_data1, following_data2, following_data3, bezier_path_coords_data)