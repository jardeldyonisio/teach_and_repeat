#!/usr/bin/env python3

import os
import matplotlib.pyplot as plt
import numpy as np

def carregar_dados_do_arquivo(file_path):
    dados = np.loadtxt(file_path, delimiter=',')
    # Separar x e y
    x = dados[:, 0]
    y = dados[:, 1]
    return x, y

def compare_paths_two(teleop_data, following_data):

    # Obter o diretório pai
    complete_path = os.path.dirname(following_data)

    # Remover a parte "/imagem.png"
    path_without_file = os.path.join(complete_path, "")

    # Carregar dados dos arquivos
    x1, y1 = carregar_dados_do_arquivo(teleop_data)
    x2, y2 = carregar_dados_do_arquivo(following_data)

    # Plotar gráfico
    plt.figure(figsize=(10, 10))
    plt.plot(x1, y1, label='Caminho ensinado')
    plt.plot(x2, y2, label='Repetição do caminho', linestyle='--')

    plt.title('Comparação de caminhos')
    plt.xlabel('Eixo X')
    plt.ylabel('Eixo Y')
    plt.legend()
    plt.savefig(path_without_file)
    plt.grid(True)
    plt.show()
