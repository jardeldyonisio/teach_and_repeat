#!/usr/bin/env python3

import os

def save_variables_to_file(file_path, **variables):
    # Extrai o diretório do caminho do arquivo
    directory = os.path.dirname(file_path)

    # Cria o diretório se não existir
    if not os.path.exists(directory):
        os.makedirs(directory)

    with open(file_path, 'w') as file:
        for variable, value in variables.items():
            line = f"{variable}: {value}\n"
            file.write(line)
