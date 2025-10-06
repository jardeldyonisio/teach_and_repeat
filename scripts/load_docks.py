#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script para testar a funcionalidade de carregamento de docas
"""

import json
import os

def load_docks_from_file(file_path):
    """Carrega as docas de um arquivo JSON"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            return data
    except FileNotFoundError:
        print(f"Arquivo não encontrado: {file_path}")
        return None
    except json.JSONDecodeError:
        print(f"Erro ao decodificar JSON do arquivo: {file_path}")
        return None

def main():
    # Exemplo de como usar
    ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
    docks_file = os.path.join(ws_dir, "src", "teach_and_repeat", "path_saves", "docks.json")
    
    docks_data = load_docks_from_file(docks_file)
    
    if docks_data:
        print(f"Arquivo carregado com sucesso!")
        print(f"Total de docas: {docks_data.get('total_docks', 0)}")
        print(f"Frame de referência: {docks_data.get('reference_frame', 'N/A')}")
        print(f"Ensina orientação: {docks_data.get('teach_orientation', False)}")
        print("\nDocas registradas:")
        
        for i, dock in enumerate(docks_data.get('docks', []), 1):
            print(f"{i}. Nome: {dock['name']}")
            print(f"   Posição: ({dock['x']:.2f}, {dock['y']:.2f})")
            if docks_data.get('teach_orientation', False):
                print(f"   Orientação: {dock['yaw']:.2f}")
            print()

if __name__ == "__main__":
    main()