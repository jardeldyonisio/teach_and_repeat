#!/usr/bin/env python3

# import bezier
# import numpy as np

# # Código que gera curvas de Bézier a partir dos pontos de controle

# def generate_bezier_curve(ctrl_pts, step_size=0.001, max = 1.0, min = 0.0):
#     ctrl_pts_degree = len(ctrl_pts.T)
#     curve = bezier.Curve(ctrl_pts, ctrl_pts_degree - 1)

#     # Cria pontos ao longo da curva com o espaçamento desejado
#     vals = np.arange(min, max + step_size, step_size)
#     vals = np.clip(vals, min, max)  # Garante que não ultrapasse 1.0
#     points = curve.evaluate_multi(vals).T
#     return points

import bezier
import numpy as np

# Código que gera curvas de Bézier a partir dos pontos de controle

def generate_bezier_curve(ctrl_pts, step_size=0.001, max = 1.0, min = 0.0):
    print("ctrl_pts: ", ctrl_pts)
    curve = bezier.Curve.from_nodes(ctrl_pts)

    # Cria pontos ao longo da curva com o espaçamento desejado
    vals = np.linspace(0.0, 1.0, num=1000)
    points = curve.evaluate_multi(vals).T
    return points

# import numpy as np
# import matplotlib.pyplot as plt

# def binomial_coefficient(n, k):
#     """Calcula o coeficiente binomial 'n choose k'."""
#     return np.math.factorial(n) / (np.math.factorial(k) * np.math.factorial(n - k))

# def generate_bezier_curve(control_points, num_points=10000):
#     """Gera uma curva de Bézier para os pontos de controle dados."""
#     n = len(control_points) - 1
#     t_values = np.linspace(0, 1, num_points)
#     curve_points = np.zeros((num_points, control_points.shape[1]))

#     for j, t in enumerate(t_values):
#         for i in range(n + 1):
#             binom = binomial_coefficient(n, i)
#             curve_points[j] += binom * (1 - t)**(n - i) * t**i * control_points[i]

#         # Selecionar a primeira e a última coluna de cada linha do array original
#     first_elements = curve_points[:, 0]
#     last_elements = curve_points[:, -1]
#     new_curve_points = np.column_stack((first_elements, last_elements))

#     print("new_curve_points: ", new_curve_points)
#     return new_curve_points

# import numpy as np
# from scipy.special import comb

# def generate_bezier_curve(control_points, num_points=100):
#     """Gera uma curva de Bézier para os pontos de controle dados."""
#     n = control_points.shape[1] - 1  # Número de pontos de controle - 1
#     t_values = np.linspace(0, 1, num_points)
#     curve_points = np.zeros((num_points, 2))  # 2 para x e y

#     for j, t in enumerate(t_values):
#         for i in range(n + 1):
#             binom = comb(n, i)
#             # Considera a linha 0 para x e a linha 1 para y
#             curve_points[j, 0] += binom * (1 - t)**(n - i) * t**i * control_points[0, i]
#             curve_points[j, 1] += binom * (1 - t)**(n - i) * t**i * control_points[1, i]

#     print("curve_points: ", curve_points)
#     asdadsa
#     return curve_points