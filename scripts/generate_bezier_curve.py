#!/usr/bin/env python3
#coding: utf-8

import bezier
import numpy as np

# Código que gera curvas de Bézier a partir dos pontos de controle

def generate_bezier_curve(ctrl_pts, step_size=0.001, max = 1.0, min = 0.0):
    # print("ctrl_pts: ", ctrl_pts)
    curve = bezier.Curve.from_nodes(ctrl_pts)

    # Cria pontos ao longo da curva com o espaçamento desejado
    vals = np.linspace(0.0, 1.0, num=1000)
    points = curve.evaluate_multi(vals).T
    return points