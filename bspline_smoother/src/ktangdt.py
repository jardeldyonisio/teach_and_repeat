import numpy as np
import math

# TODO: Apontar P e dt direto para esse arquivo, não precisar calcular novamente aqui 

def ktangdt(x):
    m = len(x)
    n = math.ceil(m/5)

    P = np.zeros((2, n))
    P[0, :] = x[:n]
    P[1, :] = x[n:2*n]

    ang = x[2*n:3*n]

    dt = np.zeros((2, n-1))
    dt[0, :] = x[3*n:4*n-1]
    dt[1, :] = x[4*n-1:m]
    return P, ang, dt
