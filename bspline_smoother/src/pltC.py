import math

import numpy as np
import matplotlib.pyplot as plt

def pltC(C, Q, P):
    '''
    This function plots the Bezier curve and the control points.

    @param C: Control points of the Bezier curve.
    @param Q: Data points to be fitted.
    @param P: The indices of the points in the polyline that will be used to
    calculate the tangent angles.
    '''
    s, t = C.shape
    x = np.arange(0, 1.025, 0.025) # Defines the interval for the polynomial.
    b = len(x)

    W = np.array([]).reshape(2, 0) # Loop to construct the Bezier curve.
    for j in range(0, t-3, 3):
        Y = np.zeros((2, b))
        M = np.array([berny(3, 0, x), berny(3, 1, x), berny(3, 2, x), berny(3, 3, x)]).T
        Y += C[:, j:j+4] @ M.T
        W = np.hstack((W, Y))

    plt.plot(W[0,:], W[1,:])
    plt.plot(C[0,:], C[1,:])
    plt.plot(Q[0,:], Q[1,:], '+')
    plt.plot(P[0,:], P[1,:], 'x')
    plt.plot(C[0,:], C[1,:], 'o')
    return plt.gcf()

def berny(n, i, t):
    return (math.comb(n, i) * t**i * (1-t)**(n-i))
