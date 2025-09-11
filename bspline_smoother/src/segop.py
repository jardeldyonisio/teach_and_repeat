import numpy as np
from ktangdt import ktangdt
from bstdst import bstdst

def segop(k, Q, x0):
    # Separates the vector x0 into its subcomponents.
    P, ang, dt = ktangdt(x0)

    # Call to the function which finds the optimum distances for a segment.
    bdt = bstdst(dt, Q, P, ang, k)
    bdt1 = [] # Empty list

    for i in range(2):
        bdt1.extend(bdt[i, :])

    # Assemble the vector of parameters for the curve.
    SOC = np.concatenate([P[0, :], P[1, :], ang, bdt1])
    
    return SOC