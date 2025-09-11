from ktangdt import ktangdt
from ctpts import ctpts

def cpoints(x):
    P, ang, dt = ktangdt(x)  # Separate vector x
    C = ctpts(P, ang, dt)  # Compute the control points
    return C
