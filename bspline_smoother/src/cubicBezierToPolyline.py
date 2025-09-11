import numpy as np

def cubicBezierToPolyline(C, n):
    # Generate polyline data from a cubic Bezier curve defined by its control points
    t = np.linspace(0, 1, n).reshape(n, 1)
    Q = (1-t)**3*C[0,:] + 3*t*(1-t)**2*C[1,:] + 3*t**2*(1-t)*C[2,:] + t**3*C[3,:]
    return Q