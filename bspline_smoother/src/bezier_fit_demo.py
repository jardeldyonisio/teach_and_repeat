import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist

from poplt import poplt
from segop import segop
from knots import knots
from globop import globop
from cpoints import cpoints
from iguess0 import iguess0
from DrawBezierCurve import drawBezierCurve
from cubicBezierToPolyline import cubicBezierToPolyline

def bezierFit():
    '''
    @brief Demonstrate Bezier curve fitting.
    '''

    # Demonstrate Bezier curve fit with adaptive knot placement
    k = None

    C = np.array([[0, 0],
                  [1, 2],
                  [3, 3],
                  [4, 2]])
    Q = cubicBezierToPolyline(C, 65)
    n = 3
        
    # Now run the final optimization with the adaptively chosen knots
    Qt = Q.T
    IG, k_out, dpkpc = iguess0(Qt, n, k)

    # Improve the fit: Segmentally Optimum Only Curve (SOO)
    SOC = segop(k, Qt, IG)

    # Improve it again: Segmentally then Globally Optimized Curve (SGO)
    GOC = globop(SOC, Qt, 0, k, dpkpc)

    # plot SGO curve using internal routine
    plt.figure(figsize=(10, 8))
    poplt(GOC, Qt)
    plt.title('Python: Plot of SGO curve')

    # Get the Bézier control points of the curve fit
    Cnew = cpoints(GOC).T  # Cnew will be Nx2 for plotting
    P = knots(Qt, k).T     # P will be Nx2 for plotting (transpose from 2xN)

    # Plot fitted, segment by segment
    plt.figure(figsize=(12, 8))
    for i in range(0, len(Cnew)-2, 3):
        if i+3 < len(Cnew):
            drawBezierCurve(Cnew[i:i+4])   # Fitted cubic Bézier segment

    plt.plot(Cnew[:,0], Cnew[:,1], 'o-', label='New control points', linewidth=2, markersize=6)
    plt.plot(Q[:,0], Q[:,1], 'k.', label='Original data', markersize=3)
    plt.plot(P[:,0], P[:,1], 'kx', markersize=10, markeredgewidth=3, label=f'Original guess n = {n} knots')

    plt.legend()
    plt.title('Python: Detailed Bézier Curve Fit (Adaptive Knots)')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
if __name__ == '__main__':
    bezierFit()