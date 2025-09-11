from ctpts import ctpts
from NearestPoint import NearestPoint

def opdist(id, Q, P, ang):
    import numpy as np
    """
    This function is the objective function to be minimized during segment optimization.
    It receives subcomponents from a curve's composite vector, a segment at a time.
    It returns the sum error for a segment.
    """
    n = len(Q[0])
    id = np.reshape(id, (-1, 1))

    C = ctpts(P, ang, id) # Call to compute the segments control points.
    se2 = 0
    cont = 0

    # Loop to find distance error in a segment.
    for j in range(n):
        npoints = NearestPoint(C.T, np.transpose((Q[:,j])))
        if (j == 0) and np.all(npoints == np.transpose(tuple(C[:, 0]))):
            d = np.zeros(2)
        elif (j == n - 1) and np.all(npoints == np.transpose(tuple(C[:, 3]))):
            d = np.zeros(2)
        else:
            d = np.transpose(((Q[:,j])) - npoints)
        se2 = se2 + np.dot(d, d.T)
        cont += 1
    return se2