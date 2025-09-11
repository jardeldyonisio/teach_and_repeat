import numpy as np

def knots(Q, k):
    P = np.empty((2, 0))
    P = np.hstack((P, Q[:, k - 1]))
    return P
