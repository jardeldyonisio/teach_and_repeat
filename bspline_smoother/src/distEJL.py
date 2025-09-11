import numpy as np

def distEJL(P, n):
    """
    Computes the initial distances from the knot points to their adjacent control points for the initial guess curve.
    
    Args:
    P: numpy array of shape (2, n), where n is the number of knot points.
    
    Returns:
    dt: numpy array of shape (2*(n-1), ), containing the vector of distances.
    """
    t = P.shape[1]
    d1 = P[:, :-1] - P[:, 1:t]  # Calculates inter-knot x and y difference values.
    d2 = np.sqrt(np.sum(d1 ** 2, axis=0)) / 3  # Computes the initial distances.
    dt = np.concatenate((d2, d2))  # Assembles the vector of distances.
    dt = np.reshape(dt, (2, n - 1))
    return dt
