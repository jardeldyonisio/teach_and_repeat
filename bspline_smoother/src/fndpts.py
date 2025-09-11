import numpy as np

def fndpts(z, h):
    # function x = fndpts(z,h). The inputs are a vector z of control
    # points for a segment of a curve and a step size h. The function
    # separates the control points into their x and y components and
    # then uses a de Casteljau or Chaikin scheme to compute new control
    # points which will produce the same curve. It was written by
    # E. J. Lane.
    
    # Separates the control points into x and y components
    M = z[:, 0]
    N = z[:, 1]
    
    n = len(M)
    # Initializing matrices
    M_mat = np.zeros((n, n))
    N_mat = np.zeros((n, n))
    
    # Assigning values to the first column of matrices
    M_mat[:, 0] = M
    N_mat[:, 0] = N
    
    # Loop which performs the computation
    # of new control point x and y values.
    for j in range(1, n):
        for i in range(j, n):
            M_mat[i, j] = M_mat[i-1, j-1] + ((M_mat[i, j-1] - M_mat[i-1, j-1]) * h)
            N_mat[i, j] = N_mat[i-1, j-1] + ((N_mat[i, j-1] - N_mat[i-1, j-1]) * h)
    
    # Assembles the vector of new control points.
    x = np.vstack((np.diag(M_mat), np.fliplr(np.diag(M_mat[:-1, -1]))))
    y = np.vstack((np.diag(N_mat), np.fliplr(np.diag(N_mat[:-1, -1]))))
    return np.column_stack((x, y))
