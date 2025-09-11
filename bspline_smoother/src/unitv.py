import numpy as np
from copy import deepcopy

# TODO n ser um self.n

def unitv(Q, k):
    k_aux = deepcopy(k)
    r, m = Q.shape
    n  = len(k)
    uv = np.zeros((2, n))
    t = np.zeros((1, 5))

    for j in range(n):
        if j == 0:
            k_aux[j] = 0
            kt = 0
        elif j == n - 1:
            k_aux[j] = m - 5
            kt = 4
        else:
            k_aux[j] = k_aux[j] - 3
            kt = 2

        x = np.asmatrix((Q[0, k_aux[j]:k_aux[j] + 5])).T
        y = np.asmatrix((Q[1, k_aux[j]:k_aux[j] + 5])).T

        xd = np.diff(x,axis=0)
        yd = np.diff(y,axis=0)

        a = np.multiply(xd, xd)
        b = np.multiply(yd, yd)
        d = np.sqrt(a + b)

        t[0][0] = 0
        t[0][1] = d[0]
        t[0][2] = t[0][1] + d[1]
        t[0][3] = t[0][2] + d[2]
        t[0][4] = t[0][3] + d[3]

        A = np.hstack([np.ones((5, 1)), t.T, (t**2).T])
        b = np.column_stack((x, y))
        c = np.linalg.lstsq(A, b, rcond=None)[0]

        u = c[1] + 2 * c[2] * t[0][kt]
        u = u / np.linalg.norm(u)
        uv[:, j] = u
    return uv
