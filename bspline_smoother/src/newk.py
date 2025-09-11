import numpy as np

def newk(Q, P, dpkpc):
    [r, m] = Q.shape
    [s, n] = P.shape

    nk = np.zeros(n)
    nk[0] = 1
    nk[n-1] = m

    for i in range(1, n - 1):
        js = dpkpc[i-1]
        je = dpkpc[i+1]
        jm = dpkpc[i]
        z = je - js + 1
        mm = jm - js
        R = Q[:, js-1:je] - np.outer(P[:, i], np.ones(z))
        D = np.zeros(z)

        for jj in range(z):
            D[jj] = np.dot(R[:, jj], R[:, jj])

        if mm < z:
            sd = np.sign(D[mm] - D[mm + 1])
        elif mm > 1:
            sd = np.sign(D[mm-1] - D[mm])
        else:
            sd = 0

        sd = int(sd)
        while D[mm] - D[mm + sd - 1] > 0:
            if (mm == 1) and (sd < 0):
                break
            if (mm == m - 2) and (sd > 0):
                break
            mm = mm + sd

        nk[i] = mm + js - 2
    return dpkpc, nk
