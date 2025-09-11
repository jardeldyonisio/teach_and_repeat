import numpy as np

def defk(m, n):
    k = np.round(((m-1)/(n-1)) * np.arange(n)) + np.ones(n).astype(int)
    k = k.astype(int)
    return k