import numpy as np

from opdist import opdist
from scipy.optimize import fmin, minimize

def bstdst(id, Q, P, ang, k):
    options = {'disp': False, 'xatol': 0.01, 'fatol': 0.01}
    n = id.shape[1]  # Use shape[1] since we want number of columns
    bdt = np.empty((id.shape[0], n))
    
    for i in range(n):
        # MATLAB uses 1-based indexing: k(i):k(i+1), i:i+1
        # Python needs 0-based: k[i]-1:k[i+1]-1, i:i+2
        # But we need to be careful about the ranges
        
        start_idx = k[i] - 1  # Convert MATLAB 1-based to Python 0-based
        end_idx = k[i+1]      # MATLAB k(i+1) becomes k[i+1] (exclusive end)
        
        Q_slice = Q[:, start_idx:end_idx]
        P_slice = P[:, i:i+2]
        ang_slice = ang[i:i+2]
        
        # Use scipy.optimize.minimize instead of deprecated fmin
        result = minimize(opdist, id[:, i], args=(Q_slice, P_slice, ang_slice), 
                         method='Nelder-Mead', options={'disp': False, 'xatol': 0.01, 'fatol': 0.01})
        bdt[:, i] = result.x
    
    return bdt