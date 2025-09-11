import numpy as np
import NearestPoint

def sod(C, Q, dpkpc):
    # Handle case where dpkpc might be a tuple from newk function
    if isinstance(dpkpc, tuple):
        y = np.array(dpkpc[0], dtype=int)  # Use first element and convert to int
    else:
        y = np.array(dpkpc, dtype=int)
    
    n = C.shape[1]  # Number of control points (columns)
    r, s = Q.shape
    cntr = 0
    sum = 0
    
    # MATLAB: for i = 1:3:n-3 -> Python: range(0, n-3, 3)
    for i in range(0, n-3, 3):
        cntr += 1
        
        # MATLAB: for j = y(cntr):y(cntr+1) (1-based)
        # Python: Convert to 0-based indexing
        start_j = y[cntr-1] - 1  # Convert to 0-based
        end_j = y[cntr] - 1      # Convert to 0-based (exclusive end)
        
        # Loop to find distances from data points to closest point on the curve.
        for j in range(start_j, end_j + 1):  # Include end_j
            if j >= s:  # Safety check
                break
                
            np_point = NearestPoint.NearestPoint(C[:,i:i+4].T, Q[:,j])
            d = Q[:,j] - np_point
            
            sum += np.dot(d, d)
            
            if j == start_j and i > 0:  # First point and not first segment
                d2 = np.dot(d, d)
                ds2 = np.dot(ds, ds)
                dm = max(d2, ds2)
                sum -= dm
                
            ds = d
    
    return sum