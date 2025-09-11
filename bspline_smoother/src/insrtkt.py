import numpy as np
import ktangdt
import ctpts
import fndpts

def insrtkt(seg, h, x0, k, Q):
    P, ang, dt = ktangdt(x0) # Separates xO into its subcomponents.
    q = len(k)
    
    # Computes the control points for the affected segment.
    Cseg = ctpts(P[:,seg:seg+2], ang[seg:seg+2], dt[:,seg:seg+2])
    
    z = fndpts(Cseg, h) # Call to compute new control points for the
                        # segment where the knot is inserted.
    
    xs = z[0, :] 
    ys = z[1, :] # Separates the new segment's control points into their x and y components.
    dx = np.diff(xs) 
    dy = np.diff(ys) # Finds the intercomponent differences.
    angs = np.arctan2(dy, dx) # Computes the angles for the tangent vectors.
    
    # Computes distances for control point locations.
    dl = np.sqrt(dx[0]**2 + dy[0]**2)
    de = np.sqrt(dx[5]**2 + dy[5]**2)
    dm = np.sqrt(dx[3]**2 + dy[3]**2)
    dn = np.sqrt(dx[2]**2 + dy[2]**2)
    
    # Inserts new knot into knot component vector.
    Pnew = np.hstack([P[:, :seg], z[:, 3:4], P[:, seg+1:]])
    
    # Inserts new angles into tangent angles component vector.
    angnew = np.hstack([ang[:seg], angs[3], ang[seg+1:]])
    
    # Inserts new distances into distance component vector.
    dtnew = np.hstack([dt[:, :seg-1], np.array([[dl, dm], [dn, de]]), dt[:, seg+1:]])
    
    dv = Q[:, k[seg]:k[seg+1]] - z[:, 3:4] * np.ones((1, k[seg+1]-k[seg]+1))
    ds = dv * dv
    dq = np.sum(ds, axis=0)
    knew = np.argmin(dq) + k[seg] - 1
    
    # New knot sequence.
    nk = np.hstack([k[:seg], knew, k[seg+1:q]])
    
    # Assembles the new components vector for the
    # parameters for the curve
    xi = np.hstack([Pnew[0, :], Pnew[1, :], angnew, dtnew[0, :], dtnew[1, :]])
    
    return xi, nk
