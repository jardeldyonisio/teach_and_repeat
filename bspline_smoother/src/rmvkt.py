import numpy as np
import ktangdt
import ctpts

def rmvkt(kt, x, k):
    # Separates the components of x
    P, ang, dt = ktangdt(x)

    n = len(P)

    # Removes the knot.
    Pnew = np.concatenate((P[:, :kt-1], P[:, kt:n]), axis=1)

    m = len(ang)
    angnew = np.concatenate((ang[:, :kt-1], ang[:, kt+1:m]), axis=1)  # Removes the knot's angles.

    q = len(k)
    nk = np.concatenate((k[0:kt-1], k[kt+1:q]))  # Get rid of removed knot in sequence.

    # Computes the control points for the blended segment.
    Cseg = ctpts(P[:, kt-1:kt+1], ang[kt-1:kt+1], dt[:, kt-1:kt])

    xs = Cseg[0, :] ; ys = Cseg[1, :]	# Separates the x and y components.
    dx = np.diff(xs); dy = np.diff(ys) 	# Gets the differences in the x's, y's.
    dm = np.sqrt(dx[2]**2 + dy[2]**2)
    dn = np.sqrt(dx[3]**2 + dy[3]**2) 	# Computes distances for the control
    dmn = dm + dn  # points on the blended segment.

    dt[0, kt-1] = dt[0, kt-1]*(dmn/dm)
    dt[1, kt] = dt[1, kt]*(dmn/dn)

    # Assembles the distances.
    d1 = dt[0, :]; d2 = dt[1, :]
    d11 = np.concatenate((d1[0:kt-1], d1[kt:p]))
    d22 = np.concatenate((d2[0:kt-2], d2[kt:p]))
    dtnew = np.array([d11, d22])

    # Assembles the composite vector of parameters for the curve.
    xi = np.concatenate((Pnew[0, :], Pnew[1, :], angnew.flatten(), dtnew[0, :], dtnew[1, :]))

    return xi, nk
