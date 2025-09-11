from cubicBezierToPolyline import cubicBezierToPolyline
from findClosestPoint import findClosestPoint

def NearestPoint(C, P):
    '''
    This function finds the nearest point on a Bezier curve to a given point.

    @param C: Control points of the Bezier curve.
    @param P: Point to find the nearest point to.
    @return np: The nearest point on the Bezier curve.
    '''
    # Compute complete Bezier
    Q = cubicBezierToPolyline(C, 128)
    
    # Find nearest point on Bezier
    ind = findClosestPoint(P, Q)
    ind = ind[0]
    ind = int(ind[0])

    np = Q[ind]
    return np