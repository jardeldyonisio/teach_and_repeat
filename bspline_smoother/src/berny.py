import numpy as np

def berny(n, i, x):
    # This function is a non-recursive formula for cubic Bernstein
    # Polynomials which form the basis for the cubic Bezier curves
    # which are used in the supporting programs. The inputs are the
    # degree of the polynomial, n; the particular curve that is ass-
    # igned a value of zero up to and including the degree, ii and
    # the points between [0,1) to be evaluated, x. The output is
    # points on the curve. It was written by M. R. Holmes.

    ni = np.array([1, 3, 3, 1])
    m = np.shape(x)
    if n < i:
        val = np.zeros(m)
    elif i < 0:
        val = np.zeros(m)
    elif ((n == 0) and (i == 0)):
        val = 1
    else:
        val = ni[i] * (x**i) * ((np.ones(m) - x)**(n-i))
    
    return val
