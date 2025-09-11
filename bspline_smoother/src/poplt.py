import matplotlib.pyplot as plt

from pltC import pltC
from ctpts import ctpts
from ktangdt import ktangdt

def poplt(x, Q):
    """
    This function picks out subcomponents of the vector x of curve parameters. 
    It calls the function that computes the control points. It then calls for a 
    plot of the curve its polygon, and the data points.
    
    Parameters:
    x (array): Vector of curve parameters
    Q (array): Data points
    
    Returns:
    pop (object): The figure object
    """
    # Separate vector x.
    P, ang, dt = ktangdt(x)
    
    # Call to compute the control points.
    C = ctpts(P, ang, dt)
    
    # Call to plot the curve, polygon, and data points.
    pop = pltC(C, Q, P)
    
    return pop