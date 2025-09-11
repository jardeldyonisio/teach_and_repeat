import numpy as np
import matplotlib.pyplot as plt

from typing import Any, List, Tuple
from isAxisHandle import isAxisHandle
from drawPolyline import drawPolyline
from cubicBezierToPolyline import cubicBezierToPolyline

def drawBezierCurve(*varargin: Any) -> Any:

    # Extract handle of axis to draw on
    if isAxisHandle(varargin[0]):
        ax = varargin[0]
        varargin[0] = []
    else:
        ax = plt.gca()

    points = varargin[0]
    varargin = varargin[1:]

    # Default number of discretization steps
    N = 64

    # Check if discretization step is specified
    if len(varargin) > 0:
        var = varargin[0]
        if len(var) == 1 and isinstance(var, (int, float, np.number)):
            N = round(var)
            varargin[0] = []

    # Convert control coordinates to polyline
    poly = cubicBezierToPolyline(points, N)

    # Draw the curve
    h = drawPolyline(ax, poly, *varargin)

    # Eventually return a handle to the created object
    if len(h) > 0:
        return [h]