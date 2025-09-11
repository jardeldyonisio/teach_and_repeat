import numpy as np
import matplotlib.pyplot as plt
from isAxisHandle import isAxisHandle

def drawPolyline(*varargin):
    """
    Draw a polyline specified by a list of points.

    Usage:
    drawPolyline(COORD)  # packs coordinates in a single [N*2] array
    drawPolyline(PX, PY)  # specifies coordinates in separate arrays
    drawPolyline(..., TYPE)  # 'closed' or 'open' (default: 'open')
    drawPolyline(..., PARAM, VALUE)  # specify plot options
    H = drawPolyline(...)  # return handle to line objects

    Example:
    t = np.linspace(0, 2*np.pi, 100)
    px = 10 * np.cos(t)
    py = 5 * np.sin(t)
    drawPolyline(px, py, 'closed')
    plt.axis('equal')

    Args:
    *varargin: Variable number of arguments.

    Returns:
    If nargout > 0, returns a handle to the list of line objects.
    """

    # Extract handle of axis to draw on
    if isAxisHandle(varargin[0]):
        ax = varargin[0]
        varargin = varargin[1:]
    else:
        ax = plt.gca()

    # If first argument is a list, draw each curve individually
    if isinstance(varargin[0], list):
        h = []
        for var in varargin[0]:
            h.extend(drawPolyline(ax, *varargin[1:], *var))
        if len(h) > 0:
            return h

    # Extract curve coordinates
    if len(varargin[0].shape) == 1:
        # First argument contains x coord, second argument contains y coord
        px = varargin[0]
        if len(varargin) == 1:
            raise ValueError('Wrong number of arguments in drawPolyline')
        py = varargin[1]
        varargin = varargin[2:]
    else:
        # First argument contains both coordinates
        px = varargin[0][:, 0]
        py = varargin[0][:, 1]
        varargin = varargin[1:]

    # Check if curve is closed or open
    closed = False
    if len(varargin) > 0:
        var = varargin[0]
        if var.lower().startswith('close'):
            closed = True
            varargin = varargin[1:]
        elif var.lower().startswith('open'):
            closed = False
            varargin = varargin[1:]

    # If curve is closed, add first point at the end of the list
    if closed:
        px = np.append(px, px[0])
        py = np.append(py, py[0])

    # Plot the curve with optional parameters
    h = ax.plot(px, py, *varargin)

    return h
