import numpy as np

def findClosestPoint(coord, points):
    """
    Find index of closest point in an array.

    index = findClosestPoint(coord, points)
    [index, minDist] = findClosestPoint(coord, points)
    """
    if coord.ndim == 1:
        coord = coord[np.newaxis, :]

    # number of points
    npoints = coord.shape[0]

    # allocate memory for result
    index = np.zeros(npoints, dtype=int)
    minDist = np.zeros(npoints)

    for i in range(npoints):
        # compute squared distance between current point and all points in array
        dist = np.sum((coord[i, :] - points) ** 2, axis=1)

        # keep index of closest point
        index[i] = np.argmin(dist)
        minDist[i] = np.min(dist)
    return index, minDist