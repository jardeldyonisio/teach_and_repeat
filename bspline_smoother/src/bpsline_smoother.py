import numpy as np
class BsplineSmoother:
    '''
    @brief Class for fitting Piecewise G1 Bezier curves to trajectory data.
    
    This class implements the Lane 1995 algorithm with adaptive knot placement
    for fitting cubic Bezier curves with G1 continuity to smooth paths.
    '''
    
    def __init__(self):
        '''
        @brief Initialize the BsplineSmoother with default values.
        '''

        # Read 

        # This will call BsplieFit
        self.bspline_fit = BsplieFit()
        # getPointsFromFile()

    def fit(self) -> np.ndarray:
        '''
        @brief Fit the Bezier curve to the input points and return the smoothed points.

        @return numpy.ndarray: The smoothed Bezier curve points.
        '''
        self.bspline_fit.fit()
        return self.getBsplinePoints()

    def getBsplinePoints(self) -> np.ndarray:
        '''
        @brief Get the smoothed Bezier curve points.

        @return numpy.ndarray: The smoothed Bezier curve points.
        '''
        return self.bspline_fit.getBezierPoints()

    def publishGlobalPathSmooth(self):
        '''
        @brief Publish the smoothed global path on the appropriate ROS2 topic.
        '''
        pass