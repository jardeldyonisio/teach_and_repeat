#!/usr/bin/env python3

import numpy as np

# Differential robot
def get_jacobian(tractor_yaw, tractor_tyre_radius):

      J = list()

      a_sin = tractor_tyre_radius * np.sin(tractor_yaw)
      a_cos = tractor_tyre_radius * np.cos(tractor_yaw)
      
      J += [[ a_sin, 0.0],
            [ a_cos, 0.0],
            [ 0.0,   1.0]]
      
      J = np.array(J)
      
      return J