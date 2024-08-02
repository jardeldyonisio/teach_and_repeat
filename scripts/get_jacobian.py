#!/usr/bin/env python3

import numpy as np

def get_jacobian(steering_angle, tractor_angle, tractor_wheelbase, tractor_tyre_radius):
      # Based on https://doi.org/10.2507/IJSIMM20-2-550

      J = list()

      beta_0 = tractor_angle
      alpha_0s = steering_angle
      h_0 = tractor_wheelbase
      r_0f = tractor_tyre_radius

      # Eq 4
      f_l0 = r_0f * np.cos(alpha_0s)

      # Eq 5
      f_a0 = (r_0f / h_0) * np.sin(alpha_0s)

      J += [[-f_l0*np.sin(beta_0), 0.0],
      [f_l0*np.cos(beta_0), 0.0],
      [0.0, 1.0],
      [f_a0, 0.0]]

      J = np.array(J)

      return J
