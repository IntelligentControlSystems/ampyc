'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

import numpy as np
import casadi

from ampyc.systems import SystemBase

class NonlinearSystem(SystemBase):
    '''
    Implements a nonlinear system of the form:
    .. math::
        x_{k+1} = f(x_k, u_k) + w_k \\
        y_k = h(x_k, u_k)

    where :math:`x` is the state, :math:`u` is the input, :math:`y` is the output, and :math:`w` is a disturbance.

    Additionally, the system can store the linear differential dynamics for the system as a list of A, B, C, and
    D matrices for different linearization points.
    '''

    def update_params(self, params):
        super().update_params(params)

        if not hasattr(params, "f"):
            raise Exception("Nonlinear dynamics f(x, u) must be defined within the system parameters!")
        self._f = params.f
        if not hasattr(params, "h"):
            raise Exception("Nonlinear dynamics h(x, u) must be defined within the system parameters!")
        self._h = params.h

        if type(self._f(np.zeros((self.n,1)), np.zeros((self.m,1)))) not in [casadi.DM, casadi.MX, casadi.SX]:
            print("WARNING: Nonlinear dynamics function f(x, u) does not return a casadi data type.\nThis may cause issues with MPC controllers using casadi!")
        if type(self._h(np.zeros((self.n,1)), np.zeros((self.m,1)))) not in [casadi.DM, casadi.MX, casadi.SX]:
            print("WARNING: Nonlinear dynamics function h(x, u) does not return a casadi data type.\nThis may cause issues with MPC controllers using casadi!")

        # store the differential dynamics for the nonlinear system if they're defined in params
        if hasattr(params, "diff_A") and hasattr(params, "diff_B"):
            self.diff_A, self.diff_B = (params.diff_A, params.diff_B)
        if hasattr(params, "diff_C") and hasattr(params, "diff_D"):
            self.diff_C, self.diff_D = (params.diff_C, params.diff_D)

    def f(self, x, u):
        self._check_x_shape(x)  # make sure x is n dimensional
        self._check_u_shape(u)  # make sure u is m dimensional
        return self._f(x, u)

    def h(self, x, u):
        self._check_x_shape(x)  # make sure x is n dimensional
        self._check_u_shape(u)  # make sure u is m dimensional
        return self._h(x, u)
