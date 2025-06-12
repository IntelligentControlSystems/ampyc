'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

import numpy as np

from ampyc.utils import Polytope
from ampyc.systems import SystemBase

class LinearAffineSystem(SystemBase):
    '''
    Implements a linear affine system of the form:
    .. math::
        x_{k+1} = A x_k + B u_k + \sum_{i=1}^{p} (\theta_i A_{\Delta i} x_k + \theta_i B_{\Delta i} u_k) + w_k \\
        y_k = C x_k + D u_k

    where :math:`x` is the state, :math:`u` is the input, :math:`y` is the output, and :math:`w` is a disturbance.
    The system is linear in the parameters :math:`\theta`, which are used to scale the uncertainty matrices :math:`A_{\Delta i}` and :math:`B_{\Delta i}`.
    The system can also store a polytopic parameter set.
    '''

    def update_params(self, params):
        super().update_params(params)

        # NOTE: This class only stores the initial estimate of theta and omega given by the params
        self.update_omega(Polytope(params.A_theta, params.b_theta))
        self.theta = params.theta
        self.num_uncertain_params = params.num_uncertain_params

        for A, B in zip(params.A_delta, params.B_delta):
            assert A.shape == (self.n, self.n), 'component of A_delta must have shape (n,n)'
            assert B.shape == (self.n, self.m), 'component of B_delta must have shape (n,m)'
        self.A_delta = params.A_delta
        self.B_delta = params.B_delta

        # NOTE: This class only stores A and B based on the initial estimate of theta from the params,
        # they're not updated
        self.A = np.zeros((self.n, self.n))
        self.A[:] = self.A_delta[0]
        for i in range(1, len(self.A_delta)):
            self.A += self.A_delta[i] * self.theta[i-1]
        
        self.B = np.zeros((self.n, self.m))
        self.B[:] = self.B_delta[0]
        for i in range(1, len(self.B_delta)):
            self.B += self.B_delta[i] * self.theta[i-1]
        
        assert params.C.shape[1] == self.n, 'C must have shape (num_output, n)'
        assert params.D.shape[1] == self.m, 'D must have shape (num_output, m)'
        self.C = params.C
        self.D = params.D

    def update_omega(self, omega: Polytope) -> None:
        '''
        Updates the parameter set omega in which the parameter :math: `\theta` is contained.
        Args:
            omega: parameter set
        '''
        self.omega = omega
        self.omega.Vrep()

    def f(self, x, u):
        self._check_x_shape(x)  # make sure x is n dimensional
        self._check_u_shape(u)  # make sure u is m dimensional
        return self.A @ x.reshape(self.n, 1) + self.B @ u.reshape(self.m, 1)

    def h(self, x, u):
        self._check_x_shape(x)  # make sure x is n dimensional
        self._check_u_shape(u)  # make sure u is m dimensional
        return self.C @ x.reshape(self.n, 1) + self.D @ u.reshape(self.m, 1)
        