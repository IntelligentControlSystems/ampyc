'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

import cvxpy as cp

from ampyc.controllers import ControllerBase

class IndirectFeedbackSMPC(ControllerBase):
    '''
    Implements the stochastic indirect feedback MPC controller proposed in:

    L. Hewing, K. P. Wabersich, and M. N. Zeilinger, "Recursively feasible stochastic model predictive control
    using indirect feedback", Automatica, 2020.

    More information is provided in Chapter 7 of the accompanying notes:
    https://github.com/IntelligentControlSystems/ampyc/notes/07_stochasticMPC2.pdf
    '''

    def _init_problem(self, sys, params, P, K, *args, **kwargs):
        # store P and K
        self.P, self.K = (P, K)

        # look up system parameters
        Q, R, N = (params.Q, params.R, params.N)
        n, m = (sys.n, sys.m)
        A, B = (sys.A, sys.B)
        X, U = (sys.X, sys.U)

        # define optimization variables
        self.x_bar = cp.Variable((n, N+1))
        self.z = cp.Variable((n, N+1))
        self.e_bar = cp.Variable((n, N+1))
        self.u_bar = cp.Variable((m, N))
        self.v = cp.Variable((m, N))
        self.x_0 = cp.Parameter((n))  # NOTE: this is really x_bar_0

        # additionally define the PRS based tightenings 
        # and z_0 as optimization parameters
        self.x_tight = cp.Parameter((X.A.shape[0], N))
        self.u_tight = cp.Parameter((U.A.shape[0], N))
        self.z_0 = cp.Parameter((n))

        # define the objective
        objective = 0.0
        for i in range(N):
            objective += cp.quad_form(self.x_bar[:, i], Q) + cp.quad_form(self.u_bar[:, i], R)
        objective += cp.quad_form(self.x_bar[:, -1], P)

        # define the constraints
        constraints = []
        constraints += [self.x_bar[:, 0] == self.x_0]
        constraints += [self.z[:, 0] == self.z_0]
        for i in range(N):
            constraints += [self.x_bar[:, i+1] == A @ self.x_bar[:, i] + B @ self.u_bar[:, i]]
            constraints += [self.z[:, i+1] == A @ self.z[:, i] + B @ self.v[:, i]]

            constraints += [self.e_bar[:, i] == self.x_bar[:, i] - self.z[:, i]]
            constraints += [self.u_bar[:, i] == K @ self.e_bar[:, i] + self.v[:, i]]

            constraints += [X.A @ self.z[:, i] <= X.b - self.x_tight[:, i]]
            constraints += [U.A @ self.v[:, i] <= U.b - self.u_tight[:, i]]

        constraints += [self.z[:, -1] == 0.0]

        # define the CVX optimization problem object
        self.prob = cp.Problem(cp.Minimize(objective), constraints)

    def _set_additional_parameters(self, additional_parameters):
        self.x_tight.value = additional_parameters['x_tight']
        self.u_tight.value = additional_parameters['u_tight']
        self.z_0.value = additional_parameters['z_0']

    def _define_output_mapping(self):
        return {
            'control': self.u_bar,
            'state': self.z,
        }
