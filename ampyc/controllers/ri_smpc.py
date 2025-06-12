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

class RecoveryInitializationSMPC(ControllerBase):
    '''
    Implements the stochastic recovery initialization MPC controller with probabilistic reachable
    set (PRS) proposed in:

    L. Hewing and M. N. Zeilinger, "Stochastic Model Predictive Control for Linear Systems Using
    Probabilistic Reachable Sets", Conference on Decision Control (CDC), 2018.

    More information is provided in Chapter 7 of the accompanying notes:
    https://github.com/IntelligentControlSystems/ampyc/notes/07_stochasticMPC2.pdf
    '''
        
    def _init_problem(self, sys, params, K, *args, **kwargs):
        # store K
        self.K = K

        # look up system parameters
        Q, R, N = (params.Q, params.R, params.N)
        n, m = (sys.n, sys.m)
        A, B = (sys.A, sys.B)
        X, U = (sys.X, sys.U)

        # define optimization variables
        self.x_bar = cp.Variable((n, N+1))
        self.u_bar = cp.Variable((m, N))
        self.x_0 = cp.Parameter((n))

        # additionally define the PRS based tightenings as optimization parameters
        self.x_tight = cp.Parameter((X.A.shape[0], N))
        self.u_tight = cp.Parameter((U.A.shape[0], N))

        # define the objective
        objective = 0.0
        for i in range(N):
            objective += cp.quad_form(self.x_bar[:, i], Q) + cp.quad_form(self.u_bar[:, i], R)
        # NOTE: terminal cost is trivially zero due to terminal constraint

        # define the constraints
        constraints = [self.x_bar[:, 0] == self.x_0]
        for i in range(N):
            constraints += [self.x_bar[:, i+1] == A @ self.x_bar[:, i] + B @ self.u_bar[:, i]]
            constraints += [X.A @ self.x_bar[:, i] <= X.b - self.x_tight[:, i]]
            constraints += [U.A @ self.u_bar[:, i] <= U.b - self.u_tight[:, i]]
        constraints += [self.x_bar[:, -1] == 0.0]

        # define the CVX optimization problem object
        self.prob = cp.Problem(cp.Minimize(objective), constraints)

    def _set_additional_parameters(self, additional_parameters):
        self.x_tight.value = additional_parameters['x_tight']
        self.u_tight.value = additional_parameters['u_tight']

    def _define_output_mapping(self):
        return {
            'control': self.u_bar,
            'state': self.x_bar,
        }
