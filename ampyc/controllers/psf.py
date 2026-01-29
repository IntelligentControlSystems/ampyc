'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

from .controller_base import ControllerBase
import cvxpy as cp

class PSF(ControllerBase):
    '''
    Implements a standard linear nominal predictive safety filter:

    Wabersich, Kim P., and Melanie N. Zeilinger. "Linear model predictive safety certification for learning-based control." 2018 IEEE Conference on Decision and Control (CDC). IEEE, 2018.

    More information is provided in Chapter 1 of the accompanying notes:
    https://github.com/IntelligentControlSystems/ampyc/notes/09_safetyFilter2.pdf
    '''

    def __init__(self, sys, params, *args, **kwargs):
        super().__init__(sys, params, *args, **kwargs)

    def _init_problem(self, sys, params, P):
        # store terminal cost
        self.P = P

        # define optimization variables
        self.x = cp.Variable((sys.n, params.N+1))
        self.u = cp.Variable((sys.m, params.N))
        self.x_0 = cp.Parameter((sys.n))
        self.u_L = cp.Parameter((sys.m))

        # define the objective
        objective = cp.norm(self.u_L-self.u[:,0],2)**2

        # define the constraints
        constraints = [self.x[:, 0] == self.x_0]
        for i in range(params.N):
            constraints += [self.x[:, i+1] == sys.A @ self.x[:, i] + sys.B @ self.u[:, i]]
            constraints += [sys.X.A @ self.x[:, i] <= sys.X.b]
            constraints += [sys.U.A @ self.u[:, i] <= sys.U.b]
        constraints += [cp.quad_form(self.x[:, -1],P) <= 1.0]

        # define the CVX optimization problem object
        self.prob = cp.Problem(cp.Minimize(objective), constraints)
    
    def _set_additional_parameters(self, additional_parameters):
        self.u_L.value = additional_parameters['u_L']
    
    def _define_output_mapping(self):
        return {
            'control': self.u,
            'state': self.x
        }

