'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

import casadi

from ampyc.controllers import ControllerBase

class NonlinearMPC(ControllerBase):
    '''
    Implements a standard nonlinear nominal MPC controller, see e.g. Section 2.5.5 in:

    J. B. Rawlings, D. Q. Mayne, and M. M. Diehl, "Model Predictive Control: Theory and Design",
    2nd edition, Nob Hill Publishing, 2009.

    More information is provided in Chapter 2 of the accompanying notes:
    https://github.com/IntelligentControlSystems/ampyc/notes/02_nominalMPC.pdf
    '''

    def _init_problem(self, sys, params, *args, **kwargs):
        # init casadi Opti object which holds the optimization problem
        self.prob = casadi.Opti()

        # define optimization variables
        self.x = self.prob.variable(sys.n, params.N+1)
        self.u = self.prob.variable(sys.m, params.N)
        self.x_0 = self.prob.parameter(sys.n)

        # define the objective
        objective = 0.0
        for i in range(params.N):
            objective += self.x[:, i].T @ params.Q @ self.x[:, i] + self.u[:, i].T @ params.R @ self.u[:, i]
        # NOTE: terminal cost is trivially zero due to terminal constraint
        self.objective = objective
        self.prob.minimize(objective)

        # define the constraints
        self.prob.subject_to(self.x[:, 0] == self.x_0)
        for i in range(params.N):
            self.prob.subject_to(self.x[:, i+1] == sys.f(self.x[:, i], self.u[:, i]))
            self.prob.subject_to(sys.X.A @ self.x[:, i] <= sys.X.b)
            self.prob.subject_to(sys.U.A @ self.u[:, i] <= sys.U.b)
        self.prob.subject_to(self.x[:, -1] == 0.0)

    def _define_output_mapping(self):
        return {
            'control': self.u,
            'state': self.x,
        }

