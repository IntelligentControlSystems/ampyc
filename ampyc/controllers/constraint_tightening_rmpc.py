'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

import cvxpy as cp
import numpy as np

from ampyc.controllers import ControllerBase
from ampyc.utils import Polytope, compute_mrpi, compute_drs, LQR


class ConstraintTighteningRMPC(ControllerBase):
    '''
    Implements the robust constraint tightening MPC controller proposed in:

    L. Chisci, J. A. Rossiter, and G. Zappa, "Systems with persistent disturbances: predictive
    control with restricted constraints", Automatica, 2001.

    More information is provided in Chapter 3 of the accompanying notes:
    https://github.com/IntelligentControlSystems/ampyc/notes/03_robustNMPC1.pdf
    '''

    def _init_problem(self, sys, params, *args, **kwargs):
        # look up parameters
        Q, R, N = (params.Q, params.R, params.N)
        n, m = (sys.n, sys.m)

        # look up system matrices
        # self.sys contains A & B computed with the initial parameter estimate
        A, B = (sys.A, sys.B)

        # look up system constraints & disturbance set
        X, U = (sys.X, sys.U)
        W = sys.W

        # compute the terminal cost P and controller K using LQR
        self.P, self.K = LQR(A, B, Q, R)

        # compute the disturbance reachable sets
        self.F = compute_drs(A + B @ self.K, W, N)

        # compute the MRPI terminal set
        Omega = Polytope(A=np.vstack([X.A, U.A @ self.K]), b=np.hstack([X.b, U.b]).reshape(-1, 1))
        self.X_f = compute_mrpi(A + B @ self.K, Omega, W)

        # define the optimization variables
        self.z = cp.Variable((n, N+1))
        self.v = cp.Variable((m, N))
        self.x_0 = cp.Parameter((n))

        # define the objective
        objective = 0.0
        for i in range(N):
            objective += cp.quad_form(self.z[:, i], Q) + cp.quad_form(self.v[:, i], R)
        objective += cp.quad_form(self.z[:, -1], self.P)

        # define the constraints
        constraints = [self.z[:, 0] == self.x_0]
        for i in range(N):
            constraints += [self.z[:, i+1] == A @ self.z[:, i] + B @ self.v[:, i]]
            if i == 0:
                # in first time step we have no tightening
                constraints += [X.A @ self.z[:, i] <= X.b]
                constraints += [U.A @ self.v[:, i] <= U.b]
            else:
                Z_i = X - self.F[i]
                constraints += [Z_i.A @ self.z[:, i] <= Z_i.b]

                V_i = U - self.K@self.F[i]
                constraints += [V_i.A @ self.v[:, i] <= V_i.b]

        # define terminal constraint
        Z_f = self.X_f - self.F[-1]
        constraints += [Z_f.A @ self.z[:, -1] <= Z_f.b]

        # define the CVX optimization problem object
        self.prob = cp.Problem(cp.Minimize(objective), constraints)

    def _define_output_mapping(self):
        return {
            'control': self.v,
            'state': self.z,
        }