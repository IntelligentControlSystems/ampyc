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
from ampyc.utils import Polytope, compute_drs, compute_mrpi, LQR

class ConstraintTighteningSMPC(ControllerBase):
    '''
    Implements the stochastic constraint tightening MPC controller proposed in:

    M. Lorenzen, G. Dabbene, R. Tempo, and F. Allgöwer, "Constraint-Tightening and Stability in
    Stochastic Model Predictive Control", Transactions on Automatic Control (TAC), 2017.

    More information is provided in Chapter 6 of the accompanying notes:
    https://github.com/IntelligentControlSystems/ampyc/notes/06_stochasticMPC1.pdf
    '''

    def _init_problem(self, sys, params, p=0.9, *args, **kwargs):
        # look up parameters
        Q, R, N = (params.Q, params.R, params.N)
        n, m = (sys.n, sys.m)

        # look up system matrices
        # self.sys contains A & B computed with the initial parameter estimate
        A, B = (self.sys.A, self.sys.B)

        # look up system constraints & disturbance set
        X, U = (sys.X, sys.U)
        W = sys.W

        # compute the terminal cost P and controller K using LQR
        self.P, self.K = LQR(A, B, Q, R)

        # compute the disturbance reachable sets
        self.F = compute_drs(A + B @ self.K, W, N)

        # compute stochastic backoff
        self.Fw_x, self.Fw_u = self.compute_stochastic_backoff(p, W)

        # compute the MRPI terminal set
        Omega = Polytope(A=np.vstack([X.A, U.A @ self.K]), b=np.hstack([X.b, U.b]).reshape(-1, 1))
        self.X_f = compute_mrpi(A + B @ self.K, Omega, W)

        # define the optimization variables
        self.x_bar = cp.Variable((n, N+1))
        self.u_bar = cp.Variable((m, N))
        self.x_0 = cp.Parameter((n))

        # define the objective
        objective = 0.0
        for i in range(N):
            objective += cp.quad_form(self.x_bar[:, i], Q) + cp.quad_form(self.u_bar[:, i], R)
        objective += cp.quad_form(self.x_bar[:, -1], self.P)

        # define the constraints
        constraints = [self.x_bar[:, 0] == self.x_0]
        for i in range(N):
            constraints += [self.x_bar[:, i+1] == A @ self.x_bar[:, i] + B @ self.u_bar[:, i]]
            if i == 0: # in first time step we have no constraints
                continue # do nothing
            elif i == 1: # in second time step we only stochastic tightening
                Z_i = X - self.Fw_x
                constraints += [Z_i.A @ self.x_bar[:, i] <= Z_i.b]

                V_i = U - self.Fw_u
                constraints += [V_i.A @ self.u_bar[:, i] <= V_i.b]
            else:
                Z_i = X - (A+B@self.K) @ self.F[i-1] - self.Fw_x 
                constraints += [Z_i.A @ self.x_bar[:, i] <= Z_i.b]

                V_i = U - self.K@(A + B@self.K) @ self.F[i-1] - self.Fw_u
                constraints += [V_i.A @ self.u_bar[:, i] <= V_i.b]

        Z_f = self.X_f - self.F[-1]
        constraints += [Z_f.A @ self.x_bar[:, -1] <= Z_f.b]

        # define the CVX optimization problem object
        self.prob = cp.Problem(cp.Minimize(objective), constraints)

    def compute_stochastic_backoff(self, p: float, W: Polytope) -> tuple[Polytope, Polytope]:
        '''
        Computes the stochastic backoff terms for the state and input constraints.

        Args:
            p (float): The probability of constraint satisfaction, in the range [0, 1).
            W (Polytope): The disturbance set.
            
        Returns:
            Fw_x (Polytope): The state backoff term.
            Fw_u (Polytope): The input backoff term.
        '''
        # compute state backoff term
        Fw_x = np.sqrt(p)*W

        # compute input backoff term
        Fw_u = self.K@Fw_x

        return Fw_x, Fw_u

    def _define_output_mapping(self):
        return {
            'control': self.u_bar,
            'state': self.x_bar,
        }