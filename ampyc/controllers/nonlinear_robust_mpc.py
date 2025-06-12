'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

import casadi
import cvxpy as cp
import numpy as np
from scipy.linalg import sqrtm

from ampyc.controllers import ControllerBase


class NonlinearRMPC(ControllerBase):
    '''
    Implements the nonlinear Tube-MPC controller proposed in:

    F. Bayer, M. Bürger, and F. Allgöwer, "Discrete-time incremental ISS: A framework for
    robust NMPC", European Control Conference (ECC), 2013.

    Including the option to avoid highly uncertain areas (state-dependent disturbance) as proposed in:
    K. P. Wabersich and M. N. Zeilinger, "Nonlinear learning-based model predictive control supporting
    state and input dependent model uncertainty estimates", International Journal of Robust and Nonlinear
    Control, 2021

    More information is provided in Chapter 3 (nonlinear tube-MPC) and Chapter 4 (uncertainty avoidance)
    of the accompanying notes:
    https://github.com/IntelligentControlSystems/ampyc/notes/03_robustNMPC1.pdf
    https://github.com/IntelligentControlSystems/ampyc/notes/04_robustNMPC2.pdf
    '''

    def _init_problem(self, sys, params, rho=0.6, w_bar=None, *args, **kwargs):
        # compute tightening
        c_x, c_u, P, K, delta, _ = self.compute_tightening(rho)
        c_x = c_x.reshape(-1)
        c_u = c_u.reshape(-1)

        # store K
        self.K = K

        # compute L_w and recompute delta according to manually set w_hat
        if w_bar is not None:
            # get state disturbance matrix
            G = self.sys.G

            sqrt_P = sqrtm(P)
            L_w = np.linalg.norm(sqrt_P @ G @ np.linalg.inv(sqrt_P), ord=2)
            delta = w_bar / (1 - rho)

        # init casadi Opti object which holds the optimization problem
        self.prob = casadi.Opti()

        # define optimization variables
        self.z = self.prob.variable(sys.n, params.N+1)
        self.v = self.prob.variable(sys.m, params.N)
        self.x_0 = self.prob.parameter(sys.n)

        # define the objective
        objective = 0.0
        for i in range(params.N):
            objective += self.z[:, i].T @ params.Q @ self.z[:, i] + self.v[:, i].T @ params.R @ self.v[:, i]
        # NOTE: terminal cost is trivially zero due to terminal constraint
        self.objective = objective
        self.prob.minimize(objective)

        # define the constraints
        self.prob.subject_to(
            (self.x_0 - self.z[:, 0]).T @ P @ (self.x_0 - self.z[:, 0]) <= delta**2
        )
        for i in range(params.N):
            self.prob.subject_to(self.z[:, i+1] == sys.f(self.z[:, i], self.v[:, i]))
            self.prob.subject_to(sys.X.A @ self.z[:, i] <= sys.X.b - c_x*delta)
            self.prob.subject_to(sys.U.A @ self.v[:, i] <= sys.U.b - c_u*delta)

            if w_bar is not None:
                self.prob.subject_to(
                    (G @ self.z[:, i]).T @ P @ (G @ self.z[:, i]) <= (w_bar - L_w*delta)**2
                )
        self.prob.subject_to(self.z[:, -1] == 0.0)

    def compute_tightening(self, rho: float, solver: str | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, float]:
        ''' 
        Computes an RPI set and the corresponding tightening, which minimizes the constraint tightening.

        Args:
            rho (float): The robustness margin, in the range [0, 1).
            solver (str | None): The solver to use for the optimization problem. If None, the default solver is used.
        Returns:
            c_x (np.ndarray): Tightening of state constraints.
            c_u (np.ndarray): Tightening of input constraints.
            P (np.ndarray): Value (Lyapunov) function of the tube controller.
            K (np.ndarray): Tube controller gain.
            delta (float): Maximum allowable disturbance.
            w_bar (float): Tightening of disturbance constraints.
        '''
        # set solver or default to CLARABEL
        solver = solver if solver is not None else "CLARABEL"

        # system dimensions
        n = self.sys.n
        m = self.sys.m

        # differential dynamics matrices
        A1 = self.sys.diff_A[0]
        A2 = self.sys.diff_A[1]
        B = self.sys.diff_B[0]

        # state and input sets
        X = self.sys.X
        U = self.sys.U
        nx = X.A.shape[0]
        nu = U.A.shape[0]

        # state dependent disturbance
        G = self.sys.G

        # setup and solve the offline optimization problem
        E = cp.Variable((n, n), symmetric=True)
        Y = cp.Variable((m, n))

        gamma_x = cp.Variable((nx, 1))
        gamma_u = cp.Variable((nu, 1))
        gamma_w = cp.Variable()

        # define constraints
        constraints = []
        constraints += [E >> np.eye(n)]

        constraints += [cp.bmat([[rho**2 * E, (A1 @ E + B @ Y).T],
                                 [(A1 @ E + B @ Y), E]]) >> 0]
        
        constraints += [cp.bmat([[rho**2 * E, (A2 @ E + B @ Y).T],
                                 [(A2 @ E + B @ Y), E]]) >> 0]

        for i, X_i in enumerate(X.A):
            constraints += [cp.bmat([[cp.reshape(gamma_x[i],(1,1),'C'), X_i.reshape(1,-1) @ E],
                                     [E.T @ X_i.reshape(1,-1).T, E]]) >> 0]

        for i, U_i in enumerate(U.A):
            constraints += [cp.bmat([[cp.reshape(gamma_u[i],(1,1),'C'), U_i.reshape(1,-1) @ Y],
                                     [Y.T @ U_i.reshape(1,-1).T, E]]) >> 0]

        W_V = G @ X.V.T
        for i, W_i in enumerate(W_V.T):
            constraints += [cp.bmat([[cp.reshape(gamma_w,(1,1),'C'), W_i.reshape(1,-1)],
                                     [W_i.reshape(1,-1).T, E]]) >> 0]

        # define objective
        '''
            Please note that we included here a weighting on the state
            tightening, i.e., 50*sum(gamma_x). We did this since for this
            specific example, the cost favours the input tightening and
            including this weighting puts more emphasis on the state
            tightening, therefore ensuring more balance between the two
            terms.
        '''
        objective = cp.Minimize(
            (50*cp.sum(gamma_x) + cp.sum(gamma_u) + (nx + nu) * gamma_w) / (2 * (1 - rho))
        )

        # solve the problem
        cp.Problem(objective, constraints).solve(solver=solver)

        # recover Lyapunov function and controller
        P = np.linalg.inv(np.array(E.value))
        K = np.array(Y.value) @ P

        # compute w_bar and delta
        w_bar = np.sqrt(gamma_w.value)
        delta = w_bar / (1 - rho)

        # compute tightening of state constraints
        c_x = np.sqrt(gamma_x.value)

        # compute tightening of input constraints
        c_u = np.sqrt(gamma_u.value)

        return c_x, c_u, P, K, delta, w_bar
    
    def _define_output_mapping(self):
        return {
            'control': self.v,
            'state': self.z,
        }