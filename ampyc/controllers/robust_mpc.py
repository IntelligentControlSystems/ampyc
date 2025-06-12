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
from scipy.linalg import sqrtm

from ampyc.controllers import ControllerBase

class RMPC(ControllerBase):
    '''
    Implements the robust Tube-MPC controller proposed in:

    D. Q. Mayne, M. M. Seron, and S. V. Rakovic, "Robust model predictive control of constrained linear
    systems with bounded disturbances", Automatica, 2005.

    Note: Here we implement the controller with an ellipsoidal tube, instead of a polytopic one!

    More information is provided in Chapter 3 of the accompanying notes:
    https://github.com/IntelligentControlSystems/ampyc/notes/03_robustNMPC1.pdf
    '''

    def _init_problem(self, sys, params, rho=0.9, *args, **kwargs):
        # compute tightening
        x_tight, u_tight, P, self.K, delta = self.compute_tightening(rho)
        x_tight = x_tight.flatten()
        u_tight = u_tight.flatten()

        # define optimization variables
        self.z = cp.Variable((sys.n, params.N+1))
        self.v = cp.Variable((sys.m, params.N))
        self.x_0 = cp.Parameter((sys.n))

        # define the objective
        objective = 0.0
        for i in range(params.N):
            objective += cp.quad_form(self.z[:, i], params.Q) + cp.quad_form(self.v[:, i], params.R)
        # NOTE: terminal cost is trivially zero due to terminal constraint

        # define the constraints
        constraints = [cp.norm(sqrtm(P) @ (self.x_0 - self.z[:, 0])) <= delta]
        for i in range(params.N):
            constraints += [self.z[:, i+1] == sys.A @ self.z[:, i] + sys.B @ self.v[:, i]]
            constraints += [sys.X.A @ self.z[:, i] <= sys.X.b - x_tight]
            constraints += [sys.U.A @ self.v[:, i] <= sys.U.b - u_tight]
        constraints += [self.z[:, -1] == 0.0]

        # define the CVX optimization problem object
        self.prob = cp.Problem(cp.Minimize(objective), constraints)

    def compute_tightening(self, rho: float, solver: str | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
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
        '''
        # set solver or default to global solver
        solver = solver if solver is not None else self.solver

        # system dimensions
        n = self.sys.n
        m = self.sys.m

        # system matrices
        A = self.sys.A
        B = self.sys.B

        # state, input, and disturbance sets
        X = self.sys.X
        U = self.sys.U
        W = self.sys.W
        nx = X.A.shape[0]
        nu = U.A.shape[0]

        # setup and solve the offline optimization problem
        E = cp.Variable((n, n), symmetric=True)
        Y = cp.Variable((m, n))

        c_x_2 = cp.Variable((nx, 1))
        c_u_2 = cp.Variable((nu, 1))
        bar_w_2 = cp.Variable()

        # define constraints
        constraints = []
        constraints += [E >> np.diag(np.ones(n))]
        
        E_bmat = cp.bmat(
            [
                [rho**2 * E,   (A @ E + B @ Y).T],
                [(A @ E + B @ Y), E]
            ]
        )
        constraints += [E_bmat >> 0]

        for i in range(nx):
            x_bmat = cp.bmat(
                [
                    [cp.reshape(c_x_2[i],(1,1),'C'),      X.A[i, :].reshape(1,-1) @ E],
                    [E.T @ X.A[i, :].reshape(1,-1).T, E]
                ]
            )
            constraints += [x_bmat >> 0]

        for i in range(nu):
            u_bmat = cp.bmat(
                [
                    [cp.reshape(c_u_2[i],(1,1),'C'),      U.A[i, :].reshape(1,-1) @ Y],
                    [Y.T @ U.A[i, :].reshape(1,-1).T, E]

                ]
            )
            constraints += [u_bmat >> 0]

        for i in range(W.vertices.shape[0]):
            w_bmat = cp.bmat(
                [
                    [cp.reshape(bar_w_2,(1,1),'C'),        W.vertices[i, :].reshape(1,-1)],
                    [W.vertices[i, :].reshape(1,-1).T, E]
                ]
            )
            constraints += [w_bmat >> 0]

        # define objective
        '''
            Please note that we included here a weighting on the state
            tightening, i.e., 50*sum(c_x_2). We did this since for this
            specific example, the cost favours the input tightening and
            including this weighting puts more emphasis on the state
            tightening, therefore ensuring more balance between the two
            terms.
        '''
        objective = cp.Minimize(
            (50*cp.sum(c_x_2) + cp.sum(c_u_2) + (nx + nu) * bar_w_2) / (2 * (1 - rho))
        )

        # solve the problem
        cp.Problem(objective, constraints).solve(solver=solver)

        # recover lyapunov function and controller
        P = np.linalg.inv(np.array(E.value))
        K = np.array(Y.value) @ P

        # compute delta
        delta = np.sqrt(bar_w_2.value) / (1 - rho)

        # compute tightening of state constraints
        x_tight = delta * np.sqrt(c_x_2.value)

        # compute tightening of input constraints
        u_tight = delta * np.sqrt(c_u_2.value)

        return x_tight, u_tight, P, K, delta
    
    def _define_output_mapping(self):
        return {
            'control': self.v,
            'state': self.z,
        }