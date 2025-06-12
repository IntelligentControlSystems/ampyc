'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

import numpy as np
import cvxpy as cp
from numpy.linalg import matrix_power
from scipy.stats.distributions import chi2
from scipy.linalg import sqrtm

from ampyc.typing import System, Controller
from ampyc.utils import Polytope, qhull, _reduce

def _robust_pre_set(Omega: Polytope, A: np.ndarray, W: Polytope) -> Polytope:
    '''
    Compute the robust pre-set of the polytopic set Omega under the linear
    autonomous dynamics A and polytopic disturbance set W.

    Args:
        Omega (Polytope): The polytopic set for which the robust pre-set is computed.
        A (np.ndarray): The state transition matrix of the autonomous linear system.
        W (Polytope): The polytopic disturbance set.

    Returns:
        Polytope: The robust pre-set of Omega under the dynamics A and disturbance W.
    '''
    b_pre = Omega.b.copy()
    for i in range(Omega.b.shape[0]):
        b_pre[i] -= W.support(Omega.A[i,:])

    return Polytope(A=Omega.A @ A, b=b_pre)


def compute_mrpi(A: np.ndarray, Omega: Polytope, W: Polytope, max_iter: int = 50) -> Polytope:
    '''
    Compute the maximal robust positive invariant (MRPI) set of the polytopic set Omega
    under the linear autonomous dynamics A and polytopic disturbance set W.

    Args:
        A (np.ndarray): The state transition matrix of the autonomous linear system.
        Omega (Polytope): The constraint set for which the MRPI is computed.
        W (Polytope): The polytopic disturbance set.
        max_iter (int): Maximum number of iterations for convergence.
    
    Returns:
        Polytope: The maximal robust positive invariant (MRPI) set.
    '''
    iters = 0
    mrpi = Polytope(A=Omega.A, b=Omega.b)

    while iters < max_iter:
        iters += 1
        mrpi_pre = _robust_pre_set(mrpi, A, W)
        mrpi_next = mrpi.intersect(mrpi_pre)

        if mrpi == mrpi_next:
            print('MRPI computation converged after {0} iterations.'.format(iters))
            break

        if iters == max_iter:
            print('MRPI computation did not converge after {0} max iterations.'.format(iters))
            break

        mrpi = mrpi_next

    return _reduce(mrpi)

def compute_drs(A_BK:np.array, W:Polytope, N:int) -> list[Polytope]:
    '''
    Compute the disturbance reachable set (DRS) of the disturbance set W
    propagated by the closed-loop dynamics A_BK.

    Args:
        A_BK (np.ndarray): The closed-loop dynamics matrix (A + B*K).
        W (Polytope): The disturbance set.
        N (int): The number of time steps to compute the DRS for.
    
    Returns:
        list: A list of Polytope objects representing the DRS for each time step from 0 to N.
    '''
    F = (N+1) * [None]
    F[0] = Polytope() # F_0 as an empty polytope
    F[1] = W
    for i in range(1, N):
        F[i+1] = F[i] + matrix_power(A_BK, i) @ W
    return F

def compute_prs(sys: System, p: float, N: int) -> tuple[np.ndarray, np.ndarray, list[np.ndarray], float, np.ndarray, np.ndarray]:
    '''
    Compute the probabilistic reachable sets (PRS) and the corresponding state and input constraint tightenings.
    The tube controller for the PRS is computed such that the tubes are minimal using semidefinite programming (SDP).

    Args:
        sys (System): The system object containing the dynamics and constraints.
        p (float): The probability level for the PRS computation.
        N (int): The number of time steps to compute the PRS for.
    
    Returns:
        x_tight (np.ndarray): The tightening to be applied to the state constraints for each time step.
        u_tight (np.ndarray): The tightening to be applied to the input constraints for each time step.
        F (list[np.ndarray]): The PRS for each time step.
        p_tilde (float): The chi-squared threshold value for the given probability p.
        P (np.ndarray): The terminal cost matrix (value function of the tube controller).
        K (np.ndarray): The tube controller gain matrix.
    '''

    # look up system parameters
    n = sys.n
    m = sys.m
    noise_cov = sys.noise_generator.cov

    # dynamics matrices
    A = sys.A
    B = sys.B

    # compute p_tilde
    p_tilde = chi2.ppf(p, n)
    sqrt_p_tilde = np.sqrt(p_tilde)

    # compute tightening according to SDP
    E = cp.Variable((n, n), symmetric=True)
    Y = cp.Variable((m, n))

    objective = cp.Minimize(cp.trace(E))

    constraints = []
    constraints += [E >> 0]
    constraints += [cp.bmat([[-noise_cov + E, (A @ E + B @ Y)],
                             [(A @ E + B @ Y).T, E]]) >> 0]

    cp.Problem(objective, constraints).solve()

    # extract tube controller
    P = np.linalg.inv(np.array(E.value))
    K = np.array(Y.value) @ P
    A_K = A + B @ K

    # error variance
    var_e = (N+1) * [None]
    var_e[0] = np.zeros((n,n))
    for i in range(N):
        var_e[i+1] = A_K @ var_e[i] @ A_K.T + noise_cov

    # set F
    F = (N+1) * [None]
    for i in range(N):
        F[i] = np.linalg.inv(var_e[i+1])
    F[-1] = P

    # compute tightening
    X = sys.X
    U = sys.U
    nx = X.A.shape[0]
    nu = U.A.shape[0]

    x_tight = np.zeros((nx,N+1))
    u_tight = np.zeros((nu,N+1))

    # for every time step
    for i in range(N):
        inv_sqrt_F_i = np.linalg.inv(sqrtm(F[i]))
        # for every constraint
        for j in range(nx):
            x_tight[j, i+1] = np.linalg.norm(inv_sqrt_F_i @ X.A[j,:].reshape(-1,1), ord=2) * sqrt_p_tilde
        for j in range(nu):
            u_tight[j, i+1] = np.linalg.norm(inv_sqrt_F_i @ K.T @ U.A[j,:].reshape(-1,1), ord=2) * sqrt_p_tilde

    # check that the tightened constraints are valid
    for i in range(N):
        if np.any(X.b - x_tight[:,i] < 0) and np.any(U.b - u_tight[:,i] < 0):
            raise Exception('Infinite Step PRS Set is bigger than the state constraints')

    return x_tight, u_tight, F, p_tilde, P, K

def compute_RoA(ctrl: Controller, sys: System, grid_size: int = 25, return_type: str = "polytope", solver: str | None = None, additional_params: dict = {}) -> Polytope | np.ndarray:
    """
    Compute the region of attraction (RoA) for a given controller and system.
    The RoA is computed by simulating the system dynamics over a grid of initial states.
    The function returns the RoA as a Polytope object or a binary array over the grid, depending on the return_type parameter.

    Note:
        This method assumes a two-dimensional state space for grid sampling.
        
    Args:
        ctrl (Controller): The controller object that defines the control strategy.
        sys (System): The system object containing the dynamics and constraints.
        grid_size (int): The size of the grid to sample initial states from.
        return_type (str): The type of return value, either "polytope" or "array".
        solver (str | None): The solver to use for the controller. If None, the default solver is used.
        additional_params (dict): Additional parameters to pass to the controller's solve method.
    
    Returns:
        Polytope | np.ndarray: The region of attraction as a Polytope object or a binary array indicating feasible states.
    """
    # Create a grid of initial states
    grid = sys.X.grid(grid_size**2)

    # allocate region of attractions (RoA)
    RoA = np.zeros(grid.shape[:2])

    # check grid for feasibility
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            x_0 = grid[i,j,:]
            
            # solve
            _, _, error_msg = ctrl.solve(x_0, additional_parameters=additional_params, verbose=False, solver=solver)
            if error_msg is None:
                RoA[i,j] = 1

    # Convert the reachable sets to a Polytope object or a list of vertices
    if return_type == "polytope":
        return qhull(grid[RoA.astype(bool)])
    elif return_type == "array":
        return RoA

