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
from tqdm import tqdm
from numpy.linalg import matrix_power, eigvals
from scipy.stats.distributions import chi2
from scipy.linalg import sqrtm

from ampyc.typing import System, Controller
from ampyc.utils import Polytope, qhull


def _pre_set(Omega: Polytope, A: np.ndarray) -> Polytope:
    '''
    Compute the pre-set of the polytopic set Omega under the linear
    autonomous dynamics A.

    Args:
        Omega (Polytope): The polytopic set for which the pre-set is computed.
        A (np.ndarray): The state transition matrix of the autonomous linear system.

    Returns:
        Polytope: The pre-set of Omega under the dynamics A.
    '''

    return Polytope(A=Omega.A @ A, b=Omega.b, lazy=True)

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

    return Polytope(A=Omega.A @ A, b=b_pre, lazy=True)

def compute_mpi(Omega: Polytope, A: np.ndarray, max_iter: int = 50) -> Polytope:
    '''
    Compute the maximal positive invariant (MPI) set of the polytopic set Omega
    under the linear autonomous dynamics A.

    Args:
        Omega (Polytope): The constraint set for which the MPI is computed.
        A (np.ndarray): The state transition matrix of the autonomous linear system.
        max_iter (int): Maximum number of iterations for convergence.
    
    Returns:
        Polytope: The maximal positive invariant (MPI) set.
    '''
    iters = 0
    mpi = Polytope(A=Omega.A, b=Omega.b, lazy=True)

    while iters < max_iter:
        iters += 1
        mpi_pre = _pre_set(mpi, A)
        mpi_next = mpi.intersect(mpi_pre)

        if mpi_next.is_empty:
            print('MPI computation converged to an empty set after {0} iterations.'.format(iters))
            return Polytope()

        if mpi == mpi_next:
            print('MPI computation converged after {0} iterations.'.format(iters))
            break

        if iters == max_iter:
            print('MPI computation did not converge after {0} max iterations.'.format(iters))
            break

        mpi = mpi_next

    return mpi

def compute_mrpi(Omega: Polytope, A: np.ndarray, W: Polytope, max_iter: int = 50) -> Polytope:
    '''
    Compute the maximal robust positive invariant (MRPI) set of the polytopic set Omega
    under the linear autonomous dynamics A and polytopic disturbance set W.

    Args:
        Omega (Polytope): The constraint set for which the MRPI is computed.
        A (np.ndarray): The state transition matrix of the autonomous linear system.
        W (Polytope): The polytopic disturbance set.
        max_iter (int): Maximum number of iterations for convergence.
    
    Returns:
        Polytope: The maximal robust positive invariant (MRPI) set.
    '''
    iters = 0
    mrpi = Polytope(A=Omega.A, b=Omega.b, lazy=True)

    while iters < max_iter:
        iters += 1
        mrpi_pre = _robust_pre_set(mrpi, A, W)
        mrpi_next = mrpi_pre.intersect(mrpi)

        if mrpi_next.is_empty:
            print('MRPI computation converged to an empty set after {0} iterations.'.format(iters))
            return Polytope()

        if mrpi == mrpi_next:
            print('MRPI computation converged after {0} iterations.'.format(iters))
            break

        if iters == max_iter:
            print('MRPI computation did not converge after {0} max iterations.'.format(iters))
            break

        mrpi = mrpi_next

    return mrpi

def eps_min_RPI(sys: System, K: np.ndarray, epsilon: float = 1e-6, s_max: int = 50, method: str = 'RPI') -> tuple[Polytope, dict]:
    """ 
    Computes the minimal robust positively invariant (RPI) set for a linear system.

    Depending on the method parameter, this method either computes an outer
    epsilon-approximation of the min. RPI set or uses a heuristic to approximate the
    min. RPI set.

    - method = 'RPI' implements Algorithm 1 in [1] to determine a RPI outer 
        approximation.
    - method = 'heuristic' implements a heuristic that computes the min. RPI set up
        to a maximal iteration of the Minkowski sum.

    Args:
        sys: A numpy array (the state transition matrix (must be strictly stable)
        K: The controller gain matrix.
        epsilon: Approximation error bound.
        s_max: Maximum number of iterations, at which the algorithm terminates.
        method (str): Which method to use for computing the minimal RPI set.
            - 'RPI': Returns a true RPI set. Choose this option for better accuracy,
                but it may take longer to compute.
            - 'heuristic': Returns not necessarily an RPI set. Choose this option
                for faster computation, but the result may not be a true RPI set.

    Returns:
        F_eps: The epsilon approximation of the min. RPI set.
        info: Additional information about the computation, including:
            - status: 0 if the algorithm converged, otherwise -1.
            - s (int): The number of iterations performed.
            - eps_min (float): The minimal approximatino error epsilon achieved
                by the computed RPI set.
            - alpha (float; only method = 'RPI'): scaling variable in Eq. (4) of [1],
                :math: A^s W \subset alpha W

    Raises:
        ValueError: An argument did not satisfy a necessary condition or the support
            function could not be evaluated successfully.
        Warning: If the maximum number of iterations is reached without convergence.

    Paper reference:
        [1] S. V. Raković, E. C. Kerrigan, K. I. Kouramas, D. Q. Mayne, "Invariant
            approximations of the minimal robust positively invariant set. IEEE
            Transactions on Automatic Control (2005).
    """

    status = -1  # set to 0 at successful termination

    # build closed-loop dynamics
    A = sys.A + sys.B @ K

    # check if A is strictly stable
    m, n = A.shape
    if m != n:
        raise ValueError('Closed-loop A must be a square matrix')
    if not np.all(eigvals(A) < 1):
        raise ValueError('Closed-loop A must be strictly stable: all eigenvalues must be < 1')

    # get disturbance polytope W
    H_w, h_w = (sys.W.A, sys.W.b)
    n_w = h_w.size
    # check if W is compact and contains the origin
    if not all(h_w > 0):
        raise ValueError('W does not contain the origin: g > 0 is not satisfied')
    
    if method in ['RPI', 'rpi']:
        """
        Implements Algorithm 1 in [1]. The comments below refer to the steps in the algorithm
        and the equations in the paper.
        """
        # create storage arrays
        alpha_o = np.full(s_max, np.nan)
        M_row = np.zeros((s_max, 2 * n)) # store positive and negative support values
        M = np.full(s_max, np.nan)

        # Pre-compute all matrix powers of A, A^s, s = 0, ..., s_max
        A_pwr = np.stack([matrix_power(A, i) for i in range(s_max)])

        # Step 1: Choose any s in the natural numbers (ideally, set s = 0).
        s = 0

        # Step 2: repeat
        while s < s_max - 1:
            # Step 3: Increment s by one.
            s += 1

            # Step 4: Compute alpha^o(s) as in (11).
            alpha_o_row = np.full(n_w, np.nan)
            for i,hi in enumerate(H_w):
                support = sys.W.support(A_pwr[s].T @ hi.reshape(-1, 1))
                alpha_o_row[i] = support / h_w[i]
            alpha_o[s] = np.max(alpha_o_row)

            # set alpha to alpha^o(s)
            alpha = alpha_o[s]

            # Look up s-th power of A
            A_pwr_s = A_pwr[s - 1]

            # Step 5: Compute M(s) as in (13).
            support_pos = np.full(n, np.nan)
            support_neg = np.full(n, np.nan)
            for i in range(n):
                support_pos[i] = sys.W.support(A_pwr_s[i])
                support_neg[i] = sys.W.support(-A_pwr_s[i])
            
            # Store all 2n support-function evaluations for iteration s and sum
            # form 0 to s - 1
            M_row[s] = M_row[s - 1] + np.concatenate((support_pos, support_neg))
            M[s] = np.max(M_row[s]) # take maximum of the support values

            # Step 6: break if alpha <= epsilon / (epsilon + M(s))
            if alpha <= epsilon / (epsilon + M[s]):
                status = 0  # success
                break

            s_final = s

        # Step 7: Compute F_eps as the Minkowski sum (2) and scale it to give
        # F(alpha, s) = (1 - alpha)^(-1) F_s.
        F_eps = sys.W
        for i in range(1, s_final + 1):
            F_eps += A_pwr[i] @ sys.W
        F_eps *= (1 / (1 - alpha)) # scale to obtain the epsilon-approximation

        # obtain the smallest approximation error epsilon for s_final terms in
        # the Minkowski sum.
        eps_min = M[s_final] * alpha / (1 - alpha)

        if status == -1:
            print(f"Warning: Maximum number of iterations {s_max} reached without convergence.")

        info = {'status': status, 's': s_final+1, 'eps_min': eps_min, 'alpha': alpha}

        return F_eps, info
    
    elif method in ['heuristic']:
        """
        This heuristic computes the minimal RPI set iteratively applying the
        closed-loop dynamics to the disturbance polytope W and summing the results
        until the vertices of a new summand are below a given threshold.
        """
        # initialize the Minkowski sum with the disturbance polytope W
        s = 1
        F_eps = sys.W

        # iterate until convergence or the maximum number of iterations is reached
        while s < s_max:
            # compute the matrix power of (A + B @ K)
            A_pow = matrix_power(sys.A + sys.B @ K, s)

            # compute the resulting transformation of the noise polytope W
            A_pow_W = A_pow @ sys.W

            # add the transformed polytope to the Minkowski sum
            F_eps += A_pow_W

            # check if the vertices of the transformed polytope are below a given threshold
            # if the set A^i @ W is small enough, we can assume the sequence has converged
            if np.all(np.abs(A_pow_W.V) < epsilon):
                status = 0
                break

            s += 1
        
        if status == -1:
            print(f"Warning: Maximum number of iterations {s_max} reached without convergence.")

        info = {'status': status, 's': s, 'eps_min': np.abs(A_pow_W.V).max()}

        return F_eps, info
    
    else:
        raise ValueError(f"Unknown method '{method}' for computing the minimal RPI set.")

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
    for i in tqdm(range(grid.shape[0])):
        for j in range(grid.shape[1]):
            x_0 = grid[i,j,:]
            
            # solve
            sol = ctrl.solve(x_0, additional_parameters=additional_params, verbose=False, solver=solver)
            error_msg = sol[-1]
            if error_msg is None:
                RoA[i,j] = 1

    # Convert the reachable sets to a Polytope object or a list of vertices
    if return_type == "polytope":
        return qhull(grid[RoA.astype(bool)])
    elif return_type == "array":
        return RoA

