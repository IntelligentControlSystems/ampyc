'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% License of the source (polytope) is provided in a separate LICENSE file.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

from typing import TypeVar
import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import polytope as pc
from polytope.polytope import projection, reduce, extreme, is_fulldim, _get_patch
from polytope.quickhull import quickhull

# Type variable for Polytope
polytope = TypeVar('Polytope', bound='Polytope')

class Polytope(pc.Polytope):
    '''
    Improved Polytope class with additional functionality compared to polytope.Polytope.
    This class inherits from polytope.Polytope, thus see polytope.Polytope for the full documentation
    of all methods and attributes:
    https://github.com/tulip-control/polytope
    '''

    def __init__(self, A: np.ndarray | None = None, b: np.ndarray | None = None, vertices: np.ndarray | None = None, **kwargs) -> polytope:
        
        if A is None and b is None and vertices is None: # empty polytope
            super().__init__(A=np.array([]), b=np.array([]), vertices=np.array([]), normalize=False, **kwargs)
        else: # non-empty polytope
            # compute H representation if not provided
            if A is None and b is None:
                # compute H-representation from vertices
                out = qhull(vertices, output="raw")

                if len(out) == 3:
                    A, b, _ = out
                else:
                    A, b = (np.array([]), np.array([]))
                    if "verbose" in kwargs and kwargs["verbose"]:
                        print("[Warning] Could not compute H-representation from vertices; creating polytope only in V-representation!")
            
            super().__init__(A=A, b=b, vertices=vertices, normalize=False, **kwargs)
            
            # always compute V representation (comment out for better performance)
            if self.vertices is None and self.dim > 0:
                self.vertices = extreme(self)

            # alias for vertices
            self.V = self.vertices

            # get bounding box
            if self.dim > 0:
                self.bounding_box

            if self.bbox is not None:
                box = np.array(self.bbox)
                self.xlim = [box[0,0].item()*1.1, box[1,0].item()*1.1]
                if self.dim > 1:
                    self.ylim = [box[0,1].item()*1.1, box[1,1].item()*1.1]

    __array_ufunc__ = None  # disable numpy ufuncs

    def __add__(self, other: polytope | np.ndarray) -> polytope:
        """
        Add a Polytope or a vector to this Polytope.
        If the other object is a Polytope, it computes the Minkowski sum.
        If the other object is a vector, it translates the Polytope by that vector.
        """
        if isinstance(other, Polytope):
            if self.vertices is None:
                self.vertices = extreme(self)
            if other.vertices is None:
                other.vertices = extreme(other)
            return _minkowski_sum(self, other)
        else:
            return Polytope(A=self.A, b=self.b.reshape(-1,1) + self.A@other.reshape(-1,1))
    
    def __radd__(self, other: np.ndarray) -> polytope:
        """
        Left addition of a vector to this Polytope. This translates the Polytope by the vector.
        This allows for expressions like `vector + Polytope`.
        """
        return Polytope(A=self.A, b=self.b.reshape(-1,1) + self.A@other.reshape(-1,1))
    
    def __and__(self, other: polytope) -> polytope:
        """
        Intersection of two Polytopes.
        This is equivalent to the `intersect` method.
        """
        return self.intersect(other)
    
    def __sub__(self, other: polytope | np.ndarray) -> polytope:
        """
        Subtract a Polytope or a vector from this Polytope.
        If the other object is a Polytope, it computes the Pontryagin difference.
        If the other object is a vector, it translates the Polytope by that vector.
        """
        if isinstance(other, Polytope):
            if self.vertices is None:
                self.vertices = extreme(self)
            if other.vertices is None:
                other.vertices = extreme(other)
            return _pontryagin_difference(self, other)
        else:
            return Polytope(A=self.A, b=self.b.reshape(-1,1) - self.A@other.reshape(-1,1))
        
    def __rsub__(self, other: np.ndarray) -> polytope:
        """
        Left subtraction of a vector from this Polytope. This translates the Polytope by the vector.
        This allows for expressions like `vector - Polytope`.
        """
        return Polytope(A=-self.A, b=self.b.reshape(-1,1) - self.A@other.reshape(-1,1))
    
    def __mul__(self, other: float | int) -> polytope:
        """
        Scale the Polytope by a scalar.
        """
        if not isinstance(other, (float, int)):
            raise NotImplementedError('Product of two polytopes is not well defined')
        else:
            return _scale_polytope(other, self)
        
    def __rmul__(self, other: float | int) -> polytope:
        """
        Left multiplication of a scalar with this Polytope.
        """
        if not isinstance(other, (float, int)):
            raise NotImplementedError('Product of two polytopes is not well defined')
        else:
            return _scale_polytope(other, self)
        
    def __matmul__(self, other: any) -> None:
        """
        Right matrix multiplication is not defined for Polytopes (only left multiplication).
        This method is provided to prevent accidental use of the `@` operator.
        """
        raise NotImplementedError('Right matrix multiplication is not defined for Polytopes')
        
    def __rmatmul__(self, other: np.ndarray) -> polytope:
        """
        Left matrix multiplication of a Polytope with a matrix, i.e., linear transformation of the Polytope.
        """
        if not isinstance(other, np.ndarray):
            raise NotImplementedError('Product of two polytopes is not well defined')
        else:
            if self.vertices is None:
                self.vertices = extreme(self)
            return _matrix_propagate_polytope(other, self)
    
    def grid(self, N: int = 10) -> np.ndarray:
        """
        Create a grid of points within the bounding box of the Polytope.

        Args:
            N (int): The number of points to generate in the grid. The grid will be approximately sqrt(N) x sqrt(N).
        
        Returns:
            np.ndarray: A grid of points within the bounding box of the Polytope.
        """
        bbox = np.hstack(self.bbox)
        XX = np.linspace(bbox[0,0],bbox[0,1], int(np.floor(np.sqrt(N))))
        YY = np.linspace(bbox[1,0],bbox[1,1], int(np.floor(np.sqrt(N))))
        return np.stack(np.meshgrid(XX, YY),axis=2)
        
    def intersect(self, other: polytope) -> polytope:
        """
        Compute the intersection of this Polytope with another Polytope.
        """
        P = super().intersect(other)
        return Polytope(A=P.A, b=P.b, vertices=P.vertices)
    
    def plot(self, ax: plt.Axes | None = None, alpha: float = 0.25, color: str | None = None, **kwargs) -> plt.Axes:
        """
        Plot the Polytope in a given plt.Axes object. If no axes object is provided, it uses the current plt.axes.
        Args:
            ax (plt.Axes | None): The plt.Axes object to plot the Polytope in. If None, uses the current plt.Axes.
            alpha (float): The transparency of the Polytope patch.
            color (str | None): The color of the Polytope patch. If None, uses the default color.
        Returns:
            plt.Axes: The plt.Axes object with the Polytope patch added.
        """
        if ax is None:
            ax = plt.gca()

        if not is_fulldim(self):
            raise RuntimeError("Cannot plot empty polytope")

        poly = _get_patch(
            self, facecolor=color, alpha=alpha, **kwargs)
        poly.set_zorder(2) # we need this because _get_patch sets zorder to 0
        ax.add_patch(poly)
        return ax
    
    def project(self, dim, solver=None, abs_tol=1e-7, verbose=0):
        """Return Polytope projection on selected subspace.

        For usage details see function: L{_projection}.
        """
        return _projection(self, dim, solver, abs_tol, verbose)
    
    def support(self, eta):
        """Compute support function of Polytope.

        For usage details see function: L{_support}.
        """
        return _support(self, eta)
    
    def Vrep(self):
        if self.vertices is None:
            self.vertices = extreme(self)
            self.V = self.vertices
            return self.vertices
        else:
            return self.vertices
    
def qhull(vertices: np.array, abs_tol: float = 1e-7, verbose: bool = False, output: str = "polytope") -> polytope | np.ndarray:
    """
    Use quickhull to compute a convex hull.

    Args:
        vertices (np.array): Array of shape (N, d) where N is the number of vertices and d is the dimension.
        abs_tol (float): Absolute tolerance for numerical stability; default is 1e-7.
        verbose (bool): If True, prints warnings about degenerate polytopes; default is False.
        output (str): If "polytope", returns a Polytope object; if "raw", returns the raw A, b, and vertices; default is "polytope".

    Returns:
        Polytope: A Polytope object representing the convex hull of the input vertices.
        np.ndarray: If output is "raw", returns the A matrix, b vector, and vertices of the convex hull.
    """
    dim = vertices.shape[1]
    rays = vertices - vertices[0, :]
    _,S,_ = np.linalg.svd(rays)

    if np.any(S < abs_tol):
        if verbose:
            print("[Warning] degenerate polytope detected! Cannot compute polytope in H-Rep; returning Polytope only in V-Rep!")
        # remove redundant vertices; only works for 2D case!
        if dim == 2:
            redundant_idx = np.all(np.abs(np.cross(rays[1:, None, :], rays[1:])) < abs_tol, axis=0)
            if np.all(redundant_idx):
                # all vertices are redundant; keep the vertex with largest distance to the first
                keep_idx = np.argmax(np.linalg.norm(rays, axis=1))
                vert = np.vstack([vertices[0], vertices[keep_idx]])
            else:
                # remove redundant vertices
                vert = np.vstack([vertices[0], vertices[1:][~redundant_idx]])
        else:
            # for higher dimensions, return original vertices
            vert = vertices
        
        if output == "polytope":
            return Polytope(vertices=vert)
        elif output == "raw":
            return vert
        else:
            raise ValueError("Output must be 'polytope' or 'raw', got: {}".format(output))
    else:
        A, b, vert = quickhull(vertices, abs_tol=abs_tol)
        if A.size == 0:
            if verbose:
                print("[Warning] Could not find convex hull; returning empty polytope!")
            return Polytope()
        
        if output == "polytope":
            return Polytope(A, b, minrep=True, vertices=vert)
        elif output == "raw":
            return A, b, vert
        else:
            raise ValueError("Output must be 'polytope' or 'raw', got: {}".format(output))

def _reduce(P: polytope) -> polytope:
    """This is just a wrapper around the polytope.reduce function."""
    pc_P = reduce(P)
    return Polytope(A=pc_P.A, b=pc_P.b, vertices=pc_P.vertices)
    
def _support(P: polytope, eta: np.array) -> polytope:
    '''
    The support function of the polytope P, evaluated at (or in the direction)
    eta in R^n.

    Based on https://github.com/heirung/pytope/blob/master/pytope/polytope.py#L457

    Args:
        P (Polytope): The polytope for which to compute the support function.
        eta (np.array): The direction in which to compute the support function.
    
    Returns:
        float: The value of the support function in the direction eta.
    '''
    n = P.A.shape[1]
    x = cp.Variable((n,1))
    constraints = [P.A @ x <= P.b.reshape(-1,1)]
    objective = cp.Maximize(eta.reshape(1,-1) @ x)
    prob = cp.Problem(objective, constraints)
    prob.solve()
    if prob.status != 'optimal':
        raise Exception('Unable to compute support for the given polytope and direction eta!')
    return objective.value

def _minkowski_sum(P: polytope, Q: polytope) -> polytope:
    '''
    Minkowski sum of two convex polytopes P and Q :math: `P + Q = {p + q in R^n : p \in P, q \in Q}`.
    In vertex representation, this is the convex hull of the pairwise sum of all
    combinations of points in P and Q.

    Based on https://github.com/heirung/pytope/blob/master/pytope/polytope.py#L601

    NOTE: This only requires vertex representations of P and Q, meaning that it will
    work even if ONE of P or Q is not full-dimensional (unbounded polyhedron).
    BUT the output must be a full-dimensional polytope!
    '''
    if P.vertices is None or Q.vertices is None:
        raise Exception('Polytopes must have a vertex representation for Minkowski Sum!')

    assert P.vertices.shape[1] == Q.vertices.shape[1], 'Polytopes must be of same dimension'
    n = P.vertices.shape[1]

    num_verts_P, num_verts_Q = (P.vertices.shape[0], Q.vertices.shape[0])
    msum_V = np.full((num_verts_P * num_verts_Q, n), np.nan, dtype=float)
    
    if num_verts_P <= num_verts_Q:
        for i in range(num_verts_P):
            msum_V[i*num_verts_Q:(i+1)*num_verts_Q, :] = Q.vertices + P.vertices[i, :].reshape(1, -1)
    else:
        for i in range(num_verts_Q):
            msum_V[i*num_verts_P:(i+1)*num_verts_P, :] = P.vertices + Q.vertices[i, :].reshape(1, -1)

    # result polytope as the convex hull of the pairwise sum of vertices
    out = qhull(msum_V)

    # check that the output is full-dimensional (bounded polyhedron)
    if len(out.b) == 0:
        raise Exception('Result of Minkowski Sum is not full-dimensional!')

    return out

def _pontryagin_difference(P: polytope, Q: polytope) -> polytope:
    '''
    Pontryagin difference for two convex polytopes P and Q :math: `P - Q = {x in R^n : x + q \in P, \forall q \in Q}`.
    In halfspace representation, this is [P.A, P.b - Q.support(P.A)], with
    Q.support(P.A) a matrix in which row i is the support of Q at row i of P.A.

    Based on https://github.com/heirung/pytope/blob/master/pytope/polytope.py#L620

    NOTE: This requires halfspace representations of P and Q.
    '''
    assert P.A.shape[1] == Q.A.shape[1], 'Polytopes must be of same dimension'
    m = P.A.shape[0]

    pdiff_b = np.full(m, np.nan)  # b vector in the Pontryagin difference P - Q
    # For each inequality i in P: subtract the support of Q in the direction P.A_i
    for i in range(m):
        ineq = P.A[i, :]
        pdiff_b[i] = P.b[i] - _support(Q, ineq)
        if pdiff_b[i] < 0:
            raise Exception('Result of Pontryagin Difference is invalid! Negative b value.')

    pdiff = Polytope(A=P.A.copy(), b=pdiff_b)

    # get a minimal representation of the result
    pdiff = _reduce(pdiff)

    return pdiff

def _projection(P: polytope, dim: list, solver: str, abs_tol: float, verbose: int) -> polytope:
    """This is just a wrapper around the polytope.projection function."""
    pc_P = projection(P, dim, solver, abs_tol, verbose)
    return Polytope(A=pc_P.A, b=pc_P.b, vertices=pc_P.vertices)

def _matrix_propagate_polytope(A:np.array, P: polytope) -> polytope:
    '''
    Propagate a polytope P through a matrix A. Based on propagating the vertices.
    '''
    assert P.vertices is not None, 'Polytope must have a vertex representation for propagation!'
    dim = P.vertices.shape[1]

    assert A.shape[1] == dim, 'A must have input dimension equal to {0}, the dimension of the polytope'.format(dim)

    verts = (A @ P.vertices.T).T
    return qhull(verts)

def _scale_polytope(a:float, P: polytope) -> polytope:
    '''
    Scale polytope P by float a.
    '''
    if not isinstance(a, (float, int)):
        raise NotImplementedError('Multiplier must be a float or int not {0}'.format(type(a)))
    
    if P.A.size != 0:
        return Polytope(P.A, a * P.b)
    else:
        print("[Warning] passed polytope has no H-Rep; returning scaled polytope only in V-Rep!")
        return Polytope(vertices=a * P.V)
