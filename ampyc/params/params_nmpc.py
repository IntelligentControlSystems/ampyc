'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

from collections.abc import Callable
from dataclasses import dataclass
import numpy as np
import casadi

from ampyc.typing import Noise
from ampyc.params import ParamsBase
from ampyc.noise import PolytopeNoise
from ampyc.utils import Polytope

def _segway_f(x, u, dt, k, g, l, c):
    '''
    Nonlinear dynamics function for the inverted pendulum (segway) system.
    
    Args:
        x (casadi.SX or casadi.MX): State vector [theta, theta_dot].
        u (casadi.SX or casadi.MX): Control input (force).
        dt (float): Time step.
        k (float): Spring constant.
        g (float): Gravitational acceleration.
        l (float): Length of the pendulum.
        c (float): Damping coefficient.
    '''
    x_next = casadi.vertcat(
        x[0] + dt*x[1],
        x[1] + dt*(-k*x[0] - c*x[1] + casadi.sin(x[0])*g/l + u)
    )
    return x_next

class NonlinearMPCParams(ParamsBase):
    '''
    Default parameters for experiments with a nominal nonlinear MPC controller.
    '''

    @dataclass
    class ctrl:
        name: str = 'nominal nonlinear MPC'
        N: int = 6
        Q: np.ndarray = 100 * np.eye(2)
        R: np.ndarray = 10 * np.eye(1)

    @dataclass
    class sys:
        # system dimensions
        n: int = 2
        m: int = 1
        dt: float = 0.1

        # nonlinear dynamics
        k: float = 4.0
        g: float = 9.81
        l: float = 1.3 
        c: float = 1.5
       
        f: Callable = lambda x, u: _segway_f(x, u, dt, k, g, l, c)
        h: Callable = lambda x, u: casadi.vertcat(x)

        # state constraints
        A_x: np.ndarray | None = np.array(
            [
                [1, 0], 
                [-1, 0],
                [0, 1],
                [0, -1]
            ])
        b_x: np.ndarray | None = np.array([np.deg2rad(45), np.deg2rad(45), np.deg2rad(60), np.deg2rad(60)]).reshape(-1,1)

        # input constraints
        A_u: np.ndarray | None = np.array([1, -1]).reshape(-1,1)
        b_u: np.ndarray | None = np.array([5, 5]).reshape(-1,1)
        
        # noise description
        A_w: np.ndarray | None = np.array(
            [
                [1, 0], 
                [-1, 0],
                [0, 1],
                [0, -1]
            ])
        b_w: np.ndarray | None = np.array([1e-6, 1e-6, 1e-6, 1e-6]).reshape(-1,1)

        # noise generator
        noise_generator: Noise = PolytopeNoise(Polytope(A_w, b_w))

        def __post_init__(self) -> None:
            '''
            Post-initialization: ensure that derived attributes, i.e., parameters that are computed from other static parameters,
            are set correctly.
            '''
            # dynamics matrices
            self.f: Callable = lambda x, u: _segway_f(x, u, self.dt, self.k, self.g, self.l, self.c)

            # noise generator
            self.noise_generator = PolytopeNoise(Polytope(self.A_w, self.b_w))

    @dataclass
    class sim:
        num_steps: int = 30
        num_traj: int = 20
        x_0: np.ndarray = np.array([np.deg2rad(20), 0]).reshape(-1,1)

    @dataclass
    class plot:
        color: str = 'blue'
        alpha: float | Callable = 1.0
        linewidth: float = 1.0
