'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

from collections.abc import Callable
from dataclasses import dataclass, field
import numpy as np
import casadi

from ampyc.typing import Noise
from ampyc.params import ParamsBase
from ampyc.noise import StateDependentNoise

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

class NonlinearRMPCParams(ParamsBase):
    '''
    Default parameters for experiments with a robust nonlinear MPC controller.
    '''

    @dataclass
    class ctrl:
        name: str = 'robust nonlinear MPC'
        N: int = 15
        Q: np.ndarray = 10 * np.eye(2)
        R: np.ndarray = 100 * np.eye(1)

    @dataclass
    class sys:
        # system dimensions
        n: int = 2
        m: int = 1
        dt: float = 0.1

        # nonlinear dynamics
        k: float = 4.
        g: float = 9.81
        l: float = 1.3 
        c: float = 1.5
        
        f: Callable = lambda x, u: _segway_f(x, u, dt, k, g, l, c)
        h: Callable = lambda x, u: casadi.vertcat(x)

        # differential dynamics
        diff_A: list = field(default_factory= lambda: [
                np.zeros((1,)),
            ]
        )
        diff_B: list = field(default_factory= lambda: [
                np.zeros((1,)),
            ]
        )

        # state constraints
        A_x: np.ndarray | None = np.array(
            [
                [1, 0], 
                [-1, 0],
                [0, 1],
                [0, -1]
            ])
        b_x: np.ndarray | None = np.array([np.deg2rad(30), np.deg2rad(30), np.deg2rad(45), np.deg2rad(45)]).reshape(-1,1)

        # input constraints
        A_u: np.ndarray | None = np.array([1, -1]).reshape(-1,1)
        b_u: np.ndarray | None = np.array([5, 5]).reshape(-1,1)

        # noise description
        A_w: np.ndarray | None = None
        b_w: np.ndarray | None = None

        # state dependent disturbance function
        mu_bar: float = 0.7
        G: np.ndarray | None = np.array([
            [0, 0], 
            [0, dt * mu_bar],
        ])

        # noise generator
        noise_generator: Noise = StateDependentNoise(G)

        def __post_init__(self) -> None:
            '''
            Post-initialization: ensure that derived attributes, i.e., parameters that are computed from other static parameters,
            are set correctly.
            '''
            # dynamics matrices
            self.f = lambda x, u: _segway_f(x, u, self.dt, self.k, self.g, self.l, self.c)

            # differential dynamics
            self.diff_A = [
                np.array([
                    [1, self.dt],
                    [self.dt * (-self.k + (self.g/self.l) * np.cos(0)), 1 - self.dt * self.c],
                ]),
                np.array([
                    [1, self.dt],
                    [self.dt * (-self.k + (self.g/self.l) * np.cos(np.deg2rad(30))), 1 - self.dt * self.c],
                ]),
            ]    
            self.diff_B = [
                np.array([
                    [0],
                    [self.dt],
                ])
            ]

            # state dependent disturbance function
            self.G = np.array([
                [0, 0], 
                [0, self.dt * self.mu_bar],
            ])

            # noise generator
            self.noise_generator = StateDependentNoise(self.G)

    @dataclass
    class sim:
        num_steps: int = 30
        num_traj: int = 25
        x_0: np.ndarray = np.array([np.deg2rad(20), np.deg2rad(10)]).reshape(-1,1)

    @dataclass
    class plot:
        color: str = 'blue'
        alpha: float | Callable = 0.3
        linewidth: float = 1.0
