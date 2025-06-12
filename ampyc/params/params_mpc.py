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

from ampyc.typing import Noise
from ampyc.noise import ZeroNoise
from ampyc.params import ParamsBase

class MPCParams(ParamsBase):
    '''
    Default parameters for experiments with a nominal linear MPC controller.
    '''

    @dataclass
    class ctrl:
        name: str = 'nominal linear MPC'
        N: int = 10
        Q: np.ndarray = np.eye(2)
        R: np.ndarray = 10 * np.eye(1)

    @dataclass
    class sys:
        # system dimensions
        n: int = 2
        m: int = 1
        dt: float = 0.1

        # dynamics matrices
        k: float = 0.
        g: float = 9.81
        l: float = 1.3 
        c: float = 0.5
        A: np.ndarray = np.array(
            [
                [1, dt],
                [dt*(-k + (g/l)), 1 - dt*c]
            ])
        B: np.ndarray = np.array([0, dt]).reshape(-1,1)
        C: np.ndarray = np.eye(n)
        D: np.ndarray = np.zeros((n,m))

        # state constraints
        A_x: np.ndarray | None = np.array(
            [
                [1, 0], 
                [-1, 0],
                [0, 1],
                [0, -1]
            ])
        b_x: np.ndarray | None = np.array([np.deg2rad(45), np.deg2rad(45), np.deg2rad(45), np.deg2rad(45)]).reshape(-1,1)

        # input constraints
        A_u: np.ndarray | None = np.array([1, -1]).reshape(-1,1)
        b_u: np.ndarray | None = np.array([5, 5]).reshape(-1,1)

        # noise description
        A_w: np.ndarray | None = None
        b_w: np.ndarray | None = None

        # noise generator
        noise_generator: Noise = ZeroNoise(dim=n)

        def __post_init__(self) -> None:
            '''
            Post-initialization: ensure that derived attributes, i.e., parameters that are computed from other static parameters,
            are set correctly.
            '''
            # dynamics matrices
            self.A = np.array(
                [
                    [1, self.dt],
                    [self.dt*(-self.k + (self.g/self.l)), 1 - self.dt*self.c]
                ])
            self.B = np.array([0, self.dt]).reshape(-1,1)
            self.C = np.eye(self.n)
            self.D = np.zeros((self.n,self.m))

            # noise generator
            self.noise_generator = ZeroNoise(dim=self.n)

    @dataclass
    class sim:
        num_steps: int = 30
        num_traj: int = 20
        x_0: np.ndarray = np.array([np.deg2rad(25), np.deg2rad(-15)]).reshape(-1,1)

    @dataclass
    class plot:
        color: str = 'blue'
        alpha: float | Callable = 1.0
        linewidth: float = 1.0
