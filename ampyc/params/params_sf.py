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

from ampyc.params import ParamsBase
from ampyc.typing import Noise
from ampyc.noise import ZeroNoise


class SFParams(ParamsBase):
    '''
    Default parameters for experiments with a nominal safety filter.
    '''
    
    @dataclass
    class ctrl:
        name: str = 'safety filter'
        N: int = 30

    @dataclass
    class sys:
        # system dimensions
        n: int = 2
        m: int = 1
        dt: float = 0.1

        # dynamics parameters
        k: float = 8.0
        g: float = 9.81
        l: float = 1.3
        c: float = 1.0

        # linear dynamics
        A: np.ndarray | None = field(init=False)
        B: np.ndarray | None = field(init=False)
        C: np.ndarray | None = field(init=False)
        D: np.ndarray | None = field(init=False)

        # state constraints
        A_x: np.ndarray | None = field(default_factory=lambda: np.array(
            [
                [1, 0], 
                [-1, 0],
                [0, 1],
                [0, -1]
            ]))
        b_x: np.ndarray | None = field(default_factory=lambda: np.array(
            [np.deg2rad(45), np.deg2rad(30), np.deg2rad(30), np.deg2rad(30)]).reshape(-1, 1))

        # input constraints
        A_u: np.ndarray | None = field(default_factory=lambda: np.array([1, -1]).reshape(-1, 1))
        b_u: np.ndarray | None = field(default_factory=lambda: np.array([2, 2]).reshape(-1, 1))

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
            self.A = np.array(
                [
                    [1, self.dt],
                    [self.dt * (-self.k + (self.g / self.l)), 1 - self.dt * self.c]
                ])
            self.B = np.array([0, self.dt]).reshape(-1, 1)
            self.C = np.eye(self.n)
            self.D = np.zeros((self.n, self.m))
            self.noise_generator = ZeroNoise(dim=self.n)

    @dataclass
    class sim:
        num_steps: int = 100
        num_traj: int = 1
        x_0: np.ndarray = field(default_factory=lambda: np.array(
            [np.deg2rad(10), np.deg2rad(-20)]).reshape(-1, 1))

    @dataclass
    class plot:
        show: bool = True
        color: str = 'red'
        alpha: float = 1.0
        linewidth: float = 1.0
