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
from ampyc.params import ParamsBase
from ampyc.noise import PolytopeVerticesNoise
from ampyc.utils import Polytope

class RMPCParams(ParamsBase):
    '''
    Default parameters for experiments with a robust linear MPC controller.
    '''

    @dataclass
    class ctrl:
        name: str = 'robust linear MPC'
        N: int = 10
        Q: np.ndarray = 100 * np.eye(2)
        R: np.ndarray = 10 * np.eye(1)

    @dataclass
    class sys:
        # system dimensions
        n: int = 2
        m: int = 1
        dt: float = 0.1

        # dynamics matrices
        k: float = 4.
        g: float = 9.81
        l: float = 1.3 
        c: float = 1.5
        A: np.ndarray = np.array(
            [
                [1, dt],
                [dt*(-k + (g/l)), 1 - dt*c]
            ])
        B: np.ndarray = np.array([0, dt]).reshape(-1,1)
        C: np.ndarray = np.diag(np.ones(n))
        D: np.ndarray = np.zeros((n,m))

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
        A_w: np.ndarray | None = np.array(
            [
                [1, 0], 
                [-1, 0],
                [0, 1],
                [0, -1]
            ])
        b_w: np.ndarray | None = np.array([np.deg2rad(0.4), np.deg2rad(0.4), np.deg2rad(0.5), np.deg2rad(0.5)]).reshape(-1,1)

        # noise generator
        noise_generator: Noise = PolytopeVerticesNoise(Polytope(A_w, b_w))

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
            self.noise_generator = PolytopeVerticesNoise(Polytope(self.A_w, self.b_w))

    @dataclass
    class sim:
        num_steps: int = 30
        num_traj: int = 25
        x_0: np.ndarray = np.array([np.deg2rad(25), np.deg2rad(20)]).reshape(-1,1)

    @dataclass
    class plot:
        color: str = 'blue'
        alpha: float | Callable = 0.3
        linewidth: float = 1.0