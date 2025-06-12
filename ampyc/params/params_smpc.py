'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2023, Alexandre Didier, Jérôme Sieber, Rahel Rickenbach and Shao (Mike) Zhang, ETH Zurich,
% {adidier,jsieber, rrahel}@ethz.ch
%
% All rights reserved.
%
% This code is only made available for students taking the advanced MPC 
% class in the fall semester of 2023 (151-0371-00L) and is NOT to be 
% distributed.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

from collections.abc import Callable
from dataclasses import dataclass
import numpy as np

from ampyc.typing import Noise
from ampyc.params import ParamsBase
from ampyc.noise import GaussianNoise

class SMPCParams(ParamsBase):
    '''
    Default parameters for experiments with a stochastic linear MPC controller.
    '''

    @dataclass
    class ctrl:
        name: str = 'stochastic linear MPC'
        N: int = 10
        Q: np.ndarray = 1 * np.eye(2)
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
        b_w: np.ndarray | None = np.array([np.deg2rad(0.2), np.deg2rad(0.2), np.deg2rad(0.3), np.deg2rad(0.3)]).reshape(-1,1)

        # noise description
        mean: np.ndarray = np.zeros((n,1))
        covariance: np.ndarray = 5e-4 * np.diag(np.ones(n))

        # noise generator
        noise_generator: Noise = GaussianNoise(mean, covariance)

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

            # noise description
            self.mean= np.zeros((self.n,1))
            self.covariance = 5e-4 * np.diag(np.ones(self.n))
            
            # noise generator
            self.noise_generator = GaussianNoise(self.mean, self.covariance)

    @dataclass
    class sim:
        num_steps: int = 40
        num_traj: int = 40
        x_0: np.ndarray = np.array([np.deg2rad(20), np.deg2rad(30)]).reshape(-1,1)

    @dataclass
    class plot:
        color: str = 'blue'
        alpha: float | Callable = lambda i: 0.9/(1+i**(1.75))
        linewidth: float = 1.0
