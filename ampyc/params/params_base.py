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
from pprint import pformat
import numpy as np

from ampyc.typing import Noise, Params

class ParamsBase():
    '''
    Base class for parameters. This class holds all parameters as dataclasses for a single experiment, i.e.,
    - ctrl: controller parameters
    - sys: system parameters
    - sim: simulation parameters
    - plot: plotting parameters
    '''
    @dataclass
    class ctrl:
        '''
        Minimally required controller parameters, more can be added as needed.

        Parameters:
            name (str): Name of the controller.
            N (int): Prediction horizon.
            Q (np.ndarray): State cost matrix.
            R (np.ndarray): Input cost matrix.
        '''
        name: str
        N: int
        Q: np.ndarray
        R: np.ndarray

    @dataclass
    class sys:
        '''
        Minimally required system parameters, more can be added as needed.
        
        Parameters:
            n (int): Number of states.
            m (int): Number of inputs.
            dt (float): Time step for discrete-time systems.
            A (np.ndarray | None): State transition matrix for linear systems.
            B (np.ndarray | None): Input matrix for linear systems.
            C (np.ndarray | None): Output matrix for linear systems.
            D (np.ndarray | None): Feed-forward matrix for linear systems.
            f (Callable | None): Nonlinear state update function.
            h (Callable | None): Nonlinear output function.
            A_x (np.ndarray | None): State constraint matrix.
            b_x (np.ndarray | None): State constraint bounds.
            -> defines state constraints as the polytope :math: `A_x * x <= b_x`
            A_u (np.ndarray | None): Input constraint matrix.
            b_u (np.ndarray | None): Input constraint bounds.
            -> defines input constraints as the polytope :math: `A_u * u <= b_u`
            A_w (np.ndarray | None): Noise constraint matrix.
            b_w (np.ndarray | None): Noise constraint bounds.
            -> defines disturbance set as the polytope :math: `A_w * w <= b_w`
            noise_generator (Noise): Noise generator instance.
        '''
        # system dimensions
        n: int
        m: int
        dt: float
        
        # linear dynamics
        A: np.ndarray | None
        B: np.ndarray | None
        C: np.ndarray | None
        D: np.ndarray | None

        # nonlinear dynamics
        f: Callable | None
        h: Callable | None

        # state constraints
        A_x: np.ndarray | None
        b_x: np.ndarray | None
        
        # input constraints
        A_u: np.ndarray | None
        b_u: np.ndarray | None

        # noise description
        A_w: np.ndarray | None
        b_w: np.ndarray | None
        
        # noise generator
        noise_generator: Noise

    @dataclass
    class sim:
        '''
        Minimally required simulation parameters, more can be added as needed.

        Parameters:
            num_steps (int): Number of simulation steps.
            num_traj (int): Number of trajectories to simulate.
            x_0 (np.ndarray): Initial state for the simulation.
        '''
        num_steps: int
        num_traj: int
        x_0: np.ndarray

    @dataclass
    class plot:
        '''
        Minimally required plotting parameters, more can be added as needed.

        Parameters:
            color (str): Color for the plot lines.
            alpha (float | Callable): Transparency of the plot lines, can be a constant or a function.
            linewidth (float): Width of the plot lines.
        '''
        color: str
        alpha: float | Callable
        linewidth: float
    
    def __init__(self, ctrl: dict | None = None, sys: dict | None = None, sim: dict | None = None, plot: dict | None = None) -> Params:
        """
        Build dataclasses for controller, system, simulation, and plotting parameters.

        Args:
            ctrl (dict | None): Controller parameters. If None, default parameters are used.
            sys (dict | None): System parameters. If None, default parameters are used.
            sim (dict | None): Simulation parameters. If None, default parameters are used.
            plot (dict | None): Plotting parameters. If None, default parameters are used.
        """
        self.ctrl = self.ctrl() if ctrl is None else self.ctrl(**ctrl)
        self.sys = self.sys() if sys is None else self.sys(**sys)
        self.sim = self.sim() if sim is None else self.sim(**sim)
        self.plot = self.plot() if plot is None else self.plot(**plot)

        print(f'Successfully initialized experiment \'{self.ctrl.name}\'.')
    
    def __repr__(self) -> str:
        return f'{self.__class__.__name__}(\n' + \
            f'    ctrl={self.ctrl},\n' + \
            f'    sys={self.sys},\n' + \
            f'    sim={self.sim},\n' + \
            f'    plot={self.plot}\n' + \
            ')'

    def __str__(self) -> str:
        """String representation of the parameters.

        Returns:
            str: Formatted string representation of the stored parameters.
        """
        return f'Parameters:\n' + \
            f'    ctrl:\n {pformat(self.ctrl.__dict__, indent=8, width=60).strip("{}")}\n' + \
            f'    sys:\n {pformat(self.sys.__dict__, indent=8, width=60).strip("{}")}\n' + \
            f'    sim:\n {pformat(self.sim.__dict__, indent=8, width=50).strip("{}")}\n' + \
            f'    plot:\n {pformat(self.plot.__dict__, indent=8, width=40).strip("{}")}\n'
