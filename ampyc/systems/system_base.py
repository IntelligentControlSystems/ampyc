'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

from abc import ABC, abstractmethod
import numpy as np

from ampyc.typing import Params, System
from ampyc.utils import Polytope


class SystemBase(ABC):
    '''
    Base class for all systems. It defines the interface for the system and the methods
    that must be implemented by a derived system.

    It defines a general nonlinear system of the form:
    .. math::
        x_{k+1} = f(x_k, u_k) + w_k \\
        y_k = h(x_k, u_k)

    where :math:`x` is the state, :math:`u` is the input, :math:`y` is the output, and :math:`w` is a disturbance.

    This class is an abstract base class and should not be instantiated directly. Instead, derive a system from
    this class and implement the following required methods:
    - update_params: update the system parameters, e.g., after a change in the system dimensions. This is also called
                     during initialization.
    - f: state update function to be implemented by the inherited class
    - h: output function to be implemented by the inherited class

    Usage:
    - get_state: Evaluates :math: `x_{k+1} = f(x_k, u_k) + w_k`
    - get_output: Evaluates :math: `y_k = h(x_k, u_k)`
    - step: Evaluates both get_state and get_output in sequence, i.e.,
        .. math::
            x_{k+1} = f(x_k, u_k) + w_k \\
            y_k = h(x_k, u_k)
        and returns both :math:`x_{k+1}` and :math:`y_k`
    '''

    def __init__(self, params: Params) -> System:
        '''
        Default constructor for the system base class. This method should not be overridden by derived systems, use
        update_params instead.

        Args:
            params: The system parameters derived from a ParamsBase dataclass.
        '''
        self.update_params(params)

    def update_params(self, params: Params) -> None:
        '''
        Updates the system parameters, e.g., after a change in the system dimensions.

        Args:
            params: The new system parameters derived from a ParamsBase dataclass.
        '''
        # time step
        self.dt = params.dt

        # system dimensions
        self.n = params.n
        self.m = params.m

        # store systems constraints as polytopes
        if params.A_x is not None and params.b_x is not None:
            self.X = Polytope(params.A_x, params.b_x)

        if params.A_u is not None and params.b_u is not None:
            self.U = Polytope(params.A_u, params.b_u)

        if params.A_w is not None and params.b_w is not None:
            self.W = Polytope(params.A_w, params.b_w)

        # noise generator
        self.noise_generator = params.noise_generator

        # handle the case of state dependent noise
        if self.noise_generator.state_dependent:
            self.G = self.noise_generator.G
        
    def get_state(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        '''
        Advance system from state x with input u, and adding a disturbance.
        
        Args:
            x: Current state of the system
            u: Input to the system
        Returns:
            x_next: Next state of the system after applying the input and adding a disturbance
                    sampled from the noise generator.
        '''
        x_next = self.f(x, u)

        # make sure that x_next is a numpy array
        if not isinstance(x_next, np.ndarray):
            x_next = np.array(x_next)

        noise = self.noise_generator.generate(x) \
            if self.noise_generator.state_dependent else self.noise_generator.generate()

        return x_next + noise

    def get_output(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        '''
        Evaluate output function for state x and input u.
        
        Args:
            x: Current state of the system
            u: Input to the system
        Returns:
            output: Output of the system after evaluating the output function.
        '''
        output = self.h(x, u)

        # make sure that output is a numpy array
        if not isinstance(output, np.ndarray):
            output = np.array(output)

        return output

    def step(self, x: np.ndarray, u: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        '''
        Advances the system by one time step, given state x & input u and returns the output.
        This method calls get_state and get_output methods in sequence.

        Args:
            x: Current state of the system
            u: Input to the system
        Returns:
            x_next: Next state of the system after applying the input and adding a disturbance.
            output: Output of the system after evaluating the output function.
        '''
        x_next = self.get_state(x, u)
        output = self.get_output(x, u)
        return x_next, output

    @classmethod
    @abstractmethod
    def f(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        '''
        Nominal (i.e. without additive disturbance) system update function to be implemented by the inherited class.
        
        Args:
            x: Current state of the system
            u: Input to the system
        Returns:
            x_next: Next state of the system after applying the input.
        Note:
            This method should not include any noise or disturbance.
        '''
        raise NotImplementedError

    @classmethod
    @abstractmethod
    def h(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        '''
        System output function to be implemented by the inherited class.
        
        Args:
            x: Current state of the system
            u: Input to the system
        Returns:
            output: Output of the system after evaluating the output function.
        '''
        raise NotImplementedError

    def _check_x_shape(self, x: np.ndarray) -> None:
        '''
        Verifies the shape of x.
        Usable if x is a float or an array type which defines a shape property (e.g. numpy and casadi arrays)
        '''
        if hasattr(x, 'shape') and self.n > 1:
            assert x.shape == (self.n, 1) or x.shape == (self.n,), 'x must be {0} dimensional, instead has shape {1}'.format(self.n, x.shape)

    def _check_u_shape(self, u: np.ndarray) -> None:
        '''
        Verifies the shape of u.
        Usable if u is a float or an array type which defines a shape property (e.g. numpy and casadi arrays)
        '''
        if hasattr(u, 'shape') and self.m > 1:
            assert u.shape == (self.m, 1) or u.shape == (self.m,), 'u must be {0} dimensional, instead has shape {1}'.format(self.m, u.shape)
