'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

from abc import ABC, abstractmethod
from typing import Union, Optional
from itertools import compress
from pprint import pformat
import numpy as np
import cvxpy as cp
import casadi
from casadi import has_nlpsol

from ampyc.typing import System, Params, Controller

class ControllerBase(ABC):
    '''
    Base class for all controllers. It defines the interface for the controller and the methods
    that must be implemented by a derived controller.

    This class is an abstract base class and should not be instantiated directly. Instead, derive a controller from
    this class and implement the following required methods:
    - _init_problem: to define the optimization problem of the controller
    - _define_output_mapping: to define the mapping from optimization variables to outputs
    - [optional] _set_additional_parameters: to set additional parameters of the optimization problem

    Attributes:
        sys: internal copy of a system object
        params: internal copy of a parameters object
        prob: optimization problem object, either a CVXPY Problem or a CasADi Opti object
    '''

    def __init__(self, sys: System, params: Params, *args: Optional, **kwargs: Optional) -> Controller:
        '''
        Default constructor for the controller. This method should not be overridden by derived controllers, use
        _init_problem instead.

        Args:
            sys: system object derived from SystemBase
            params: parameters object derived from ParamsBase
            *args: additional arguments for the controller
            **kwargs: additional keyword arguments for the controller
        '''
        self.sys = sys
        self.params = params
        self.solver = kwargs.pop('solver', None)
        self._init_problem(sys, params, *args, **kwargs)
        self.output_mapping = self._define_output_mapping()
    
    @classmethod
    @abstractmethod
    def _init_problem(self, sys: System, params: Params, *args: Optional, **kwargs: Optional) -> None:
        '''
        This method must be implemented by the controller to define its inner structure, e.g. an optimization problem.
        This method is called during the initialization of the controller.
        A controller derived from this class must implement this method instead of overriding the __init__ method.

        Args:
            sys: system object derived from SystemBase
            params: parameters object derived from ParamsBase
            *args: additional arguments for the controller
            **kwargs: additional keyword arguments for the controller
        '''
        raise NotImplementedError

    def _set_additional_parameters(self, additional_parameters: dict) -> None:
        '''
        Some controllers require setting additional parameters of the optimization problem beside just setting the initial
        condition. For these controllers, override this method to set the value of those static parameters.
        This method will be called to set the additional parameters right before calling the solver.

        Args:
            additional_parameters: dictionary of additional parameters to be set in the optimization problem
        '''
        pass

    @classmethod
    @abstractmethod
    def _define_output_mapping(self) -> None:
        '''
        Depending on the controller, the final output of the controller may correspond to different variables. For example,
        in case of LQR controllers, the output is the optimal control input and the predicted state trajectory over the
        horizon.
        This method must be implemented by a derived controller to define the mapping from optimization variables to outputs.
        '''

        ''' TEMPLATE (Nominal MPC)
        return {
            'control': # planned control input trajectory,
            'state': # planned state trajectory
        }
        '''
        
        raise NotImplementedError

    def solve(self,
              x: np.ndarray,
              additional_parameters: dict = {},
              verbose: bool = False,
              solver: str | None = None,
              options: dict | None = None
              ) -> Union[tuple[np.ndarray, np.ndarray, dict, str | None], tuple[np.ndarray, np.ndarray, str | None]]:
        '''
        Solve the optimization problem defined in the controller.

        Args:
            x: initial condition of the system, i.e. the state at time t=0
            additional_parameters: dictionary of additional parameters to be set in the optimization problem
            verbose: if True, print solver output
            solver: solver to be used for the optimization problem, if None, use the default solver defined below
            options: options for the solver, if None, use default options

        Returns:
            control: planned control input trajectory
            state: planned state trajectory
            out_map: output mapping of the optimization problem, if defined (beyond just control and state)
            error_msg: error message, if the solver did not achieve an optimal solution or encountered an error

        Raises:
            Exception: if the optimization problem is not initialized; an initial condition is not defined; if the defined
                       optimization problem is neither a CVXPY nor CasADi object; or if the output mapping is not defined
            Warning: if a non-default solver is used to solve a CasADi object AND options are not defined
        
        Note:
            Default solvers: for CVXPY, the default solver automatically selected by CVXPY depending on the type of optimization
                             problem; for CasADi, the default solver is "ipopt".
        '''
        # if solver is not provided, use default global solver
        solver = solver if solver is not None else self.solver
        
        if self.prob != None:
            if not hasattr(self, 'x_0'):
                raise Exception(
                    'The MPC problem must define the initial condition as an optimization parameter self.x_0')
            
            out_map = self.output_mapping.copy()

            if isinstance(self.prob,cp.Problem):
                try:
                    self.x_0.value = x
                    self._set_additional_parameters(additional_parameters)
                    self.prob.solve(verbose=verbose, solver=solver)

                    if self.prob.status != cp.OPTIMAL:
                        error_msg = 'Solver did not achieve an optimal solution. Status: {0}'.format(self.prob.status)
                        for mapping in self.output_mapping:
                            out_map[mapping] = None

                    else:
                        error_msg = None
                        for mapping in self.output_mapping:
                            out_map[mapping] = self.output_mapping[mapping].value
                    control = out_map['control']
                    state = out_map['state']
                except Exception as e:
                    error_msg = 'Solver encountered an error. {0}'.format(e)
                    for mapping in self.output_mapping:
                            out_map[mapping] = None
                    control = out_map['control']
                    state = out_map['state']

            elif isinstance(self.prob, casadi.Opti):
                solver = solver if solver is not None else "ipopt"
                if solver in ["ipopt"]:
                    if verbose:
                        opts = {'ipopt.print_level': 5, 'print_time': 1}
                    else:
                        opts = {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
                else:
                    if options is None:
                        opts = {'print_time': 0}
                        print("[WARNING] Solver {0} did not get options, using defaults. This can result in unnecessary verbose behavior.\nSee https://web.casadi.org/api/internal/d4/d89/group__nlpsol.html for options.".format(solver))
                    else:
                        opts = options
                self.prob.solver(solver, opts)

                # casadi will raise an exception if solve() detects an infeasible problem
                try:
                    self.prob.set_value(self.x_0, x)
                    self._set_additional_parameters(additional_parameters)
                    sol = self.prob.solve()
                    if sol.stats()['success']:
                        error_msg = None
                        for mapping in self.output_mapping:
                            out_map[mapping] = sol.value(self.output_mapping[mapping])
                    else:
                        error_msg = 'Solver was not successful with return status: {0}'.format(sol.stats()['return_status'])
                        for mapping in self.output_mapping:
                            out_map[mapping] = None
                            
                    control = out_map['control']
                    state = out_map['state']
                except Exception as e:
                    error_msg = 'Solver encountered an error. {0}'.format(e)
                    for mapping in self.output_mapping:
                            out_map[mapping] = None
                    control = out_map['control']
                    state = out_map['state']

            else:
                raise Exception('Optimization problem type not supported!')
        else:
            raise Exception('Optimization problem is not initialized!')
        if len(out_map) == 2:
            return control, state, error_msg
        elif len(out_map) > 2:
            return control, state, out_map, error_msg
        else:
            raise Exception('Output mapping is not defined properly!')


def available_solvers() -> None:
    """
    Print all available solvers for CVXPY and CasADi problems.
    """
    CASADI_SOLVERS = ["ampl", "blocksqp", "bonmin", "fatrop", "ipopt", "knitro", "madnlp",
                      "snopt", "worhp", "qrsqp", "scpgen", "sqpmethod", "feasiblesqpmethod"]

    available_casadi = [False] * len(CASADI_SOLVERS)
    for i,solver in enumerate(CASADI_SOLVERS):
        available_casadi[i] = has_nlpsol(solver)
    available_casadi = list(compress(CASADI_SOLVERS, available_casadi))

    available_cvxpy = cp.installed_solvers()

    print(f'CVXPY available solvers:\n' + \
        f'    {pformat(available_cvxpy, indent=4, width=80)}\n' + \
        f'further information on solvers at:\n' + \
        f'    https://www.cvxpy.org/tutorial/solvers/index.html\n' + \
        f'\n' + \
        f'CasADi available solvers:\n' + \
        f'    {pformat(available_casadi, indent=4, width=80, compact=True)}\n' + \
        f'further information on solvers at:\n' + \
        f'    https://web.casadi.org/api/internal/d4/d89/group__nlpsol.html\n'
    )
