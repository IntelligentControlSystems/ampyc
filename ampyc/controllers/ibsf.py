'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

from .controller_base import ControllerBase
import cvxpy as cp
import numpy as np

class IBSF(ControllerBase):
    '''
    Implements a standard invariance-based safety filter, see e.g.,:

    K. P. Wabersich, and M. N. Zeilinger. "Scalable synthesis of safety certificates from data with application to learning-based control." 2018 European control conference (ECC). IEEE, 2018.

    More information is provided in the accompanying notes:
    https://github.com/IntelligentControlSystems/ampyc/notes/08_safetyFilter1.pdf
    '''

    def __init__(self, sys, params):
        super().__init__(sys, params)
        
    def _init_problem(self, sys, params):
        # define optimization variables
        self.E = cp.Variable((sys.n, sys.n), symmetric=True)
        self.Y = cp.Variable((sys.m, sys.n))

        # define the objective
        objective = -cp.log_det(self.E)

        # define the constraints
        constraints = [ self.E >> 0,
                        cp.bmat([[self.E, (sys.A@self.E+sys.B@self.Y).T], 
                                 [sys.A@self.E+sys.B@self.Y, self.E]]) >> 0 ]
        
        for i, A_i in enumerate(sys.X.A):
            constraints += [A_i.reshape(1,-1)@self.E@A_i.reshape(1,-1).T <= sys.X.b[i]**2]
            
        for i, A_i in enumerate(sys.U.A):
            constraints += [cp.bmat([[sys.U.b[i].reshape(1,1)**2, A_i.reshape(1,-1)@self.Y],
                                     [self.Y.T@A_i.reshape(1,-1).T, self.E]]) >> 0]
            

        # define the CVX optimization problem object
        self.prob = cp.Problem(cp.Minimize(objective), constraints)
         
        # solve the SDP
        self.prob.solve(verbose=False, solver=None)
                            
        # Recover the ellipsoid shape and feedback matrices
        self.P = np.linalg.inv(self.E.value)
        self.K = self.Y.value @ self.P

    def _define_output_mapping(self):
        # IBSF is not an MPC controller, so we don't have planned trajectories
        return {
            'control': None,
            'state': None
        }
                            
    def solve(self, sys, x, u):
        # since the IBSF is not an MPC controller, we need to overwrite the solve method

        # if ((x.T@self.P@x <= 1) and all(sys.U.A@u <= sys.U.b)):
        #     return u
        # else:
        #     return self.K@x


        x_next = sys.A@x+sys.B@u
        if ((x_next.T@self.P@x_next <= 1) and all(sys.U.A@u <= sys.U.b)):
            return u
        else:
            return self.K@x
        

class MinIBSF(ControllerBase):
    '''
    Implements a minimally invasive invariance-based safety filter, see e.g.,:

    K. P. Wabersich, and M. N. Zeilinger. "Linear model predictive safety certification for learning-based control." 2018 IEEE Conference on Decision and Control (CDC). IEEE, 2018.
    
    More information is provided in the accompanying notes:
    https://github.com/IntelligentControlSystems/ampyc/notes/09_safetyFilter2.pdf
    '''

    def __init__(self, sys, P, params):
        self.P = P
        super().__init__(sys, params)
        
    def _init_problem(self, sys, params):        
        self.x = cp.Variable((sys.n, 2))
        self.u = cp.Variable((sys.m))
        self.x_0 = cp.Parameter((sys.n))
        self.u_L = cp.Parameter((sys.m))

        # define the objective
        objective = cp.norm(self.u_L-self.u,2)**2

        # define the constraints
        constraints = [self.x[:, 0] == self.x_0]
        constraints += [self.x[:, 1] == sys.A @ self.x[:, 0] + sys.B @ self.u]
        constraints += [sys.U.A @ self.u <= sys.U.b]
        constraints += [cp.quad_form(self.x[:, 1], self.P) <= 1.0]

        # define the CVX optimization problem object
        self.prob = cp.Problem(cp.Minimize(objective), constraints)

    def _set_additional_parameters(self, additional_parameters):
        self.u_L.value = additional_parameters['u_L']

    def _define_output_mapping(self):
        return {
            'control': self.u,
            'state': self.x
        }


class DampIBSF(ControllerBase):
    '''
    Implements an invariance-based safety filter with control barrier function dampening constraint, see e.g.,:

    A. Agrawal, and K. Sreenath. "Discrete control barrier functions for safety-critical control of discrete systems with application to bipedal robot navigation." Robotics: Science and Systems. Vol. 13. 2017.    
    
    More information is provided in the accompanying notes:
    https://github.com/IntelligentControlSystems/ampyc/notes/09_safetyFilter2.pdf
    '''

    def __init__(self, sys, P, params, gamma=1):
        self.P = P
        self.gamma = gamma
        super().__init__(sys, params)
        
    def _init_problem(self, sys, params):  

        self.x = cp.Variable((sys.n, 2))
        self.u = cp.Variable((sys.m))
        self.x_0 = cp.Parameter((sys.n))
        self.u_L = cp.Parameter((sys.m))
        self.V_x_0 = cp.Parameter()

        # define the objective
        objective = cp.norm(self.u_L-self.u,2)**2

        # define the constraints
        constraints = [self.x[:, 0] == self.x_0]
        constraints += [self.x[:, 1] == sys.A @ self.x[:, 0] + sys.B @ self.u]
        constraints += [sys.U.A @ self.u <= sys.U.b]
        constraints += [cp.quad_form(self.x[:, 1], self.P) - self.V_x_0 <= self.gamma*(1 - self.V_x_0)]

        # define the CVX optimization problem object
        self.prob = cp.Problem(cp.Minimize(objective), constraints)

    def _set_additional_parameters(self, additional_parameters):
        self.u_L.value = additional_parameters['u_L']
        self.V_x_0.value = additional_parameters['V_x_0']

    def _define_output_mapping(self):
        return {
            'control': self.u,
            'state': self.x
        }
