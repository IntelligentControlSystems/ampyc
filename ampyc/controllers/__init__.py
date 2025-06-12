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

'''Controllers'''
from .controller_base import ControllerBase, available_solvers

from .mpc import MPC
from .nonlinear_mpc import NonlinearMPC

from .robust_mpc import RMPC
from .nonlinear_robust_mpc import NonlinearRMPC

from .ri_smpc import RecoveryInitializationSMPC
from .if_smpc import IndirectFeedbackSMPC


from .constraint_tightening_rmpc import ConstraintTighteningRMPC

from .constraint_tightening_smpc import ConstraintTighteningSMPC

