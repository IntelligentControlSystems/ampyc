# AMPyC
[GitHub](https://github.com/IntelligentControlSystems/ampyc) | [PyPI](https://pypi.org/project/ampyc/) | [Issues](https://github.com/IntelligentControlSystems/ampyc/issues) | [Changelog](https://github.com/IntelligentControlSystems/ampyc/blob/main/CHANGELOG.md)

``ampyc`` -- *Advanced Model Predictive Control in Python*

General Python package for control theory research, including some reference implementations of various advanced model predictive control (MPC) algorithms.

**Features:**
- Implements dynamical systems and control interfaces to allow seamless interactions
- Provides abstract base classes to allow custom implementation of any type of dynamical system and controller
- Reference implementations of many advanced MPC algorithms; for a full list of implemented algorithms see below
- Global parameter management for easy experiment setup and management
- Various utility tools for set computations, polytope manipulation, and plotting
- [Lecture-style notes](https://github.com/IntelligentControlSystems/ampyc/tree/main/notes/) and [notebook tutorials](https://github.com/IntelligentControlSystems/ampyc/tree/main/notebooks/) explaining advanced predictive control concepts


## Installation

``ampyc`` requires Python 3.10 or higher.  Just use [pip](https://pip.pypa.io) for Python 3 to install ``ampyc`` and its dependencies:
```
    python3 -m pip install ampyc
```

### Local (editable) installation

1. Clone this repository using
```
    git clone git@github.com:IntelligentControlSystems/ampyc.git
```
2. Install all dependencies (preferably in a [virtual environment](https://docs.python.org/3/library/venv.html)) using
```
    python3 -m pip install -r requirements.txt
```
3. Install ``ampyc`` in editable mode for development. Navigate to this top-level folder and run
```
    pip install -e .
```

## Getting Started
To get started with the ``ampyc`` package, run the [tutorial notebook](https://github.com/IntelligentControlSystems/ampyc/blob/main/notebooks/tutorial.ipynb), which provides an introduction to all parts of the package.

For specific control algorithms implemented in ``ampyc``, run the associated notebook in the [notebook folder](https://github.com/IntelligentControlSystems/ampyc/tree/main/notebooks/).


## Implemented Control Algorithms
| Year | Authors          | Method/Paper                                                                                                                                         | AMPyC                                                                                            |
| :--- | :------------- | :-------------------------------------------------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------------------------------------- |
| - | - | [Linear Model Predictive Control](https://sites.engineering.ucsb.edu/~jbraw/mpc/MPC-book-2nd-edition-1st-printing.pdf) | [code](https://github.com/IntelligentControlSystems/ampyc/blob/main/ampyc/controllers/mpc.py) |
| - | - | [Nonlinear Model Predictive Control](https://sites.engineering.ucsb.edu/~jbraw/mpc/MPC-book-2nd-edition-1st-printing.pdf) | [code](https://github.com/IntelligentControlSystems/ampyc/blob/main/ampyc/controllers/nonlinear_mpc.py) |
| 2001 | Chisci et al. | [Systems with persistent disturbances: predictive control with restricted constraints](https://www.sciencedirect.com/science/article/abs/pii/S0005109801000516) | [code](https://github.com/IntelligentControlSystems/ampyc/blob/main/ampyc/controllers/constraint_tightening_rmpc.py) |
| 2005 | Mayne et al. | [Robust model predictive control of constrained linear systems with bounded disturbances](https://www.sciencedirect.com/science/article/abs/pii/S0005109804002870) | [code](https://github.com/IntelligentControlSystems/ampyc/blob/main/ampyc/controllers/robust_mpc.py) |
| 2013 | Bayer et al. | [Discrete-time incremental ISS: A framework for robust NMPC](https://ieeexplore.ieee.org/document/6669322) | [code](https://github.com/IntelligentControlSystems/ampyc/blob/main/ampyc/controllers/nonlinear_robust_mpc.py) |
| 2017 | Lorenzen et al. | [Constraint-Tightening and Stability in Stochastic Model Predictive Control](https://arxiv.org/pdf/1511.03488) | [code](https://github.com/IntelligentControlSystems/ampyc/blob/main/ampyc/controllers/constraint_tightening_smpc.py) |
| 2018 | Hewing \& Zeilinger | [Stochastic Model Predictive Control for Linear Systems Using Probabilistic Reachable Sets](https://arxiv.org/pdf/1805.07145) | [code](https://github.com/IntelligentControlSystems/ampyc/blob/main/ampyc/controllers/if_smpc.py) |
| 2020 | Hewing et al. | [Recursively feasible stochastic model predictive control using indirect feedback](https://arxiv.org/pdf/1812.06860) | [code](https://github.com/IntelligentControlSystems/ampyc/blob/main/ampyc/controllers/ri_smpc.py) |

## Cite this Package \& Developers
If you find this package/repository helpful, please cite our work:
```bib
@software{ampyc,
  title  = {AMPyC: Advanced Model Predictive Control in Python},
  author = {Sieber, Jerome and Didier, Alexandre and Rickenbach, Rahel and Zeilinger, Melanie},
  url    = {https://github.com/IntelligentControlSystems/ampyc},
  month  = jun,
  year   = {2025}
}
```

### Principal Developers

&nbsp; [Jerome Sieber](https://github.com/jsie7) &nbsp; <img src="https://cultofthepartyparrot.com/parrots/hd/hackerparrot.gif" width="25" height="25" /> &nbsp; | &nbsp; [Alex Didier](https://github.com/alexdidier) &nbsp; <img src="https://cultofthepartyparrot.com/parrots/schnitzelparrot.gif" width="25" height="25" /> &nbsp; | &nbsp; [Mike Zhang]() &nbsp; <img src="https://cultofthepartyparrot.com/guests/hd/partyfieri.gif" width="25" height="25" /> &nbsp; | &nbsp; [Rahel Rickenbach](mailto:rrahel@ethz.ch) &nbsp; <img src="https://cultofthepartyparrot.com/parrots/wave4parrot.gif" width="25" height="25" />