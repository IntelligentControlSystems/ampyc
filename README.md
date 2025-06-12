## AMPyC
[GitHub](https://github.com/IntelligentControlSystems/ampyc) | [PyPI](https://pypi.org/project/ampyc/) | [Issues](https://github.com/IntelligentControlSystems/ampyc/issues) | [Changelog](https://github.com/IntelligentControlSystems/ampyc/CHANGELOG.md)

``ampyc`` -- *Advanced Model Predictive Control in Python*

General Python package for control theory research, including some reference implementations of various advanced model predictive control (MPC) algorithms.

**Features:**
- Implements dynamical systems and control interfaces to allow seamless interactions
- Provides abstract base classes to allow custom implementation of any type of dynamical system and controller
- Reference implementations of many advanced MPC algorithms; for a full list of implemented algorithms see below
- Global parameter management for easy experiment setup and management
- Various utility tools for set computations, polytope manipulation, and plotting
- [Lecture-style notes](https://github.com/IntelligentControlSystems/ampyc/notes/) and [notebook tutorials](https://github.com/IntelligentControlSystems/ampyc/notebooks/) explaining advanced predictive control concepts


### Installation
``ampyc`` requires Python 3.10 or higher.  Just use [pip](https://pip.pypa.io) for Python 3 to install ``ampyc`` and its dependencies:
```
    python3 -m pip install ampyc
```

**Local (editable) installation:**

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

### Implemented Control Algorithms
| Year | Authors          | Paper                                                                                                                                         | Code                                                                                            |
| :--- | :------------- | :-------------------------------------------------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------------------------------------- |
| 2001 | Chisci et al.         | [Systems with persistent disturbances: predictive control with restricted constraints](https://www.sciencedirect.com/science/article/abs/pii/S0005109801000516)                                   | [ampyc.ct-RMPC](https://github.com/IntelligentControlSystems/ampyc/controllers/constraint_tightening_rmpc.py)                                   |

### Cite this Package \& Developers
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

**Principal Developers:**

&nbsp; [Jerome Sieber](@jsieber) &nbsp; <img src="https://cultofthepartyparrot.com/parrots/hd/hackerparrot.gif" width="25" height="25" /> &nbsp; | &nbsp; [Alex Didier](@adidier) &nbsp; <img src="https://cultofthepartyparrot.com/parrots/schnitzelparrot.gif" width="25" height="25" /> &nbsp; | &nbsp; [Mike Zhang]() &nbsp; <img src="https://cultofthepartyparrot.com/guests/hd/partyfieri.gif" width="25" height="25" /> &nbsp; | &nbsp; [Rahel Rickenbach](@rrahel) &nbsp; <img src="https://cultofthepartyparrot.com/parrots/wave4parrot.gif" width="25" height="25" />