import pytest
import numpy as np
from ampyc.params import MPCParams

def assert_system_correct(params: MPCParams):
    sys = params.sys
    assert np.all(sys.A == np.array([
        [1, sys.dt],
        [sys.dt*(-sys.k + sys.g/sys.l), 1 - sys.dt*sys.c],
    ]))
    assert np.all(sys.B == np.array([[0], [sys.dt]]))

DEFAULT_g = 9.81

def test_default_params():
    params = MPCParams()
    assert params.sys.dt == 0.1
    assert_system_correct(params)
    assert params.sys.g == DEFAULT_g

@pytest.mark.parametrize("N,dt,l,num_traj", [
    (15, 0.25, 2.5, 2),
])
def test_partial_paramsparams_init(N: int, dt: float, l: float, num_traj: int):
    params_init = {
        'ctrl': {
            'N': N,
        },
        'sys': {
            'dt': dt,
            "l": l,
        },
        'sim': {
            'num_traj': num_traj,
        },
    }
    params = MPCParams(**params_init)
    assert_system_correct(params)
    assert params.sys.g == DEFAULT_g, "Non-specified parameters should not have changed"
    assert params.sys.dt == dt, "Specified override parameter should have changed"

