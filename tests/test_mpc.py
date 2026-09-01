from pathlib import Path

import numpy as np

from airbearing.control.mpc import LinearMPC
from airbearing.control.pd import PDController
from airbearing.control.lqr import LQRController
from airbearing.spec import load_vehicle

REPO = Path(__file__).resolve().parents[1]


def test_mpc_feasible_octagon():
    spec = load_vehicle(REPO / "vehicles" / "uk_solenoid_octagon.json")
    mpc = LinearMPC(spec, horizon=8)
    x = np.array([-0.6, 0.1, 0.2, 0.0, 0.0, 0.0])
    ref = np.array([0.6, 0.0, 0.0])
    out = mpc.compute(x, ref)
    assert out.feasible
    assert out.wrench.shape == (3,)
    assert np.isfinite(out.wrench).all()
    # should push +x in body (theta small) to go toward +0.6
    body_Fx = out.wrench[0]
    assert body_Fx > 0.0


def test_mpc_feasible_fans():
    spec = load_vehicle(REPO / "vehicles" / "fan_quadrotor_plus.json")
    mpc = LinearMPC(spec, horizon=8)
    x = np.zeros(6)
    x[0] = -0.3
    out = mpc.compute(x, np.array([0.3, 0.0, 0.0]))
    assert out.feasible


def test_pd_and_lqr_shapes():
    spec = load_vehicle(REPO / "vehicles" / "fan_hex.json")
    x = np.array([0.2, -0.1, 0.3, 0.01, 0.0, 0.0])
    ref = np.zeros(3)
    for ctrl in (PDController(spec), LQRController(spec)):
        o = ctrl.compute(x, ref)
        assert o.wrench.shape == (3,)
