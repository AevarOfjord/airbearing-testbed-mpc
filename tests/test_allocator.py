from pathlib import Path

import numpy as np
import pytest

from airbearing.allocator import allocate, pinv_allocate
from airbearing.spec import load_vehicle

REPO = Path(__file__).resolve().parents[1]
FILES = {
    3: "micro_3thruster.json",
    4: "fan_quadrotor_plus.json",
    6: "fan_hex.json",
    8: "uk_solenoid_octagon.json",
}


@pytest.mark.parametrize("n,fname", FILES.items())
def test_allocator_dimension(n, fname):
    spec = load_vehicle(REPO / "vehicles" / fname)
    assert spec.n_thrusters == n
    B = spec.allocation_matrix()
    assert B.shape == (3, n)
    w = np.array([0.05, -0.02, 0.01])
    a = allocate(spec, w)
    assert a.cmd.shape == (n,)
    lo, hi = spec.cmd_bounds()
    assert np.all(a.cmd >= lo - 1e-8)
    assert np.all(a.cmd <= hi + 1e-8)


@pytest.mark.parametrize("n,fname", FILES.items())
def test_zero_wrench_near_zero_cmd(n, fname):
    spec = load_vehicle(REPO / "vehicles" / fname)
    a = allocate(spec, np.zeros(3))
    assert np.linalg.norm(a.cmd) < 0.05


def test_eight_thruster_can_match_small_wrench():
    spec = load_vehicle(REPO / "vehicles" / "uk_solenoid_octagon.json")
    w = np.array([0.4, 0.0, 0.05])
    a = allocate(spec, w)
    assert np.linalg.norm(a.residual) < 0.08


def test_pinv_clips():
    spec = load_vehicle(REPO / "vehicles" / "fan_quadrotor_plus.json")
    u = pinv_allocate(spec, np.array([10.0, 10.0, 10.0]))
    lo, hi = spec.cmd_bounds()
    assert np.all(u <= hi + 1e-12)
    assert np.all(u >= lo - 1e-12)


def test_binary_rounding():
    spec = load_vehicle(REPO / "vehicles" / "uk_solenoid_octagon.json")
    a = allocate(spec, np.array([0.5, 0.0, 0.0]), round_binary=True)
    assert set(np.unique(a.cmd_rounded)).issubset({0.0, 1.0})
