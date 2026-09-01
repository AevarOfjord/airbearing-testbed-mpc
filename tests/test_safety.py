import numpy as np

from airbearing.safety import SafetySupervisor
from airbearing.spec import load_vehicle
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def test_null_telemetry_zeros_and_abort():
    spec = load_vehicle(REPO / "examples" / "vehicles" / "solenoid_octagon.json")
    sup = SafetySupervisor(spec)
    cmd = np.ones(spec.n_thrusters)
    last = None
    for _ in range(spec.n_thrusters * 0 + 20):
        last = sup.review(cmd, None, False, spec.control_dt)
    assert last.overridden
    assert np.allclose(last.cmd, 0)
    assert last.abort


def test_table_edge_abort():
    spec = load_vehicle(REPO / "examples" / "vehicles" / "fan_plus.json")
    sup = SafetySupervisor(spec)
    st = np.array([spec.table_size, 0, 0, 0, 0, 0], dtype=float)
    d = sup.review(np.ones(4), st, True, spec.control_dt)
    assert d.abort
    assert np.allclose(d.cmd, 0)


def test_deadman_expire():
    spec = load_vehicle(REPO / "examples" / "vehicles" / "solenoid_octagon.json")
    sup = SafetySupervisor(spec)
    zeros = sup.expire(1.0)
    assert zeros is not None
    assert np.allclose(zeros, 0)
