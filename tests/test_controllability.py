from pathlib import Path

from airbearing.spec import controllability_report, load_vehicle

REPO = Path(__file__).resolve().parents[1]


def test_octagon_full():
    spec = load_vehicle(REPO / "vehicles" / "uk_solenoid_octagon.json")
    r = controllability_report(spec)
    assert r["full"] is True
    assert r["rank_B"] == 3


def test_fan_plus_full():
    spec = load_vehicle(REPO / "vehicles" / "fan_quadrotor_plus.json")
    r = controllability_report(spec)
    assert r["full"] is True


def test_hex_full():
    spec = load_vehicle(REPO / "vehicles" / "fan_hex.json")
    r = controllability_report(spec)
    assert r["full"] is True


def test_three_unidirectional_not_full():
    spec = load_vehicle(REPO / "vehicles" / "micro_3thruster.json")
    r = controllability_report(spec)
    assert r["n_thrusters"] == 3
    assert r["full"] is False
    assert r["warning"]
