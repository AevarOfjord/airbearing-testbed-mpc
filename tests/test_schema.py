import json
from pathlib import Path

import pytest

from airbearing.spec import load_vehicle, validate_dict

REPO = Path(__file__).resolve().parents[1]


def _vehicle_files():
    return sorted((REPO / "vehicles").glob("*.json"))


def test_example_exists():
    assert (REPO / "vehicles" / "YOUR_SATELLITE.json.example").is_file()


@pytest.mark.parametrize("path", _vehicle_files(), ids=lambda p: p.name)
def test_shipped_vehicles_validate(path: Path):
    spec = load_vehicle(path)
    assert spec.n_thrusters >= 3
    assert spec.mass > 0


def test_rejects_unknown_fields():
    raw = json.loads((REPO / "vehicles" / "fan_quadrotor_plus.json").read_text())
    raw["gurobi_dll"] = "nope"
    with pytest.raises(Exception):
        validate_dict(raw)


def test_rejects_missing_solenoid_pulse():
    raw = json.loads((REPO / "vehicles" / "uk_solenoid_octagon.json").read_text())
    del raw["thrusters"][0]["min_pulse_ms"]
    with pytest.raises(Exception):
        validate_dict(raw)


def test_uk_is_eight_solenoids():
    spec = load_vehicle(REPO / "vehicles" / "uk_solenoid_octagon.json")
    assert spec.n_thrusters == 8
    assert all(t.type == "binary_solenoid" for t in spec.thrusters)


def test_micro_is_three():
    spec = load_vehicle(REPO / "vehicles" / "micro_3thruster.json")
    assert spec.n_thrusters == 3


def test_vehicle_schema_is_sot():
    assert (REPO / "schemas" / "vehicle.schema.json").is_file()
    from airbearing.spec import _schema_file
    assert _schema_file().name == "vehicle.schema.json"
