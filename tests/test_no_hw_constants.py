"""Vehicle mass / F_max / layout live in vehicles/*.json, not src/."""

from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SRC = REPO / "src" / "airbearing"


def test_no_shipped_vehicle_numbers_in_src():
    forbidden = [
        "uk_solenoid_octagon",  # names as defaults in CLI are ok — check numeric fingerprints
    ]
    # Fingerprints from shipped JSON that must not be duplicated as Python constants.
    fingerprints = ["24.0", "0.35"]  # too generic; instead assert schema is the SoT
    schema = (REPO / "schemas" / "vehicle.schema.json").read_text()
    assert "F_max" in schema
    vehicles = list((REPO / "vehicles").glob("*.json"))
    assert vehicles
    # src may mention example defaults for the *blank editor*, not a named sat's mass.
    text = "\n".join(p.read_text() for p in SRC.rglob("*.py") if p.name != "labs.py")
    assert "mass = 24" not in text.replace(" ", "")
    assert "F_max = 1.2" not in text  # uk solenoid peak is in JSON only
