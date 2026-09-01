"""Hardware numbers live in example/student JSON, not in src/ (except tests)."""

from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SRC = REPO / "src" / "airbearing"


def test_no_shipped_vehicle_numbers_in_src():
    schema = (REPO / "schemas" / "vehicle.schema.json").read_text()
    assert "F_max" in schema
    examples = list((REPO / "examples" / "vehicles").glob("*.json"))
    assert examples
    assert (REPO / "vehicles").is_dir()
    text = "\n".join(p.read_text() for p in SRC.rglob("*.py") if p.name != "labs.py")
    assert "mass = 24" not in text.replace(" ", "")
    assert "F_max = 1.2" not in text
    assert "uk_solenoid" not in text
    assert "COM20" not in text
    # example *filenames* may be referenced; numeric F_max from solenoid JSON is 0.7
    assert "F_max = 0.7" not in text
