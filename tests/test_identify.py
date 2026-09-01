from pathlib import Path

from airbearing.cli import main
from airbearing.identify import identify_from_log, synthesize_excitation_log
from airbearing.spec import load_vehicle

REPO = Path(__file__).resolve().parents[1]


def test_identify_synthetic(tmp_path: Path):
    spec = load_vehicle(REPO / "vehicles" / "fan_quadrotor_plus.json")
    log = tmp_path / "log.csv"
    synthesize_excitation_log(spec, log, duration=5.0)
    out = tmp_path / "fan_quadrotor_plus_identified.json"
    r = identify_from_log(log, REPO / "vehicles" / "fan_quadrotor_plus.json", out=out)
    assert out.is_file()
    assert abs(r["mass"] - spec.mass) / spec.mass < 0.4
    assert r["rmse"] < 2.0
    rc = main(["identify", str(log), "--vehicle", str(REPO / "vehicles" / "fan_quadrotor_plus.json"), "--out", str(out)])
    assert rc == 0
