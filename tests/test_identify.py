from pathlib import Path

from airbearing.cli import main
from airbearing.identify import identify_from_log, synthesize_excitation_log
from airbearing.spec import load_vehicle, spec_to_dict, save_vehicle

REPO = Path(__file__).resolve().parents[1]
FAN = REPO / "examples" / "vehicles" / "fan_plus.json"


def test_identify_fmax_scale(tmp_path: Path):
    spec = load_vehicle(FAN)
    log = tmp_path / "log.csv"
    synthesize_excitation_log(spec, log, duration=5.0, kind="chirp")
    wrong = spec_to_dict(spec)
    for th in wrong["thrusters"]:
        th["F_max"] = th["F_max"] * 0.5
    wrong_path = tmp_path / "wrong.json"
    save_vehicle(wrong, wrong_path, allow_any=True)
    out = tmp_path / "id.json"
    png = tmp_path / "residual.png"
    r = identify_from_log(log, wrong_path, out=out, mode="fmax_scale", delay_steps=0, residual_png=png)
    assert out.is_file()
    assert png.is_file()
    assert abs(r["F_max_scale"] - 2.0) / 2.0 < 0.35
    assert r["rmse"] < 2.0


def test_identify_full_mass(tmp_path: Path):
    spec = load_vehicle(FAN)
    log = tmp_path / "log.csv"
    synthesize_excitation_log(spec, log, duration=5.0)
    out = tmp_path / "fan_plus_identified.json"
    r = identify_from_log(log, FAN, out=out, mode="full", delay_steps=0)
    assert abs(r["mass"] - spec.mass) / spec.mass < 0.4
    rc = main([
        "identify", str(log), "--vehicle", str(FAN), "--out", str(out),
        "--mode", "fmax_scale", "--delay-steps", "0", "--residual", str(tmp_path / "r.png"),
    ])
    assert rc == 0


def test_identify_prbs_and_delay(tmp_path: Path):
    spec = load_vehicle(FAN)
    log = tmp_path / "prbs.csv"
    synthesize_excitation_log(spec, log, duration=4.0, kind="prbs", delay_steps=1)
    r = identify_from_log(log, FAN, mode="fmax_scale", delay_steps="auto")
    assert r["delay_steps"] in (0, 1, 2)
    assert r["success"]
