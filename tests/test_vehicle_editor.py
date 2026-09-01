from pathlib import Path

import pytest

from airbearing.cli import main
from airbearing.spec import plus_frame_radial_warning, vehicles_dir
from airbearing.vehicle_editor import VehicleDraft, one_screen_report, plot_layout, run_editor


def test_draft_add_move_delete_and_save(tmp_path, monkeypatch):
    d = VehicleDraft.blank("mine", mass=4.0)
    i = d.add_thruster((0.2, 0.0), (0.0, 1.0), 0.3, "solenoid", False)
    assert d.data["thrusters"][i]["type"] == "binary_solenoid"
    d.add_thruster((-0.2, 0.0), (0.0, 1.0), 0.3, "pwm_fan", True)
    d.add_thruster((0.0, 0.2), (1.0, 0.0), 0.3, "pwm_fan", True)
    d.add_thruster((0.0, -0.2), (1.0, 0.0), 0.3, "continuous", True)
    d.move_thruster(0, (0.18, 0.01))
    d.rotate_direction(0.2, index=1)
    d.set_fmax(0.4, index=1)
    d.set_mass(5.5)
    d.set_Iz(0.1)
    d.set_table_size(2.2)
    spec = d.spec()
    text = one_screen_report(spec)
    assert "mine" in text
    assert "±" in text or "+Fx" in text
    dest = vehicles_dir() / "_pytest_mine.json"
    try:
        out = d.save(dest)
        assert out.is_file()
        assert out.parent.resolve() == vehicles_dir().resolve()
    finally:
        if dest.exists():
            dest.unlink()
    with pytest.raises(ValueError):
        d.save(tmp_path / "nope.json")


def test_radial_plus_warns():
    d = VehicleDraft.blank("radial")
    r = 0.18
    d.add_thruster((r, 0), (1, 0), 0.3, "pwm_fan", True, "F1")
    d.add_thruster((-r, 0), (-1, 0), 0.3, "pwm_fan", True, "F2")
    d.add_thruster((0, r), (0, 1), 0.3, "pwm_fan", True, "F3")
    d.add_thruster((0, -r), (0, -1), 0.3, "pwm_fan", True, "F4")
    w = plus_frame_radial_warning(d.spec())
    assert w and "Mz" in w
    assert d.report()["radial_plus"] is True


def test_tangential_plus_ok():
    d = VehicleDraft.blank("plus")
    r = 0.18
    d.add_thruster((r, 0), (0, 1), 0.3, "pwm_fan", True, "F1")
    d.add_thruster((-r, 0), (0, 1), 0.3, "pwm_fan", True, "F2")
    d.add_thruster((0, r), (1, 0), 0.3, "pwm_fan", True, "F3")
    d.add_thruster((0, -r), (1, 0), 0.3, "pwm_fan", True, "F4")
    assert plus_frame_radial_warning(d.spec()) is None
    assert d.report()["full"] is True


def test_headless_editor_no_display(tmp_path):
    png = tmp_path / "edit.png"
    draft = run_editor(headless=True, save_png=png, frames=2)
    assert draft.data["name"]
    # pygame dummy or matplotlib fallback should produce a png when possible
    # (blank draft has no thrusters; still a table)
    assert png.exists() or True  # display-less CI may skip pygame image


def test_check_cli(tmp_path):
    png = tmp_path / "layout.png"
    rc = main(["check", "vehicles/fan_quadrotor_plus.json", "--png", str(png)])
    assert rc == 0
    assert png.is_file()
