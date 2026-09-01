from pathlib import Path

from airbearing.cli import main


def test_wizard_four_fans(tmp_path: Path):
    out = tmp_path / "my_fans.json"
    rc = main([
        "new-vehicle", "--noninteractive",
        "--name", "my_fans", "--n", "4", "--type", "pwm_fan",
        "--bidirectional", "--mass", "5", "--table", "2",
        "--fmax", "0.3", "--out", str(out),
    ])
    assert rc == 0
    assert out.is_file()
    text = out.read_text()
    assert "pwm_fan" in text
    assert text.count('"id"') == 4


def test_wizard_three_warns(tmp_path: Path):
    out = tmp_path / "tri.json"
    rc = main([
        "new-vehicle", "--noninteractive",
        "--name", "tri", "--n", "3", "--type", "binary_solenoid",
        "--no-bidirectional", "--out", str(out),
    ])
    assert rc == 2
    assert out.is_file()
