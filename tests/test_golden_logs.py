"""Shipped examples/logs must make `airbearing compare` work on a fresh clone."""
from pathlib import Path

from airbearing.cli import main
from airbearing.compare import compare_logs

REPO = Path(__file__).resolve().parents[1]
SIM = REPO / "examples" / "logs" / "sim.csv"
REAL = REPO / "examples" / "logs" / "hardware.csv"


def test_golden_logs_exist():
    assert SIM.is_file(), f"missing {SIM} (commit a small CSV so clone works offline)"
    assert REAL.is_file(), f"missing {REAL}"


def test_compare_cli_on_golden_logs(capsys):
    rc = main(["compare", "--sim", str(SIM), "--real", str(REAL)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "RMSE" in out
    assert "position" in out.lower()


def test_compare_logs_reports_finite_rmse():
    rep = compare_logs(SIM, REAL)
    assert rep["rmse_position_m"] >= 0.0
    assert rep["rmse_yaw_rad"] >= 0.0
    assert rep["n"] >= 8
    assert rep["t1"] > rep["t0"]
