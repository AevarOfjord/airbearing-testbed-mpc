from pathlib import Path

from airbearing.cli import main
from airbearing.labs import run_lab

REPO = Path(__file__).resolve().parents[1]


def test_lab1(tmp_path: Path):
    assert run_lab(1, runs=tmp_path) == 0


def test_lab2(tmp_path: Path):
    assert run_lab(2, runs=tmp_path) == 0


def test_lab3(tmp_path: Path):
    assert run_lab(3, runs=tmp_path) == 0


def test_cli_lab1(tmp_path: Path):
    rc = main(["lab", "1", "--runs", str(tmp_path)])
    assert rc == 0
