from pathlib import Path

import numpy as np

from airbearing.control.mpc import LinearMPC
from airbearing.missions import point_to_point
from airbearing.runtime import Runtime
from airbearing.spec import load_vehicle

REPO = Path(__file__).resolve().parents[1]


def _run(vehicle: str, duration: float = 7.0):
    spec = load_vehicle(REPO / "vehicles" / vehicle)
    mission = point_to_point(spec.table_size)
    mission.duration = duration
    rt = Runtime(spec, LinearMPC(spec, horizon=8), mission, runs_root=REPO / "runs" / "pytest")
    return spec, mission, rt.run()


def test_solenoid_closed_loop_progress(tmp_path=None):
    spec, mission, res = _run("uk_solenoid_octagon.json")
    assert not res.aborted
    assert len(res.logs) > 10
    start = np.hypot(res.logs[0].state[0] - mission.goal[0], res.logs[0].state[1] - mission.goal[1])
    assert res.final_error < 0.7 * start
    assert res.mean_solver_ms < spec.control_dt * 1000 * 4  # generous on CI


def test_fan_closed_loop_progress():
    spec, mission, res = _run("fan_quadrotor_plus.json", duration=6.0)
    assert not res.aborted
    start = np.hypot(res.logs[0].state[0] - mission.goal[0], res.logs[0].state[1] - mission.goal[1])
    assert res.final_error < 0.6 * start


def test_csv_written():
    _, _, res = _run("fan_quadrotor_plus.json", duration=2.0)
    assert (res.run_dir / "log.csv").is_file()
    assert (res.run_dir / "summary.json").is_file()
