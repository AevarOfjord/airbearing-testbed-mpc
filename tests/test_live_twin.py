from pathlib import Path

from airbearing.cli import main
from airbearing.labs import write_mocap_csv
from airbearing.control.mpc import LinearMPC
from airbearing.missions import point_to_point
from airbearing.runtime import Runtime
from airbearing.spec import load_vehicle

REPO = Path(__file__).resolve().parents[1]


def test_view_record(tmp_path: Path):
    png = tmp_path / "live_twin.png"
    gif = tmp_path / "live_twin.gif"
    rc = main([
        "view", "--record", "--duration", "1.2",
        "--vehicle", str(REPO / "vehicles" / "fan_quadrotor_plus.json"),
        "--out", str(png), "--gif", str(gif),
        "--runs", str(tmp_path / "runs"),
    ])
    assert rc == 0
    assert png.is_file() and png.stat().st_size > 100


def test_replay_mocap(tmp_path: Path):
    spec = load_vehicle(REPO / "vehicles" / "fan_quadrotor_plus.json")
    mission = point_to_point(spec.table_size)
    mission.duration = 2.0
    res = Runtime(spec, LinearMPC(spec, horizon=6), mission, runs_root=tmp_path).run()
    csv = tmp_path / "example_mocap.csv"
    write_mocap_csv(res, csv)
    rt = Runtime(spec, LinearMPC(spec, horizon=6), mission, replay=csv, runs_root=tmp_path / "rep")
    out = rt.run()
    assert len(out.logs) >= 5
    # replayed first pose matches the recording
    assert abs(out.logs[0].state[0] - res.logs[0].state[0]) < 1e-6
