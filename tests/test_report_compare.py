from pathlib import Path

from airbearing.cli import main
from airbearing.control.mpc import LinearMPC
from airbearing.logschema import LogSchemaError, load_log
from airbearing.missions import point_to_point
from airbearing.runtime import Runtime
from airbearing.spec import load_vehicle

REPO = Path(__file__).resolve().parents[1]
FAN = REPO / "examples" / "vehicles" / "fan_plus.json"


def test_run_writes_methods_summary(tmp_path: Path):
    spec = load_vehicle(FAN)
    mission = point_to_point(spec.table_size)
    mission.duration = 2.0
    res = Runtime(spec, LinearMPC(spec, horizon=6), mission, runs_root=tmp_path, seed=7).run()
    assert (res.run_dir / "summary.json").is_file()
    assert (res.run_dir / "methods.txt").is_file()
    text = (res.run_dir / "methods.txt").read_text()
    assert "settling_s" in text
    assert "solver_p50_ms" in text
    assert "vehicle_sha256" in text
    log = load_log(res.run_dir / "log.csv")
    assert log["schema_version"] == 1
    rc = main(["report", str(res.run_dir)])
    assert rc == 0


def test_log_refuses_mismatched_columns(tmp_path: Path):
    bad = tmp_path / "bad.csv"
    bad.write_text("# airbearing_log schema_version=1 units=SI\nfoo,bar\n1,2\n")
    try:
        load_log(bad)
        assert False, "expected LogSchemaError"
    except LogSchemaError:
        pass


def test_compare_sim_vs_replay(tmp_path: Path):
    spec = load_vehicle(FAN)
    mission = point_to_point(spec.table_size)
    mission.duration = 2.0
    a = Runtime(spec, LinearMPC(spec, horizon=6), mission, runs_root=tmp_path / "a").run()
    b = Runtime(spec, LinearMPC(spec, horizon=6), mission, runs_root=tmp_path / "b").run()
    rc = main([
        "compare",
        "--sim", str(a.run_dir / "log.csv"),
        "--real", str(b.run_dir / "log.csv"),
        "--mismatch-delay", "1",
        "--out", str(tmp_path / "cmp.json"),
    ])
    assert rc == 0
    assert (tmp_path / "cmp.json").is_file()
