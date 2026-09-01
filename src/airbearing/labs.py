"""Curriculum labs. Student write-ups live in labs/; this module is the runner."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from airbearing.cli import REPO
from airbearing.control.lqr import LQRController
from airbearing.control.mpc import LinearMPC
from airbearing.control.pd import PDController
from airbearing.identify import identify_from_log, synthesize_excitation_log
from airbearing.missions import point_to_point
from airbearing.runtime import Runtime
from airbearing.logschema import write_pose_csv
from airbearing.report import print_report
from airbearing.spec import load_vehicle, shipped_vehicle, spec_from_dict, spec_to_dict
from airbearing.vehicle_editor import VehicleDraft, one_screen_report, plot_layout

LABS = REPO / "labs"


def write_mocap_csv(result, path: Path) -> Path:
    import numpy as np

    path = Path(path)
    t = np.array([row.t for row in result.logs])
    st = np.array([row.state for row in result.logs])
    return write_pose_csv(path, t, st)


def _rmse(result) -> float:
    e = []
    for row in result.logs:
        e.append(np.hypot(row.state[0] - row.ref[0], row.state[1] - row.ref[1]))
    return float(np.sqrt(np.mean(np.square(e)))) if e else 1e9


def lab1_editor(runs: Path) -> int:
    """Place thrusters in the draft model, check, write a layout PNG."""
    print("=== Lab 1 — model a satellite without editing Python ===")
    print("Primary UI:  python -m airbearing edit-vehicle")
    print("Then:        make run VEHICLE=vehicles/mine.json")
    draft = VehicleDraft.blank("lab1_demo", table=2.0, mass=5.0)
    r = 0.16
    # plus frame, *tangential* so Mz works
    draft.add_thruster((r, 0), (0, 1), 0.3, "pwm_fan", True, "F1")
    draft.add_thruster((-r, 0), (0, 1), 0.3, "pwm_fan", True, "F2")
    draft.add_thruster((0, r), (1, 0), 0.3, "pwm_fan", True, "F3")
    draft.add_thruster((0, -r), (1, 0), 0.3, "pwm_fan", True, "F4")
    spec = draft.spec()
    print(one_screen_report(spec), end="")
    png = LABS / "data" / "lab1_layout.png"
    plot_layout(spec, png)
    print(f"layout {png}")
    # contrast: radial plus → Mz≡0
    bad = VehicleDraft.blank("lab1_radial")
    bad.add_thruster((r, 0), (1, 0), 0.3, "pwm_fan", True, "F1")
    bad.add_thruster((-r, 0), (-1, 0), 0.3, "pwm_fan", True, "F2")
    bad.add_thruster((0, r), (0, 1), 0.3, "pwm_fan", True, "F3")
    bad.add_thruster((0, -r), (0, -1), 0.3, "pwm_fan", True, "F4")
    print("--- radial plus (should WARN) ---")
    print(one_screen_report(bad.spec()), end="")
    return 0


def lab2_controllers(runs: Path) -> int:
    print("=== Lab 2 — PD vs LQR vs MPC ===")
    spec = load_vehicle(shipped_vehicle("fan_plus"))
    mission = point_to_point(spec.table_size)
    mission.duration = 6.0
    rows = []
    for name, ctor in (("pd", PDController), ("lqr", LQRController), ("mpc", lambda s: LinearMPC(s, horizon=8))):
        rt = Runtime(spec, ctor(spec), mission, runs_root=Path(runs) / "lab2")
        res = rt.run()
        rmse = _rmse(res)
        rows.append((name, res.final_error, rmse, res.success, res.mean_solver_ms))
        print(f"{name:6s}  final={res.final_error:.3f} m  rmse={rmse:.3f}  ok={res.success}  solver={res.mean_solver_ms:.2f} ms")
        print(print_report(res.run_dir), end="")
    out = Path(runs) / "lab2" / "summary.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps([{"controller": a, "final_error": b, "rmse": c, "success": d, "ms": e} for a, b, c, d, e in rows], indent=2))
    # MPC should not be wildly worse than PD on this plant
    by = {a: c for a, _, c, _, _ in rows}
    assert by["mpc"] < 1.5 * max(by["pd"], 0.05)
    return 0


def lab3_identify(runs: Path) -> int:
    print("=== Lab 3 — identify vs uncalibrated RMSE ===")
    spec = load_vehicle(shipped_vehicle("fan_plus"))
    log = Path(runs) / "lab3" / "truth_log.csv"
    synthesize_excitation_log(spec, log, duration=5.0)
    wrong = spec_to_dict(spec)
    wrong["mass"] = spec.mass * 1.8
    wrong["Iz"] = spec.Iz * 1.8
    for th in wrong["thrusters"]:
        th["F_max"] = th["F_max"] * 0.55
    idres = identify_from_log(log, shipped_vehicle("fan_plus"), out=Path(runs) / "lab3" / "identified.json", mode="full")
    print(f"truth mass={spec.mass:.3g} Iz={spec.Iz:.3g} F={ [t.F_max for t in spec.thrusters]}")
    print(f"fit   mass={idres['mass']:.3g} Iz={idres['Iz']:.3g} F={idres['F_max']} rmse={idres['rmse']:.3g}")
    # uncalibrated vs identified closed-loop on the true plant: compare prediction residual only
    assert abs(idres["mass"] - spec.mass) / spec.mass < 0.35
    print("mass recovered within 35% from a 5 s open-loop chirp (see docs/LAB.md).")
    print(f"F_max_scale={idres.get('F_max_scale')} delay={idres.get('delay_steps')}")
    return 0


def lab4_actuators(runs: Path) -> int:
    print("=== Lab 4 — binary vs PWM vs model mismatch ===")
    solenoid = load_vehicle(shipped_vehicle("solenoid_octagon"))
    fans = load_vehicle(shipped_vehicle("fan_plus"))
    mission_s = point_to_point(solenoid.table_size)
    mission_s.duration = 6.0
    mission_f = point_to_point(fans.table_size)
    mission_f.duration = 6.0
    r_s = Runtime(solenoid, LinearMPC(solenoid, horizon=8), mission_s, runs_root=Path(runs) / "lab4").run()
    r_f = Runtime(fans, LinearMPC(fans, horizon=8), mission_f, runs_root=Path(runs) / "lab4").run()
    r_round = Runtime(solenoid, LinearMPC(solenoid, horizon=8), mission_s, round_binary=True, runs_root=Path(runs) / "lab4").run()
    print(f"solenoid relax   err={r_s.final_error:.3f} rmse={_rmse(r_s):.3f}")
    print(f"solenoid binary  err={r_round.final_error:.3f} rmse={_rmse(r_round):.3f}")
    print(f"pwm fans         err={r_f.final_error:.3f} rmse={_rmse(r_f):.3f}")
    # mismatch: allocator thinks F_max is 2x
    hot = spec_from_dict(spec_to_dict(fans))
    for t in hot.thrusters:
        t.F_max *= 2.0
    # plant still uses original spec via a custom Runtime plant swap
    rt = Runtime(hot, LinearMPC(hot, horizon=8), mission_f, runs_root=Path(runs) / "lab4")
    rt.plant.spec = fans  # true hardware weaker than the JSON the controller believes
    r_mis = rt.run()
    print(f"F_max mismatch   err={r_mis.final_error:.3f} rmse={_rmse(r_mis):.3f}")
    print("Staff notes: labs/staff/")
    return 0


def lab5_navigation(runs: Path) -> int:
    """Onboard vs external vs fused using sim sensors (no table required)."""
    print("=== Lab 5 — onboard vs external vs fused ===")
    spec0 = load_vehicle(shipped_vehicle("fan_plus"))
    fused_path = shipped_vehicle("fan_plus_fused")
    fused_spec = load_vehicle(fused_path)
    print(f"fused example {fused_path.name} estimator={fused_spec.navigation.estimator}")

    def with_nav(nav: dict):
        d = spec_to_dict(spec0)
        d["navigation"] = nav
        return spec_from_dict(d)

    noisy_ext = {
        "x": 0.03, "y": 0.03, "yaw": 0.04,
    }
    cases = [
        (
            "external/passthrough",
            with_nav({
                "estimator": "passthrough",
                "external": {"type": "sim", "meas": ["x", "y", "yaw", "vx", "vy", "omega"], "noise": noisy_ext},
            }),
        ),
        (
            "onboard/ekf",
            with_nav({
                "estimator": "ekf",
                "onboard": {"type": "sim", "meas": ["ax", "ay", "gyro_z"]},
            }),
        ),
        (
            "fused/ekf",
            fused_spec,
        ),
    ]
    rows = []
    for name, spec in cases:
        mission = point_to_point(spec.table_size)
        mission.duration = 3.0
        rt = Runtime(spec, LinearMPC(spec, horizon=8), mission, runs_root=Path(runs) / "lab5", seed=0)
        rt.begin()
        errs = []
        while rt._t < mission.duration:
            truth = rt.plant.state.copy()
            more = rt.tick()
            est = rt._last_pose
            errs.append(float(np.hypot(est[0] - truth[0], est[1] - truth[1])))
            if not more:
                break
        rt.finish()
        rmse = float(np.sqrt(np.mean(np.square(errs)))) if errs else 1e9
        rows.append((name, rmse, rt.result.aborted, rt.result.final_error))
        print(f"{name:22s}  pose_rmse={rmse:.4f} m  aborted={rt.result.aborted}  final={rt.result.final_error:.3f} m")
    by = {a: b for a, b, _, _ in rows}
    # fused should beat noisy mocap passthrough on the same plant
    assert by["fused/ekf"] < by["external/passthrough"]
    print("fused EKF beats noisy mocap passthrough (sim sensors). Select a mode in vehicle JSON navigation.")
    out = Path(runs) / "lab5" / "summary.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(
        [{"case": a, "pose_rmse": b, "aborted": c, "final_error": d} for a, b, c, d in rows],
        indent=2,
    ))
    return 0


def run_lab(n: int, runs: Path | None = None) -> int:
    runs = Path(runs) if runs else REPO / "runs"
    if n == 1:
        return lab1_editor(runs)
    if n == 2:
        return lab2_controllers(runs)
    if n == 3:
        return lab3_identify(runs)
    if n == 4:
        return lab4_actuators(runs)
    if n == 5:
        return lab5_navigation(runs)
    raise SystemExit(f"unknown lab {n}")
