"""Student-facing CLI: run, compare-actuators, new-vehicle."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np

from airbearing.control.lqr import LQRController
from airbearing.control.mpc import LinearMPC
from airbearing.control.pd import PDController
from airbearing.missions import point_to_point
from airbearing.runtime import Runtime
from airbearing.spec import controllability_report, load_vehicle, spec_from_dict
from airbearing.viz import animate, plot_compare, plot_trajectory

REPO = Path(__file__).resolve().parents[2]


def _controller(name: str, spec):
    name = name.lower()
    if name == "mpc":
        return LinearMPC(spec)
    if name == "pd":
        return PDController(spec)
    if name == "lqr":
        return LQRController(spec)
    raise SystemExit(f"unknown controller {name}")


def cmd_run(args: argparse.Namespace) -> int:
    spec = load_vehicle(args.vehicle)
    mission = point_to_point(spec.table_size)
    if args.duration:
        mission.duration = args.duration
    ctrl = _controller(args.controller, spec)
    if args.armed and not args.port and spec.mocap.enabled is False:
        print("refusing: --armed requires --port and working telemetry (set mocap.enabled)", file=sys.stderr)
        return 2
    rt = Runtime(
        spec,
        ctrl,
        mission,
        armed=args.armed,
        port=args.port,
        round_binary=args.round_binary,
        runs_root=Path(args.runs),
    )
    result = rt.run()
    plot_trajectory(spec, result, result.run_dir / "trajectory.png")
    gif = result.run_dir / "animation.gif"
    try:
        animate(spec, result, gif)
    except Exception as exc:  # animation is best-effort
        print(f"animation skipped: {exc}")
    if args.assets:
        assets = REPO / "docs" / "assets"
        assets.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(result.run_dir / "trajectory.png", assets / "solenoid_demo.png")
        if gif.exists():
            shutil.copyfile(gif, assets / "solenoid_demo.gif")
    print(f"run dir: {result.run_dir}")
    print(f"success={result.success}  final_error={result.final_error:.3f} m  "
          f"solver={result.mean_solver_ms:.1f} ms  deadline_misses={result.deadline_misses}")
    return 0 if result.success else 1


def cmd_compare(args: argparse.Namespace) -> int:
    pairs = []
    vehicles = [
        REPO / "vehicles" / "uk_solenoid_octagon.json",
        REPO / "vehicles" / "fan_quadrotor_plus.json",
    ]
    out_root = Path(args.runs) / "compare"
    out_root.mkdir(parents=True, exist_ok=True)
    for v in vehicles:
        spec = load_vehicle(v)
        mission = point_to_point(spec.table_size)
        rt = Runtime(spec, LinearMPC(spec), mission, runs_root=out_root)
        res = rt.run()
        plot_trajectory(spec, res, res.run_dir / "trajectory.png")
        pairs.append((spec.name, res))
        print(f"{spec.name}: success={res.success} err={res.final_error:.3f} m")
    dest = out_root / "compare_actuators.png"
    plot_compare(pairs, dest)
    if args.assets:
        assets = REPO / "docs" / "assets"
        assets.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(dest, assets / "compare_actuators.png")
        for name, res in pairs:
            src = res.run_dir / "trajectory.png"
            if src.exists():
                shutil.copyfile(src, assets / f"{name}.png")
    print(f"wrote {dest}")
    return 0


def _plus_layout(n: int, radius: float, F: float, typ: str, bidir: bool) -> list[dict]:
    """n in {3,4,6,8} geometric layouts students can start from."""
    import math

    items = []
    if n == 4:
        # Plus frame, force tangent to each arm so Mz is not a structural zero
        # (a fan on the x-arm blowing along +x through the COM produces no torque).
        pts = [
            ([radius, 0.0], [0.0, 1.0]),
            ([-radius, 0.0], [0.0, 1.0]),
            ([0.0, radius], [1.0, 0.0]),
            ([0.0, -radius], [1.0, 0.0]),
        ]
        for i, (pos, d) in enumerate(pts, 1):
            items.append(_thruster(f"T{i}", pos, d, F, typ, bidir))
        return items
    if n == 8:
        L, dlt = radius, radius * 0.28
        geom = [
            ([L, dlt], [-1, 0]), ([L, -dlt], [-1, 0]),
            ([-L, dlt], [1, 0]), ([-L, -dlt], [1, 0]),
            ([dlt, L], [0, -1]), ([-dlt, L], [0, -1]),
            ([dlt, -L], [0, 1]), ([-dlt, -L], [0, 1]),
        ]
        for i, (p, d) in enumerate(geom, 1):
            items.append(_thruster(f"T{i}", p, d, F, typ, bidir))
        return items
    for i in range(n):
        a = i * 2 * math.pi / n
        pos = [radius * math.cos(a), radius * math.sin(a)]
        if bidir or typ == "pwm_fan":
            direc = [-math.sin(a), math.cos(a)]  # tangential
        else:
            direc = [math.cos(a), math.sin(a)]  # radial out (force on body)
        items.append(_thruster(f"T{i+1}", pos, direc, F, typ, bidir))
    return items


def _thruster(tid, pos, direc, F, typ, bidir) -> dict:
    t = {
        "id": tid,
        "position": [float(pos[0]), float(pos[1])],
        "force_direction": [float(direc[0]), float(direc[1])],
        "F_max": F,
        "type": typ,
    }
    if typ == "binary_solenoid":
        t["min_pulse_ms"] = 30
        t["deadman_ms"] = 100
    elif typ == "pwm_fan":
        t["tau"] = 0.18
        t["bidirectional"] = bool(bidir)
        t["duty_min"] = 0.0
        t["duty_max"] = 1.0
        t["deadman_ms"] = 100
    else:
        t["bidirectional"] = True
    return t


def cmd_new_vehicle(args: argparse.Namespace) -> int:
    if sys.stdin.isatty() and not args.noninteractive:
        name = input("vehicle name [my_fan_sat]: ").strip() or "my_fan_sat"
        n = int(input("thruster count [4]: ").strip() or "4")
        typ = input("type binary_solenoid|pwm_fan|continuous [pwm_fan]: ").strip() or "pwm_fan"
        mass = float(input("mass kg [5]: ").strip() or "5")
        table = float(input("table size m [2]: ").strip() or "2")
        F = float(input("F_max N (calibrate later) [0.3]: ").strip() or "0.3")
        bidir = (input("bidirectional? [Y/n]: ").strip() or "y").lower().startswith("y")
        out = Path(input(f"write to [vehicles/{name}.json]: ").strip() or f"vehicles/{name}.json")
    else:
        name = args.name or "my_fan_sat"
        n = args.n or 4
        typ = args.type or "pwm_fan"
        mass = args.mass or 5.0
        table = args.table or 2.0
        F = args.fmax or 0.3
        bidir = args.bidirectional if args.bidirectional is not None else True
        out = Path(args.out or REPO / "vehicles" / f"{name}.json")

    radius = args.radius or 0.18
    Iz = mass * (radius ** 2) * 0.6
    data = {
        "name": name,
        "mass": mass,
        "Iz": Iz,
        "com": [0.0, 0.0],
        "table_size": table,
        "hull_radius": radius * 1.15,
        "linear_damping": 0.05,
        "rotational_damping": 0.01,
        "control_dt": 0.05,
        "sim_dt": 0.01,
        "notes": "Generated by python -m airbearing new-vehicle. Calibrate F_max (docs/LAB.md).",
        "thrusters": _plus_layout(n, radius, F, typ, bidir),
    }
    spec = spec_from_dict(data)
    report = controllability_report(spec)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, indent=2) + "\n")
    print(f"wrote {out}")
    print(f"B rank={report['rank_B']}  full_both_signs={report['full']}  n={report['n_thrusters']}")
    for k, v in report["details"].items():
        print(" ", v)
    if report["warning"]:
        print("WARNING:", report["warning"])
        return 2
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="airbearing",
        description="Planar air-bearing satellite GNC kit (not flight software).",
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="simulate (default) or drive hardware with --armed")
    r.add_argument("--vehicle", default=str(REPO / "vehicles" / "uk_solenoid_octagon.json"))
    r.add_argument("--controller", default="mpc", choices=["mpc", "pd", "lqr"])
    r.add_argument("--runs", default="runs")
    r.add_argument("--duration", type=float, default=None)
    r.add_argument("--armed", action="store_true", help="enable real gateway; refuses null telemetry")
    r.add_argument("--port", default=None, help="serial port, e.g. /dev/ttyUSB0")
    r.add_argument("--round-binary", action="store_true")
    r.add_argument("--assets", action="store_true", help="copy plots into docs/assets")
    r.set_defaults(func=cmd_run)

    c = sub.add_parser("compare-actuators", help="same point-to-point: solenoids vs fans")
    c.add_argument("--runs", default="runs")
    c.add_argument("--assets", action="store_true")
    c.set_defaults(func=cmd_compare)

    n = sub.add_parser("new-vehicle", help="wizard: write vehicles/*.json + controllability check")
    n.add_argument("--noninteractive", action="store_true")
    n.add_argument("--name", default=None)
    n.add_argument("--n", type=int, default=None)
    n.add_argument("--type", dest="type", default=None, choices=["binary_solenoid", "pwm_fan", "continuous"])
    n.add_argument("--mass", type=float, default=None)
    n.add_argument("--table", type=float, default=None)
    n.add_argument("--fmax", type=float, default=None)
    n.add_argument("--radius", type=float, default=None)
    n.add_argument("--out", default=None)
    n.add_argument("--bidirectional", action=argparse.BooleanOptionalAction, default=None)
    n.set_defaults(func=cmd_new_vehicle)

    v = sub.add_parser("check", help="validate a vehicle JSON and print controllability")
    v.add_argument("vehicle")

    def cmd_check(args):
        spec = load_vehicle(args.vehicle)
        report = controllability_report(spec)
        print(json.dumps({k: report[k] for k in ("full", "rank_B", "reachable", "warning", "n_thrusters")}, indent=2))
        return 0 if report["full"] else 2

    v.set_defaults(func=cmd_check)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args) or 0)
