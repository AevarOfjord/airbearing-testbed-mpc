"""Citable run reports: summary.json + methods-style table."""

from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path
from typing import Any

import numpy as np

from airbearing.logschema import SCHEMA_VERSION, UNITS
from airbearing.spec import SatelliteSpec, repo_root


def file_sha256(path: str | Path | None) -> str | None:
    if path is None:
        return None
    p = Path(path)
    if not p.is_file():
        return None
    h = hashlib.sha256()
    h.update(p.read_bytes())
    return h.hexdigest()


def git_hash(cwd: Path | None = None) -> str | None:
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=str(cwd or repo_root()),
            stderr=subprocess.DEVNULL,
            timeout=2,
        )
        return out.decode().strip()
    except Exception:
        return None


def settling_time_s(logs, pos_tol: float, yaw_tol: float, goal: np.ndarray) -> float | None:
    if not logs:
        return None
    n = len(logs)
    for i, row in enumerate(logs):
        ok = True
        for later in logs[i:]:
            e = float(np.hypot(later.state[0] - goal[0], later.state[1] - goal[1]))
            ye = abs(((later.state[2] - goal[2] + np.pi) % (2 * np.pi)) - np.pi)
            if e > pos_tol or ye > yaw_tol:
                ok = False
                break
        if ok:
            return float(row.t)
    return float(logs[-1].t) if n else None


def integrated_abs_u(logs, dt: float) -> float:
    if not logs:
        return 0.0
    s = 0.0
    for row in logs:
        s += float(np.sum(np.abs(row.cmd))) * dt
    return s


def solver_percentiles(logs) -> tuple[float, float, float]:
    if not logs:
        return 0.0, 0.0, 0.0
    v = np.array([r.mpc_ms + r.alloc_ms for r in logs], dtype=float)
    return float(np.mean(v)), float(np.percentile(v, 50)), float(np.percentile(v, 95))


def build_summary(
    *,
    spec: SatelliteSpec,
    result,
    mission,
    vehicle_path: str | Path | None = None,
    seed: int | None = None,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    logs = result.logs
    mean_ms, p50, p95 = solver_percentiles(logs)
    dt = float(spec.control_dt)
    goal = mission.goal
    settle = settling_time_s(logs, mission.pos_tol, mission.yaw_tol, goal)
    src = vehicle_path or (spec.source_path if spec.source_path else None)
    summary: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "units": UNITS,
        "package": "airbearing",
        "not_flight_software": True,
        "vehicle": spec.name,
        "vehicle_file": str(src) if src else None,
        "vehicle_sha256": file_sha256(src),
        "controller": getattr(result, "controller_name", None)
        or getattr(getattr(result, "controller", None), "name", None),
        "mission": mission.name,
        "success": bool(result.success),
        "final_error_m": float(result.final_error),
        "settling_s": settle,
        "pos_tol_m": float(mission.pos_tol),
        "yaw_tol_rad": float(mission.yaw_tol),
        "integrated_abs_u": integrated_abs_u(logs, dt),
        "solver_mean_ms": mean_ms,
        "solver_p50_ms": p50,
        "solver_p95_ms": p95,
        "deadline_misses": int(result.deadline_misses),
        "steps": len(logs),
        "control_dt_s": dt,
        "seed": seed,
        "git_hash": git_hash(),
        "aborted": bool(result.aborted),
        "abort_reason": result.abort_reason,
        "wall_s": getattr(result, "wall_s", None),
    }
    if extra:
        summary.update(extra)
    return summary


def methods_table(summary: dict[str, Any]) -> str:
    rows = [
        ("vehicle", summary.get("vehicle")),
        ("vehicle_sha256", (summary.get("vehicle_sha256") or "")[:12] or "—"),
        ("git_hash", (summary.get("git_hash") or "—")[:12]),
        ("seed", summary.get("seed") if summary.get("seed") is not None else "—"),
        ("settling_s", _fmt(summary.get("settling_s"), "s")),
        ("final_error_m", _fmt(summary.get("final_error_m"), "m")),
        ("integrated_|u|", _fmt(summary.get("integrated_abs_u"), "")),
        ("solver_p50_ms", _fmt(summary.get("solver_p50_ms"), "ms")),
        ("solver_p95_ms", _fmt(summary.get("solver_p95_ms"), "ms")),
        ("deadline_misses", summary.get("deadline_misses")),
        ("success", summary.get("success")),
        ("units", summary.get("units", UNITS)),
    ]
    lines = [
        "Metric                  Value",
        "----------------------  --------------------",
    ]
    for k, v in rows:
        lines.append(f"{k:22s}  {v}")
    return "\n".join(lines) + "\n"


def _fmt(v: Any, unit: str) -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        s = f"{v:.4g}"
    else:
        s = str(v)
    return f"{s} {unit}".strip()


def write_summary(run_dir: Path, summary: dict[str, Any]) -> Path:
    run_dir = Path(run_dir)
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    (run_dir / "methods.txt").write_text(methods_table(summary))
    return run_dir / "summary.json"


def load_run_summary(run_dir: str | Path) -> dict[str, Any]:
    run_dir = Path(run_dir)
    p = run_dir / "summary.json" if run_dir.is_dir() else run_dir
    if not p.is_file():
        raise FileNotFoundError(f"no summary.json at {p}")
    return json.loads(p.read_text())


def print_report(run_dir: str | Path) -> str:
    run_dir = Path(run_dir)
    summary = load_run_summary(run_dir)
    table = methods_table(summary)
    methods = run_dir / "methods.txt" if run_dir.is_dir() else None
    if methods and not methods.is_file() and run_dir.is_dir():
        methods.write_text(table)
    return table
