"""Load and validate SatelliteSpec JSON. Students edit vehicles/*.json, not this file."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

import jsonschema
import numpy as np

def _schema_file() -> Path:
    here = Path(__file__).resolve()
    candidates = [
        here.parents[2] / "schemas" / "satellite_spec.schema.json",
        Path.cwd() / "schemas" / "satellite_spec.schema.json",
    ]
    for c in candidates:
        if c.is_file():
            return c
    raise FileNotFoundError("satellite_spec.schema.json not found; run from the repo root")


ActuatorType = Literal["binary_solenoid", "pwm_fan", "continuous"]


@dataclass
class Thruster:
    id: str
    position: np.ndarray
    force_direction: np.ndarray
    F_max: float
    type: ActuatorType
    min_pulse_ms: float = 0.0
    deadman_ms: float = 100.0
    duty_min: float = 0.0
    duty_max: float = 1.0
    bidirectional: bool = False
    tau: float = 0.0

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float).reshape(2)
        d = np.asarray(self.force_direction, dtype=float).reshape(2)
        n = np.linalg.norm(d)
        if n < 1e-12:
            raise ValueError(f"thruster {self.id}: force_direction is zero")
        self.force_direction = d / n
        if self.type == "pwm_fan" and self.bidirectional is False:
            # unidirectional fan still 0..1
            pass
        if self.type == "continuous":
            self.bidirectional = True

    @property
    def cmd_min(self) -> float:
        if self.type == "binary_solenoid":
            return 0.0
        if self.bidirectional or self.type == "continuous":
            return -1.0
        return float(self.duty_min)

    @property
    def cmd_max(self) -> float:
        if self.type == "binary_solenoid":
            return 1.0
        return float(self.duty_max) if not self.bidirectional else 1.0


@dataclass
class ReactionWheel:
    inertia: float
    max_torque: float
    max_momentum: float
    initial_momentum: float = 0.0
    sim_only: bool = True


@dataclass
class Mocap:
    enabled: bool = False
    rigid_body_id: int = 1
    endpoint: str = "http://127.0.0.1:8080/pose"
    timeout_s: float = 0.05


@dataclass
class SatelliteSpec:
    name: str
    mass: float
    Iz: float
    com: np.ndarray
    table_size: float
    thrusters: list[Thruster]
    hull_radius: float = 0.2
    linear_damping: float = 0.0
    rotational_damping: float = 0.0
    control_dt: float = 0.1
    sim_dt: float = 0.02
    reaction_wheel: ReactionWheel | None = None
    mocap: Mocap = field(default_factory=Mocap)
    notes: str = ""
    source_path: Path | None = None

    def __post_init__(self) -> None:
        self.com = np.asarray(self.com, dtype=float).reshape(2)
        ids = [t.id for t in self.thrusters]
        if len(ids) != len(set(ids)):
            raise ValueError("thruster ids must be unique")

    @property
    def n_thrusters(self) -> int:
        return len(self.thrusters)

    def allocation_matrix(self) -> np.ndarray:
        """3 x n map from command (1 = F_max along force_direction) to body wrench [Fx, Fy, Mz]."""
        B = np.zeros((3, self.n_thrusters))
        for i, t in enumerate(self.thrusters):
            r = t.position - self.com
            f = t.F_max * t.force_direction
            mz = r[0] * f[1] - r[1] * f[0]
            B[:, i] = [f[0], f[1], mz]
        return B

    def cmd_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        lo = np.array([t.cmd_min for t in self.thrusters], dtype=float)
        hi = np.array([t.cmd_max for t in self.thrusters], dtype=float)
        return lo, hi


def _schema() -> dict[str, Any]:
    return json.loads(_schema_file().read_text())


def validate_dict(data: dict[str, Any]) -> None:
    jsonschema.validate(instance=data, schema=_schema())


def spec_from_dict(data: dict[str, Any], source: Path | None = None) -> SatelliteSpec:
    validate_dict(data)
    thrusters = []
    for raw in data["thrusters"]:
        thrusters.append(
            Thruster(
                id=raw["id"],
                position=raw["position"],
                force_direction=raw["force_direction"],
                F_max=raw["F_max"],
                type=raw["type"],
                min_pulse_ms=raw.get("min_pulse_ms", 0.0),
                deadman_ms=raw.get("deadman_ms", 100.0),
                duty_min=raw.get("duty_min", 0.0),
                duty_max=raw.get("duty_max", 1.0),
                bidirectional=raw.get("bidirectional", False),
                tau=raw.get("tau", 0.0),
            )
        )
    rw = None
    if data.get("reaction_wheel"):
        r = data["reaction_wheel"]
        rw = ReactionWheel(
            inertia=r["inertia"],
            max_torque=r["max_torque"],
            max_momentum=r["max_momentum"],
            initial_momentum=r.get("initial_momentum", 0.0),
            sim_only=r.get("sim_only", True),
        )
    mocap = Mocap()
    if data.get("mocap"):
        m = data["mocap"]
        mocap = Mocap(
            enabled=m.get("enabled", False),
            rigid_body_id=m.get("rigid_body_id", 1),
            endpoint=m.get("endpoint", mocap.endpoint),
            timeout_s=m.get("timeout_s", 0.05),
        )
    return SatelliteSpec(
        name=data["name"],
        mass=data["mass"],
        Iz=data["Iz"],
        com=data["com"],
        table_size=data["table_size"],
        thrusters=thrusters,
        hull_radius=data.get("hull_radius", 0.2),
        linear_damping=data.get("linear_damping", 0.0),
        rotational_damping=data.get("rotational_damping", 0.0),
        control_dt=data.get("control_dt", 0.1),
        sim_dt=data.get("sim_dt", 0.02),
        reaction_wheel=rw,
        mocap=mocap,
        notes=data.get("notes", ""),
        source_path=source,
    )


def load_vehicle(path: str | Path) -> SatelliteSpec:
    p = Path(path)
    data = json.loads(p.read_text())
    return spec_from_dict(data, source=p)


def controllability_report(spec: SatelliteSpec, eps: float = 1e-3) -> dict[str, Any]:
    """Check whether both signs of Fx, Fy, Mz are reachable with feasible cmds."""
    import cvxpy as cp

    B = spec.allocation_matrix()
    lo, hi = spec.cmd_bounds()
    n = spec.n_thrusters
    labels = ["Fx", "Fy", "Mz"]
    reachable: dict[str, bool] = {}
    details: dict[str, str] = {}
    for i, lab in enumerate(labels):
        for sign, tag in ((1.0, "+"), (-1.0, "-")):
            u = cp.Variable(n)
            wrench = B @ u
            # maximize signed component subject to bounds
            prob = cp.Problem(
                cp.Maximize(sign * wrench[i]),
                [u >= lo, u <= hi],
            )
            try:
                prob.solve(solver=cp.CLARABEL, verbose=False)
            except Exception:
                prob.solve(solver=cp.OSQP, verbose=False)
            val = float(prob.value) if prob.value is not None else 0.0
            ok = val > eps
            reachable[f"{tag}{lab}"] = ok
            details[f"{tag}{lab}"] = f"max {tag}{lab}={val:.4g} {'OK' if ok else 'FAIL'}"
    full = all(reachable.values())
    rank = int(np.linalg.matrix_rank(B, tol=1e-8))
    warning = None
    if not full:
        warning = (
            "Not fully controllable with the current actuator cone. "
            "Three unidirectional thrusters cannot positively span R^3; "
            "add thrusters or make some bidirectional."
        )
    return {
        "full": full,
        "rank_B": rank,
        "reachable": reachable,
        "details": details,
        "n_thrusters": n,
        "warning": warning,
    }
