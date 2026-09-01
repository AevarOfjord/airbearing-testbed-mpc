"""Load and validate SatelliteSpec JSON. Students edit vehicles/*.json, not this file."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

import jsonschema
import numpy as np

def repo_root() -> Path:
    here = Path(__file__).resolve()
    for cand in (here.parents[2], Path.cwd()):
        if (cand / "schemas").is_dir() and (cand / "pyproject.toml").is_file():
            return cand
        if (cand / "schemas").is_dir() and ((cand / "examples" / "vehicles").is_dir() or (cand / "vehicles").is_dir()):
            return cand
    return here.parents[2]


def _schema_file() -> Path:
    here = Path(__file__).resolve()
    root = repo_root()
    candidates = [
        root / "schemas" / "vehicle.schema.json",
        root / "schemas" / "satellite_spec.schema.json",
        here.parents[2] / "schemas" / "vehicle.schema.json",
        here.parents[2] / "schemas" / "satellite_spec.schema.json",
        Path.cwd() / "schemas" / "vehicle.schema.json",
        Path.cwd() / "schemas" / "satellite_spec.schema.json",
    ]
    for c in candidates:
        if c.is_file():
            return c
    raise FileNotFoundError("vehicle.schema.json not found; run from the repo root")


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
class SensorSource:
    type: str
    meas: list[str] = field(default_factory=list)
    endpoint: str = ""
    port: str = ""
    baud: int = 115200
    path: str = ""
    timeout_s: float = 0.0
    camera: int = 0
    marker_id: int = 0
    marker_size_m: float = 0.05
    table_size: float = 0.0
    noise: dict[str, float] = field(default_factory=dict)


@dataclass
class Navigation:
    estimator: str = "passthrough"
    external: SensorSource | None = None
    onboard: SensorSource | None = None
    Q: dict[str, float] = field(default_factory=dict)
    R: dict[str, float] = field(default_factory=dict)


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
    navigation: Navigation | None = None
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


def _sensor_source_from_dict(raw: dict[str, Any]) -> SensorSource:
    return SensorSource(
        type=raw["type"],
        meas=list(raw.get("meas") or []),
        endpoint=raw.get("endpoint", "") or "",
        port=raw.get("port", "") or "",
        baud=int(raw.get("baud", 115200) or 115200),
        path=raw.get("path", "") or "",
        timeout_s=float(raw.get("timeout_s", 0.0) or 0.0),
        camera=int(raw.get("camera", 0) or 0),
        marker_id=int(raw.get("marker_id", 0) or 0),
        marker_size_m=float(raw.get("marker_size_m", 0.05) or 0.05),
        table_size=float(raw.get("table_size", 0.0) or 0.0),
        noise={k: float(v) for k, v in (raw.get("noise") or {}).items()},
    )


def navigation_from_dict(raw: dict[str, Any]) -> Navigation:
    ext = _sensor_source_from_dict(raw["external"]) if raw.get("external") else None
    onboard = _sensor_source_from_dict(raw["onboard"]) if raw.get("onboard") else None
    return Navigation(
        estimator=str(raw.get("estimator", "passthrough") or "passthrough"),
        external=ext,
        onboard=onboard,
        Q={k: float(v) for k, v in (raw.get("Q") or {}).items()},
        R={k: float(v) for k, v in (raw.get("R") or {}).items()},
    )


def _sensor_source_to_dict(src: SensorSource) -> dict[str, Any]:
    d: dict[str, Any] = {"type": src.type}
    if src.meas:
        d["meas"] = list(src.meas)
    if src.endpoint:
        d["endpoint"] = src.endpoint
    if src.port:
        d["port"] = src.port
    if src.baud and src.baud != 115200:
        d["baud"] = int(src.baud)
    if src.path:
        d["path"] = src.path
    if src.timeout_s:
        d["timeout_s"] = float(src.timeout_s)
    if src.type == "webcam_aruco":
        d["camera"] = int(src.camera)
        d["marker_id"] = int(src.marker_id)
        d["marker_size_m"] = float(src.marker_size_m)
    if src.table_size:
        d["table_size"] = float(src.table_size)
    if src.noise:
        d["noise"] = {k: float(v) for k, v in src.noise.items()}
    return d


def navigation_to_dict(nav: Navigation) -> dict[str, Any]:
    d: dict[str, Any] = {"estimator": nav.estimator}
    if nav.external is not None:
        d["external"] = _sensor_source_to_dict(nav.external)
    if nav.onboard is not None:
        d["onboard"] = _sensor_source_to_dict(nav.onboard)
    if nav.Q:
        d["Q"] = {k: float(v) for k, v in nav.Q.items()}
    if nav.R:
        d["R"] = {k: float(v) for k, v in nav.R.items()}
    return d


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
    navigation = None
    if data.get("navigation"):
        navigation = navigation_from_dict(data["navigation"])
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
        navigation=navigation,
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
    limits: dict[str, float] = {}
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
            limits[f"{tag}{lab}"] = val
    radial = plus_frame_radial_warning(spec)
    full = all(reachable.values())
    rank = int(np.linalg.matrix_rank(B, tol=1e-8))
    warning = None
    if radial:
        warning = radial
    elif not full:
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
        "limits": limits,
        "n_thrusters": n,
        "warning": warning,
        "radial_plus": bool(radial),
    }


def plus_frame_radial_warning(spec: SatelliteSpec, ang_tol: float = 0.18) -> str | None:
    """Plus-frame fans that blow through the COM produce Mz ≡ 0."""
    if spec.n_thrusters != 4:
        return None
    axes = 0
    radial = 0
    mz = spec.allocation_matrix()[2]
    for t in spec.thrusters:
        r = t.position - spec.com
        rn = float(np.linalg.norm(r))
        if rn < 1e-6:
            continue
        ru = r / rn
        # on a body axis?
        if min(abs(ru[0]), abs(ru[1])) < 0.25:
            axes += 1
        align = abs(float(np.dot(ru, t.force_direction)))
        if align > np.cos(ang_tol):
            radial += 1
    if axes >= 3 and radial >= 3 and float(np.max(np.abs(mz))) < 1e-6 * max(t.F_max for t in spec.thrusters):
        return (
            "Plus-frame fans are radial (through COM): Mz ≡ 0. "
            "Rotate each jet 90° (tangent to the arm) or offset the nozzle."
        )
    if float(np.max(np.abs(mz))) < 1e-8:
        return "Allocation Mz column is structurally zero (jets through COM)."
    return None


def spec_to_dict(spec: SatelliteSpec) -> dict[str, Any]:
    """Serialize a spec back to the student JSON shape (no Python constants)."""
    def tdict(t: Thruster) -> dict[str, Any]:
        d: dict[str, Any] = {
            "id": t.id,
            "position": [float(t.position[0]), float(t.position[1])],
            "force_direction": [float(t.force_direction[0]), float(t.force_direction[1])],
            "F_max": float(t.F_max),
            "type": t.type,
        }
        if t.type == "binary_solenoid":
            d["min_pulse_ms"] = float(t.min_pulse_ms)
            d["deadman_ms"] = float(t.deadman_ms)
        elif t.type == "pwm_fan":
            d["tau"] = float(t.tau)
            d["bidirectional"] = bool(t.bidirectional)
            d["duty_min"] = float(t.duty_min)
            d["duty_max"] = float(t.duty_max)
            d["deadman_ms"] = float(t.deadman_ms)
        else:
            d["bidirectional"] = True
        return d

    data: dict[str, Any] = {
        "name": spec.name,
        "mass": float(spec.mass),
        "Iz": float(spec.Iz),
        "com": [float(spec.com[0]), float(spec.com[1])],
        "table_size": float(spec.table_size),
        "hull_radius": float(spec.hull_radius),
        "linear_damping": float(spec.linear_damping),
        "rotational_damping": float(spec.rotational_damping),
        "control_dt": float(spec.control_dt),
        "sim_dt": float(spec.sim_dt),
        "notes": spec.notes,
        "thrusters": [tdict(t) for t in spec.thrusters],
    }
    if spec.reaction_wheel is not None:
        rw = spec.reaction_wheel
        data["reaction_wheel"] = {
            "inertia": rw.inertia,
            "max_torque": rw.max_torque,
            "max_momentum": rw.max_momentum,
            "initial_momentum": rw.initial_momentum,
            "sim_only": rw.sim_only,
        }
    if spec.mocap and spec.mocap.enabled:
        data["mocap"] = {
            "enabled": spec.mocap.enabled,
            "rigid_body_id": spec.mocap.rigid_body_id,
            "endpoint": spec.mocap.endpoint,
            "timeout_s": spec.mocap.timeout_s,
        }
    if spec.navigation is not None:
        data["navigation"] = navigation_to_dict(spec.navigation)
    return data


def examples_dir() -> Path:
    return repo_root() / "examples" / "vehicles"


def vehicles_dir() -> Path:
    """Student-owned JSON (created on demand). Shipped examples live in examples/vehicles/."""
    d = repo_root() / "vehicles"
    d.mkdir(parents=True, exist_ok=True)
    return d


def shipped_vehicle(name: str) -> Path:
    """Resolve a packaged example: fan_plus, solenoid_octagon, fan_hex, micro_3thruster."""
    stem = name.replace(".json", "")
    p = examples_dir() / f"{stem}.json"
    if p.is_file():
        return p
    fallback = vehicles_dir() / f"{stem}.json"
    if fallback.is_file():
        return fallback
    raise FileNotFoundError(f"vehicle {name} not found under examples/vehicles or vehicles/")


def default_vehicle() -> Path:
    return shipped_vehicle("fan_plus")


def assert_vehicle_save_path(path: Path, *, allow_any: bool = False) -> Path:
    path = Path(path)
    if allow_any:
        return path
    root = vehicles_dir().resolve()
    try:
        path.resolve().relative_to(root)
    except ValueError as exc:
        raise ValueError(f"vehicle JSON must be saved under {root}") from exc
    if path.suffix != ".json":
        raise ValueError("vehicle file must end in .json")
    return path


def save_vehicle(spec: SatelliteSpec | dict[str, Any], path: str | Path, *, allow_any: bool = False) -> Path:
    path = assert_vehicle_save_path(Path(path), allow_any=allow_any)
    data = spec_to_dict(spec) if isinstance(spec, SatelliteSpec) else dict(spec)
    validate_dict(data)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")
    return path
