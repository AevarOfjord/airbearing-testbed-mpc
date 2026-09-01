"""Versioned run logs. SI units. Refuse mismatched columns."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

SCHEMA_VERSION = 1
UNITS = "SI"
MAGIC = "airbearing_log"

POSE_COLUMNS = ("t", "x", "y", "yaw", "vx", "vy", "omega")
REF_COLUMNS = ("ref_x", "ref_y", "ref_yaw")
WRENCH_COLUMNS = ("Fx", "Fy", "Mz", "Fx_ach", "Fy_ach", "Mz_ach")
TIMING_COLUMNS = ("mpc_ms", "alloc_ms", "deadline_miss")
TEXT_COLUMNS = ("safety", "status")

# Pose is the minimum any log (mocap replay or full run) must provide.
REQUIRED_COLUMNS = POSE_COLUMNS


class LogSchemaError(ValueError):
    """CSV/JSON log does not match the airbearing schema."""


def comment_line(version: int = SCHEMA_VERSION) -> str:
    return f"# {MAGIC} schema_version={version} units={UNITS}\n"


def parse_comment(line: str) -> dict[str, str]:
    line = line.strip()
    if not line.startswith("#"):
        return {}
    body = line.lstrip("#").strip()
    if not body.startswith(MAGIC):
        return {}
    out: dict[str, str] = {"magic": MAGIC}
    for tok in body.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def u_columns(n: int) -> list[str]:
    return [f"u{i}" for i in range(n)]


def run_fieldnames(n_thrusters: int) -> list[str]:
    return (
        list(POSE_COLUMNS)
        + list(REF_COLUMNS)
        + list(WRENCH_COLUMNS)
        + list(TIMING_COLUMNS)
        + list(TEXT_COLUMNS)
        + u_columns(n_thrusters)
    )


def _open_rows(path: Path) -> tuple[dict[str, str], list[dict[str, str]]]:
    text = path.read_text()
    meta: dict[str, str] = {}
    lines = text.splitlines()
    data_lines: list[str] = []
    for line in lines:
        if line.startswith("#"):
            parsed = parse_comment(line)
            if parsed:
                meta.update(parsed)
            continue
        data_lines.append(line)
    if not data_lines:
        raise LogSchemaError(f"empty log {path}")
    reader = csv.DictReader(data_lines)
    if reader.fieldnames is None:
        raise LogSchemaError(f"no header in {path}")
    rows = list(reader)
    if not rows:
        raise LogSchemaError(f"empty log {path}")
    return meta, rows


def assert_columns(fieldnames: Sequence[str], required: Sequence[str] = REQUIRED_COLUMNS) -> None:
    have = [f.strip() for f in fieldnames if f]
    missing = [c for c in required if c not in have]
    if missing:
        raise LogSchemaError(
            f"log schema mismatch: missing columns {missing} (have {have}). "
            f"Expected SI units, schema_version={SCHEMA_VERSION}."
        )


def load_log(path: str | Path, *, required: Sequence[str] | None = None) -> dict[str, np.ndarray]:
    """Load a pose or full run CSV. Raises LogSchemaError on missing required columns."""
    path = Path(path)
    if path.suffix.lower() == ".json":
        return _load_json_log(path)
    meta, rows = _open_rows(path)
    version = int(meta.get("schema_version", SCHEMA_VERSION))
    if version != SCHEMA_VERSION:
        raise LogSchemaError(
            f"unsupported log schema_version={version} (this package reads {SCHEMA_VERSION})"
        )
    if meta.get("units") not in (None, "", UNITS):
        raise LogSchemaError(f"log units must be {UNITS}, got {meta.get('units')}")
    keys = list(rows[0].keys())
    req = list(required) if required is not None else list(REQUIRED_COLUMNS)
    assert_columns(keys, req)
    out: dict[str, Any] = {
        "schema_version": version,
        "units": UNITS,
        "path": str(path),
    }
    numeric = [k for k in keys if k not in TEXT_COLUMNS]
    for k in numeric:
        vals = []
        for r in rows:
            raw = r.get(k, "")
            if raw is None or raw == "":
                vals.append(0.0)
            else:
                vals.append(float(raw))
        out[k] = np.array(vals, dtype=float)
    for k in TEXT_COLUMNS:
        if k in keys:
            out[k] = np.array([r.get(k, "") for r in rows], dtype=object)
    u_cols = sorted([k for k in keys if k.startswith("u") and k[1:].isdigit()], key=lambda s: int(s[1:]))
    if u_cols:
        out["u"] = np.column_stack([out[k] for k in u_cols])
    return out


def _load_json_log(path: Path) -> dict[str, np.ndarray]:
    data = json.loads(path.read_text())
    ver = int(data.get("schema_version", SCHEMA_VERSION))
    if ver != SCHEMA_VERSION:
        raise LogSchemaError(f"unsupported log schema_version={ver}")
    samples = data.get("samples") or data.get("rows")
    if not samples:
        raise LogSchemaError(f"json log missing samples: {path}")
    keys = list(samples[0].keys())
    assert_columns(keys)
    out: dict[str, Any] = {"schema_version": ver, "units": UNITS, "path": str(path)}
    for k in keys:
        if k in TEXT_COLUMNS:
            out[k] = np.array([s.get(k, "") for s in samples], dtype=object)
        else:
            out[k] = np.array([float(s.get(k, 0.0) or 0.0) for s in samples], dtype=float)
    u_cols = sorted([k for k in keys if k.startswith("u") and k[1:].isdigit()], key=lambda s: int(s[1:]))
    if u_cols:
        out["u"] = np.column_stack([out[k] for k in u_cols])
    return out


def write_run_csv(path: Path, fieldnames: Sequence[str], rows: Iterable[dict[str, Any]]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        f.write(comment_line())
        w = csv.DictWriter(f, fieldnames=list(fieldnames), extrasaction="ignore")
        w.writeheader()
        for row in rows:
            w.writerow(row)


def write_pose_csv(path: Path, t: np.ndarray, states: np.ndarray, u: np.ndarray | None = None) -> Path:
    """Write a minimal SI pose log (optionally with commands). Used by ID and mocap dumps."""
    path = Path(path)
    n = 0 if u is None else int(u.shape[1])
    fields = list(POSE_COLUMNS) + (u_columns(n) if n else [])
    rows = []
    for i in range(len(t)):
        d: dict[str, Any] = {
            "t": float(t[i]),
            "x": float(states[i, 0]),
            "y": float(states[i, 1]),
            "yaw": float(states[i, 2]),
            "vx": float(states[i, 3]) if states.shape[1] > 3 else 0.0,
            "vy": float(states[i, 4]) if states.shape[1] > 4 else 0.0,
            "omega": float(states[i, 5]) if states.shape[1] > 5 else 0.0,
        }
        if u is not None:
            for j in range(n):
                d[f"u{j}"] = float(u[i, j])
        rows.append(d)
    write_run_csv(path, fields, rows)
    return path
