"""Pose source: simulated plant or HTTP mocap. Real mode refuses null telemetry."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from airbearing.dynamics import Plant


class PoseSource(Protocol):
    def read(self) -> tuple[np.ndarray | None, bool]:
        """Return (state6 or None, ok)."""
        ...


@dataclass
class SimulatedMocap:
    """Drop-in mock: the plant IS the 'camera'. Optional dropout for testing deadman."""

    plant: Plant
    dropout: bool = False

    def read(self) -> tuple[np.ndarray | None, bool]:
        if self.dropout:
            return None, False
        # modest quantization like a cheap mocap
        z = self.plant.state.copy()
        z[0] = round(z[0], 4)
        z[1] = round(z[1], 4)
        z[2] = round(z[2], 5)
        return z, True


class HttpMocap:
    def __init__(self, endpoint: str, timeout_s: float = 0.05):
        self.endpoint = endpoint
        self.timeout_s = timeout_s

    def read(self) -> tuple[np.ndarray | None, bool]:
        import json
        import urllib.request

        try:
            with urllib.request.urlopen(self.endpoint, timeout=self.timeout_s) as r:
                data = json.loads(r.read().decode())
            st = np.array(
                [
                    data["x"],
                    data["y"],
                    data["yaw"],
                    data.get("vx", 0.0),
                    data.get("vy", 0.0),
                    data.get("omega", 0.0),
                ],
                dtype=float,
            )
            return st, True
        except Exception:
            return None, False



@dataclass
class CsvReplay:
    """Recorded mocap: sequential 6-state rows from labs/data/example_mocap.csv."""

    path: str
    _rows: list = field(default_factory=list, init=False)
    _i: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        import csv
        from pathlib import Path

        rows = []
        with Path(self.path).open(newline="") as f:
            lines = [ln for ln in f if not ln.startswith("#")]
            r = csv.DictReader(lines)
            for d in r:
                st = np.array(
                    [
                        float(d["x"]),
                        float(d["y"]),
                        float(d["yaw"]),
                        float(d.get("vx") or 0.0),
                        float(d.get("vy") or 0.0),
                        float(d.get("omega") or 0.0),
                    ],
                    dtype=float,
                )
                rows.append(st)
        if not rows:
            raise ValueError(f"empty mocap csv: {self.path}")
        self._rows = rows
        self._i = 0

    def read(self) -> tuple[np.ndarray | None, bool]:
        if self._i >= len(self._rows):
            return self._rows[-1].copy(), True
        z = self._rows[self._i].copy()
        self._i += 1
        return z, True

    @property
    def n(self) -> int:
        return len(self._rows)
