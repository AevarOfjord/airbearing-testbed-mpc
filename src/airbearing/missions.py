"""Simple planar missions. Default demo is a 1 m point-to-point."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Mission:
    name: str
    start: np.ndarray  # x, y, yaw
    goal: np.ndarray
    duration: float
    hold_s: float = 2.0
    pos_tol: float = 0.08
    yaw_tol: float = 0.15

    def ref_at(self, t: float) -> np.ndarray:
        return self.goal.copy()


def point_to_point(spec_table: float, scale: float | None = None) -> Mission:
    """Stay well inside the table. Scale the hop to the vehicle's table."""
    hop = min(0.65, 0.16 * spec_table)
    hop = float(max(hop, 0.30))
    if scale is not None:
        hop = scale
    return Mission(
        name="point_to_point",
        start=np.array([-hop, 0.0, 0.0]),
        goal=np.array([hop, 0.15, 0.0]),
        duration=16.0 if spec_table >= 4 else 10.0,
        pos_tol=0.12 if spec_table >= 4 else 0.08,
    )
