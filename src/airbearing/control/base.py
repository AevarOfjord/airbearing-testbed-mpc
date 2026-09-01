from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class ControllerOutput:
    wrench: np.ndarray  # body Fx, Fy, Mz
    solver_ms: float = 0.0
    status: str = "ok"
    feasible: bool = True
    extra: dict = field(default_factory=dict)


class Controller:
    name = "base"

    def reset(self) -> None:
        return

    def compute(self, state: np.ndarray, ref: np.ndarray) -> ControllerOutput:
        raise NotImplementedError
