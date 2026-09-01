"""Fail-safe supervisor: never adds thrust, only removes it. Deadman + workspace."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from airbearing.spec import SatelliteSpec


@dataclass
class SafetyLimits:
    max_speed: float = 0.6
    max_yaw_rate: float = 1.5
    telemetry_strikes: int = 8
    command_age_limit_s: float = 0.15


@dataclass
class SafetyDecision:
    cmd: np.ndarray
    abort: bool = False
    overridden: bool = False
    reasons: list[str] = field(default_factory=list)


class SafetySupervisor:
    def __init__(self, spec: SatelliteSpec, limits: SafetyLimits | None = None):
        self.spec = spec
        self.limits = limits or SafetyLimits()
        # deadman: shortest gateway timeout among actuators
        deadmans = [t.deadman_ms / 1000.0 for t in spec.thrusters if t.deadman_ms]
        self.deadman_s = min(deadmans) if deadmans else 0.1
        self._telemetry_losses = 0
        self._last_cmd_age = 0.0

    def review(
        self,
        cmd: np.ndarray,
        state: np.ndarray | None,
        telemetry_ok: bool,
        dt: float,
    ) -> SafetyDecision:
        reasons: list[str] = []
        abort = False
        overridden = False
        out = np.asarray(cmd, dtype=float).copy()
        n = self.spec.n_thrusters
        if out.shape[0] != n:
            out = np.zeros(n)
            reasons.append("malformed command")
            overridden = True

        if not telemetry_ok or state is None:
            self._telemetry_losses += 1
            out[:] = 0.0
            overridden = True
            reasons.append("null telemetry — refuse to fire")
            if self._telemetry_losses >= self.limits.telemetry_strikes:
                abort = True
                reasons.append("telemetry deadman abort")
            return SafetyDecision(out, abort=abort, overridden=True, reasons=reasons)
        self._telemetry_losses = 0

        half = 0.5 * self.spec.table_size
        x, y, _, vx, vy, om = state
        if abs(x) > half or abs(y) > half:
            out[:] = 0.0
            abort = True
            overridden = True
            reasons.append("outside table — emergency stop")

        speed = float(np.hypot(vx, vy))
        if speed > self.limits.max_speed or abs(om) > self.limits.max_yaw_rate:
            out[:] = 0.0
            overridden = True
            reasons.append("speed limit — coast")

        self._last_cmd_age = 0.0
        return SafetyDecision(out, abort=abort, overridden=overridden, reasons=reasons)

    def expire(self, dt: float) -> np.ndarray:
        """If the host stops sending, gateways and this supervisor zero actuators."""
        self._last_cmd_age += dt
        if self._last_cmd_age >= self.deadman_s:
            return np.zeros(self.spec.n_thrusters)
        return None  # type: ignore[return-value]
