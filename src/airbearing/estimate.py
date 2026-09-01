"""State estimators: passthrough (copy pose) or planar EKF (IMU predict, pose update).

MPC always consumes a 6-state. Invalid estimates are flagged so --armed can refuse.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from airbearing.dynamics import Plant, wrap_angle
from airbearing.telemetry import (
    Measurement,
    PoseSourceDriver,
    has_imu,
    has_pose,
    make_driver,
    state_from_values,
)

STATE_KEYS = ("x", "y", "yaw", "vx", "vy", "omega")

# Discrete process-noise variances (SI^2) — conservative teaching defaults.
DEFAULT_Q = {
    "x": 1e-5,
    "y": 1e-5,
    "yaw": 1e-6,
    "vx": 5e-4,
    "vy": 5e-4,
    "omega": 1e-4,
}
# Measurement-noise variances for external x,y,yaw (and unused IMU keys).
DEFAULT_R = {
    "x": 1e-3,
    "y": 1e-3,
    "yaw": 4e-4,
    "vx": 1e-2,
    "vy": 1e-2,
    "omega": 1e-2,
    "ax": 5e-2,
    "ay": 5e-2,
    "gyro_z": 1e-3,
}


def _diag(keys: tuple[str, ...], src: dict[str, float], defaults: dict[str, float]) -> np.ndarray:
    return np.diag([float(src.get(k, defaults[k])) for k in keys])


class PassthroughEstimator:
    """Copy the pose-bearing source (external preferred). No fusion."""

    name = "passthrough"

    def __init__(self) -> None:
        self.x = np.zeros(6)
        self._inited = False

    def reset(self, x0: np.ndarray) -> None:
        self.x = np.asarray(x0, dtype=float).reshape(6).copy()
        self._inited = True

    def step(
        self,
        dt: float,
        onboard: Measurement | None,
        external: Measurement | None,
    ) -> tuple[np.ndarray | None, bool]:
        src = None
        if has_pose(external):
            src = external
        elif has_pose(onboard):
            src = onboard
        if src is None:
            return self.x.copy() if self._inited else None, False
        self.x = state_from_values(src.values, prev=None)
        # IMU-only passthrough cannot invent a pose; gyro may fill omega if present.
        if "omega" not in src.values and has_imu(onboard) and "gyro_z" in onboard.values:
            self.x[5] = float(onboard.values["gyro_z"])
        self._inited = True
        return self.x.copy(), True


class PlanarEKF:
    """6-state EKF. IMU (ax, ay, gyro_z) predicts; external (x, y, yaw) updates."""

    name = "ekf"

    def __init__(
        self,
        Q: dict[str, float] | None = None,
        R: dict[str, float] | None = None,
        timeout_s: float = 0.2,
    ):
        q = {**DEFAULT_Q, **(Q or {})}
        r = {**DEFAULT_R, **(R or {})}
        self.Q = _diag(STATE_KEYS, q, DEFAULT_Q)
        self.R_pose = {k: float(r.get(k, DEFAULT_R[k])) for k in ("x", "y", "yaw")}
        self.timeout_s = timeout_s
        self.x = np.zeros(6)
        self.P = np.eye(6)
        self._inited = False
        self._last_valid: float | None = None
        self._t = 0.0

    def reset(self, x0: np.ndarray) -> None:
        self.x = np.asarray(x0, dtype=float).reshape(6).copy()
        self.P = np.diag([1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2])
        self._inited = True
        self._last_valid = 0.0
        self._t = 0.0

    def step(
        self,
        dt: float,
        onboard: Measurement | None,
        external: Measurement | None,
    ) -> tuple[np.ndarray | None, bool]:
        dt = float(max(dt, 1e-9))
        self._t += dt
        imu = onboard if has_imu(onboard) else None
        pose = external if has_pose(external) else None
        if not self._inited:
            if pose is not None:
                self.x = state_from_values(pose.values)
            self._inited = True
        self._predict(dt, imu)
        if pose is not None:
            self._update_pose(pose)
        got = imu is not None or pose is not None
        if got:
            self._last_valid = self._t
        ok = True
        if self.timeout_s > 0 and self._last_valid is not None:
            if (self._t - self._last_valid) > self.timeout_s:
                ok = False
        elif self.timeout_s > 0 and not got and self._last_valid is None:
            ok = False
        if not got and pose is None and imu is None and self.timeout_s > 0:
            # no sensors this tick: still a prediction, but armed callers want invalid
            # only after timeout. First tick with neither is invalid.
            if self._last_valid is None:
                ok = False
        self.x[2] = wrap_angle(float(self.x[2]))
        return self.x.copy(), ok

    def _predict(self, dt: float, imu: Measurement | None) -> None:
        x, y, th, vx, vy, om = self.x
        F = np.eye(6)
        if imu is not None and "ax" in imu.values and "ay" in imu.values:
            ax = float(imu.values["ax"])
            ay = float(imu.values["ay"])
            gyro = float(imu.values["gyro_z"]) if "gyro_z" in imu.values else om
            c, s = np.cos(th), np.sin(th)
            ax_i = c * ax - s * ay
            ay_i = s * ax + c * ay
            dax_dth = -s * ax - c * ay
            day_dth = c * ax - s * ay
            x = x + vx * dt + 0.5 * ax_i * dt * dt
            y = y + vy * dt + 0.5 * ay_i * dt * dt
            th = wrap_angle(th + gyro * dt)
            vx = vx + ax_i * dt
            vy = vy + ay_i * dt
            om = gyro
            F[0, 2] = 0.5 * dt * dt * dax_dth
            F[0, 3] = dt
            F[1, 2] = 0.5 * dt * dt * day_dth
            F[1, 4] = dt
            F[3, 2] = dt * dax_dth
            F[4, 2] = dt * day_dth
            F[5, 5] = 0.0
        else:
            x = x + vx * dt
            y = y + vy * dt
            th = wrap_angle(th + om * dt)
            F[0, 3] = dt
            F[1, 4] = dt
            F[2, 5] = dt
        self.x = np.array([x, y, th, vx, vy, om], dtype=float)
        self.P = F @ self.P @ F.T + self.Q

    def _update_pose(self, pose: Measurement) -> None:
        keys = [k for k in ("x", "y", "yaw") if k in pose.values]
        if not keys:
            return
        idx = {"x": 0, "y": 1, "yaw": 2}
        m = len(keys)
        H = np.zeros((m, 6))
        z = np.zeros(m)
        R = np.zeros((m, m))
        for i, k in enumerate(keys):
            H[i, idx[k]] = 1.0
            z[i] = float(pose.values[k])
            R[i, i] = self.R_pose[k]
        y = z - H @ self.x
        if "yaw" in keys:
            y[keys.index("yaw")] = wrap_angle(float(y[keys.index("yaw")]))
        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = self.P @ H.T @ np.linalg.pinv(S)
        self.x = self.x + K @ y
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P
        self.x[2] = wrap_angle(float(self.x[2]))


def _timeout_from_sources(external, onboard, nav) -> float:
    times = []
    for src in (external, onboard):
        if src is None:
            continue
        t = float(getattr(src, "timeout_s", 0.0) or 0.0)
        if t > 0:
            times.append(t)
    if nav is not None:
        # EKF stale window: a bit looser than a single HTTP timeout
        pass
    return max(times) if times else 0.2


@dataclass
class NavigationStack:
    """Drivers + estimator. `step(dt)` → (state6 or None, ok)."""

    estimator: Any
    external: Any | None = None
    onboard: Any | None = None
    name: str = "passthrough"
    last_onboard: Any | None = None
    last_external: Any | None = None

    def reset(self, x0: np.ndarray) -> None:
        self.estimator.reset(x0)
        self.last_onboard = None
        self.last_external = None

    def step(self, dt: float, now: float | None = None) -> tuple[np.ndarray | None, bool]:
        onboard_m = self.onboard.measure(now) if self.onboard is not None else None
        external_m = self.external.measure(now) if self.external is not None else None
        self.last_onboard = onboard_m
        self.last_external = external_m
        return self.estimator.step(dt, onboard_m, external_m)

    def close(self) -> None:
        for drv in (self.external, self.onboard):
            if drv is not None and hasattr(drv, "close"):
                try:
                    drv.close()
                except Exception:
                    pass


def build_navigation(
    spec,
    plant: Plant,
    *,
    mocap=None,
    replay: str | Path | None = None,
    armed: bool = False,
    seed: int | None = None,
) -> NavigationStack:
    """Wire JSON `navigation` (or legacy mocap / sim) to drivers + estimator."""
    rng = np.random.default_rng(seed)
    nav = getattr(spec, "navigation", None)

    if mocap is not None:
        est = PassthroughEstimator()
        return NavigationStack(estimator=est, external=PoseSourceDriver(mocap), name="passthrough")

    if replay is not None:
        from airbearing.telemetry import CsvReplay

        est = PassthroughEstimator()
        return NavigationStack(estimator=est, external=CsvReplay(str(replay)), name="passthrough")

    if nav is None:
        # Legacy: armed HTTP mocap or simulated plant-as-camera.
        if armed and spec.mocap.enabled:
            from airbearing.telemetry import HttpMocap

            est = PassthroughEstimator()
            return NavigationStack(
                estimator=est,
                external=HttpMocap(spec.mocap.endpoint, spec.mocap.timeout_s),
                name="passthrough",
            )
        from airbearing.telemetry import SimulatedMocap

        est = PassthroughEstimator()
        return NavigationStack(estimator=est, external=SimulatedMocap(plant), name="passthrough")

    kind = (nav.estimator or "passthrough").lower()
    external = None
    onboard = None
    if nav.external is not None:
        # csv path may be filled by replay override already handled
        src = nav.external
        if replay is not None and src.type == "csv_replay":
            from airbearing.telemetry import CsvReplay

            meas = tuple(src.meas) if src.meas else ("x", "y", "yaw", "vx", "vy", "omega")
            external = CsvReplay(str(replay), meas=meas)
        else:
            if src.type == "sim" and not src.table_size:
                src.table_size = spec.table_size
            external = make_driver(src, role="external", plant=plant, rng=rng, replay=replay)
    if nav.onboard is not None:
        src = nav.onboard
        if src.type == "sim" and not src.table_size:
            src.table_size = spec.table_size
        onboard = make_driver(src, role="onboard", plant=plant, rng=rng)

    timeout = _timeout_from_sources(nav.external, nav.onboard, nav)
    if kind == "ekf":
        est = PlanarEKF(Q=nav.Q, R=nav.R, timeout_s=timeout)
    else:
        est = PassthroughEstimator()
    return NavigationStack(estimator=est, external=external, onboard=onboard, name=kind)
