"""Planar 3-DOF air-bearing plant. Nearly frictionless translation + yaw."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from airbearing.spec import SatelliteSpec


def rot2(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def wrap_angle(a: float) -> float:
    return float((a + np.pi) % (2 * np.pi) - np.pi)


@dataclass
class Plant:
    spec: SatelliteSpec
    # inertial: x, y, theta, vx, vy, omega
    state: np.ndarray = field(default_factory=lambda: np.zeros(6))
    f_actual: np.ndarray = field(default_factory=lambda: np.zeros(0))  # along each thruster axis, Newtons
    rw_momentum: float = 0.0
    t: float = 0.0

    def __post_init__(self) -> None:
        self.state = np.asarray(self.state, dtype=float).reshape(6)
        self.f_actual = np.zeros(self.spec.n_thrusters)

    @property
    def pose(self) -> np.ndarray:
        return self.state[:3].copy()

    def reset(self, x: float, y: float, theta: float, vx: float = 0.0, vy: float = 0.0, omega: float = 0.0) -> None:
        self.state = np.array([x, y, theta, vx, vy, omega], dtype=float)
        self.f_actual[:] = 0.0
        self.t = 0.0
        if self.spec.reaction_wheel:
            self.rw_momentum = self.spec.reaction_wheel.initial_momentum

    def wrench_from_forces(self, forces: np.ndarray) -> np.ndarray:
        """Body wrench from per-thruster force magnitudes (N) along force_direction."""
        Fx = Fy = Mz = 0.0
        com = self.spec.com
        for f, t in zip(forces, self.spec.thrusters, strict=True):
            fb = f * t.force_direction
            r = t.position - com
            Fx += fb[0]
            Fy += fb[1]
            Mz += r[0] * fb[1] - r[1] * fb[0]
        return np.array([Fx, Fy, Mz])

    def step(self, cmd: np.ndarray, dt: float | None = None, rw_torque: float = 0.0) -> np.ndarray:
        dt = self.spec.sim_dt if dt is None else dt
        cmd = np.asarray(cmd, dtype=float).reshape(self.spec.n_thrusters)
        spec = self.spec
        target_F = np.zeros(spec.n_thrusters)
        for i, t in enumerate(spec.thrusters):
            c = float(np.clip(cmd[i], t.cmd_min, t.cmd_max))
            if t.type == "binary_solenoid":
                # Average force = duty * F_max (PWM of a valve over the control interval).
                # Min-pulse is enforced by the gateway / allocator, not per sim substep —
                # comparing 30 ms to sim_dt=20 ms would snap every drip of duty to 1.0
                # and opposing thrusters would cancel.
                target_F[i] = c * t.F_max
                self.f_actual[i] = target_F[i]
            elif t.type == "pwm_fan":
                target_F[i] = c * t.F_max
                tau = max(t.tau, 1e-4)
                self.f_actual[i] += dt * (target_F[i] - self.f_actual[i]) / tau
            else:  # continuous
                target_F[i] = c * t.F_max
                self.f_actual[i] = target_F[i]

        wrench_b = self.wrench_from_forces(self.f_actual)
        if spec.reaction_wheel is not None:
            rw = spec.reaction_wheel
            tau_w = float(np.clip(rw_torque, -rw.max_torque, rw.max_torque))
            # saturate on momentum
            next_h = self.rw_momentum + tau_w * dt
            if abs(next_h) > rw.max_momentum:
                tau_w = 0.0
                next_h = np.clip(self.rw_momentum, -rw.max_momentum, rw.max_momentum)
            self.rw_momentum = next_h
            wrench_b = wrench_b + np.array([0.0, 0.0, tau_w])

        x, y, th, vx, vy, om = self.state
        R = rot2(th)
        F_i = R @ wrench_b[:2]
        ax = F_i[0] / spec.mass - spec.linear_damping * vx / spec.mass
        ay = F_i[1] / spec.mass - spec.linear_damping * vy / spec.mass
        alpha = wrench_b[2] / spec.Iz - spec.rotational_damping * om / spec.Iz

        # semi-implicit Euler (stable enough for teaching dt)
        vx += ax * dt
        vy += ay * dt
        om += alpha * dt
        x += vx * dt
        y += vy * dt
        th = wrap_angle(th + om * dt)
        self.state = np.array([x, y, th, vx, vy, om])
        self.t += dt
        return self.state.copy()
