"""Top-down animation and comparison plots. Headless-friendly (Agg)."""

from __future__ import annotations

from pathlib import Path

import numpy as np

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, RegularPolygon
from matplotlib.animation import FuncAnimation, PillowWriter

from airbearing.runtime import RunResult
from airbearing.spec import SatelliteSpec


def _states(result: RunResult) -> np.ndarray:
    return np.array([r.state for r in result.logs])


def plot_trajectory(spec: SatelliteSpec, result: RunResult, path: Path) -> None:
    st = _states(result)
    fig, axes = plt.subplots(2, 2, figsize=(9.5, 8))
    ax = axes[0, 0]
    half = spec.table_size / 2
    ax.add_patch(Rectangle((-half, -half), spec.table_size, spec.table_size, fill=False, lw=2, color="0.4"))
    ax.plot(st[:, 0], st[:, 1], color="#1f6feb", lw=2, label="path")
    ax.scatter(st[0, 0], st[0, 1], c="#2da44e", s=40, zorder=5, label="start")
    ax.scatter(st[-1, 0], st[-1, 1], c="#cf222e", s=40, zorder=5, label="end")
    g = result.logs[-1].ref
    ax.scatter(g[0], g[1], marker="x", c="k", s=60, label="goal")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{spec.name}  ·  {result.success and 'OK' or 'miss'}")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    t = np.array([r.t for r in result.logs])
    axes[0, 1].plot(t, st[:, 0], label="x")
    axes[0, 1].plot(t, st[:, 1], label="y")
    axes[0, 1].plot(t, st[:, 2], label="yaw")
    axes[0, 1].set_title("pose")
    axes[0, 1].legend(fontsize=8)
    axes[0, 1].grid(True, alpha=0.3)

    u = np.array([r.cmd for r in result.logs])
    axes[1, 0].imshow(u.T, aspect="auto", cmap="magma", interpolation="nearest", origin="lower")
    axes[1, 0].set_title("actuator commands")
    axes[1, 0].set_ylabel("thruster index")
    axes[1, 0].set_xlabel("step")

    ms = np.array([r.mpc_ms + r.alloc_ms for r in result.logs])
    axes[1, 1].plot(t, ms, color="#8250df")
    axes[1, 1].axhline(spec.control_dt * 1000, color="#cf222e", ls="--", lw=1, label="period")
    axes[1, 1].set_title("solver time (ms)")
    axes[1, 1].legend(fontsize=8)
    axes[1, 1].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def animate(spec: SatelliteSpec, result: RunResult, path: Path, stride: int = 2) -> None:
    st = _states(result)
    t = np.array([r.t for r in result.logs])
    u = np.array([r.cmd for r in result.logs])
    half = spec.table_size / 2
    fig, ax = plt.subplots(figsize=(6.2, 6.2))
    ax.set_xlim(-half, half)
    ax.set_ylim(-half, half)
    ax.set_aspect("equal")
    ax.add_patch(Rectangle((-half, -half), spec.table_size, spec.table_size, fill=False, lw=2, color="0.35"))
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    trail, = ax.plot([], [], color="#1f6feb", lw=1.5, alpha=0.8)
    body = RegularPolygon((0, 0), 8, radius=spec.hull_radius, orientation=0, fc="#79c0ff", ec="#0969da", lw=1.5)
    ax.add_patch(body)
    jet_lines = [ax.plot([], [], color="#ff7b72", lw=3, solid_capstyle="round")[0] for _ in spec.thrusters]
    goal = result.logs[0].ref
    ax.scatter([goal[0]], [goal[1]], marker="x", c="k", s=50, zorder=6)
    title = ax.set_title("")

    idx = list(range(0, len(st), stride)) or [0]

    def update(frame_i: int):
        i = idx[frame_i]
        x, y, th = st[i, 0], st[i, 1], st[i, 2]
        c, s = np.cos(th), np.sin(th)
        body.xy = (x, y)
        body.orientation = th
        trail.set_data(st[: i + 1, 0], st[: i + 1, 1])
        for k, thr in enumerate(spec.thrusters):
            pb = thr.position
            pi = np.array([x + c * pb[0] - s * pb[1], y + s * pb[0] + c * pb[1]])
            fb = -thr.force_direction * (0.08 + 0.18 * abs(u[i, k]))  # jet opposite to force
            fi = np.array([c * fb[0] - s * fb[1], s * fb[0] + c * fb[1]])
            jet_lines[k].set_data([pi[0], pi[0] + fi[0]], [pi[1], pi[1] + fi[1]])
            jet_lines[k].set_alpha(0.15 + 0.85 * min(1.0, abs(u[i, k])))
        title.set_text(f"{spec.name}  t={t[i]:.2f}s")
        return [trail, body, title, *jet_lines]

    anim = FuncAnimation(fig, update, frames=len(idx), interval=40, blit=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    anim.save(path, writer=PillowWriter(fps=20))
    plt.close(fig)


def plot_compare(results: list[tuple[str, RunResult]], path: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.4))
    colors = ["#1f6feb", "#bf3989", "#2da44e"]
    for (name, res), col in zip(results, colors):
        st = _states(res)
        t = [r.t for r in res.logs]
        axes[0].plot(st[:, 0], st[:, 1], color=col, lw=2, label=name)
        err = np.hypot(
            st[:, 0] - np.array([r.ref[0] for r in res.logs]),
            st[:, 1] - np.array([r.ref[1] for r in res.logs]),
        )
        axes[1].plot(t, err, color=col, lw=2, label=name)
    axes[0].set_aspect("equal")
    axes[0].set_title("same mission, different actuators")
    axes[0].set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)
    axes[1].set_title("position error (m)")
    axes[1].set_xlabel("t (s)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
