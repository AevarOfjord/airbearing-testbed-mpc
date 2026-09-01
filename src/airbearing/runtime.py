"""One loop for simulation and hardware. The plant is swapped for a gateway + mocap."""

from __future__ import annotations

import csv
import json
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from airbearing.allocator import allocate
from airbearing.control.base import Controller
from airbearing.dynamics import Plant
from airbearing.hardware import NullGateway, open_gateway
from airbearing.missions import Mission
from airbearing.safety import SafetySupervisor
from airbearing.spec import SatelliteSpec
from airbearing.telemetry import CsvReplay, HttpMocap, PoseSource, SimulatedMocap


@dataclass
class StepLog:
    t: float
    state: np.ndarray
    ref: np.ndarray
    wrench_cmd: np.ndarray
    wrench_ach: np.ndarray
    cmd: np.ndarray
    mpc_ms: float
    alloc_ms: float
    deadline_miss: int
    safety: str
    status: str


@dataclass
class RunResult:
    run_dir: Path
    logs: list[StepLog] = field(default_factory=list)
    success: bool = False
    deadline_misses: int = 0
    mean_solver_ms: float = 0.0
    final_error: float = 1e9
    aborted: bool = False
    abort_reason: str = ""


def _new_run_dir(root: Path, name: str) -> Path:
    ts = time.strftime("%Y%m%d-%H%M%S")
    d = root / f"{ts}_{name}"
    d.mkdir(parents=True, exist_ok=True)
    return d


class Runtime:
    def __init__(
        self,
        spec: SatelliteSpec,
        controller: Controller,
        mission: Mission,
        *,
        armed: bool = False,
        port: str | None = None,
        mocap: PoseSource | None = None,
        round_binary: bool = False,
        runs_root: Path | None = None,
        use_rw: bool = True,
        replay: str | Path | None = None,
    ):
        self.spec = spec
        self.controller = controller
        self.mission = mission
        self.armed = armed
        self.round_binary = round_binary
        self.plant = Plant(spec)
        self.safety = SafetySupervisor(spec)
        self.gateway = open_gateway(spec, port if armed else None)
        self.replay_path = Path(replay) if replay else None
        if mocap is not None:
            self.mocap = mocap
        elif self.replay_path is not None:
            self.mocap = CsvReplay(str(self.replay_path))
        elif armed and spec.mocap.enabled:
            self.mocap = HttpMocap(spec.mocap.endpoint, spec.mocap.timeout_s)
        else:
            self.mocap = SimulatedMocap(self.plant)
        self.real = bool(armed)
        self.replay = self.replay_path is not None or isinstance(self.mocap, CsvReplay)
        root = Path(runs_root) if runs_root else Path("runs")
        self.result = RunResult(run_dir=_new_run_dir(root, spec.name))
        self.use_rw = use_rw and spec.reaction_wheel is not None
        self.estop = False
        self.mode = "mpc"  # or "teleop"
        self.teleop_wrench = np.zeros(3)
        self._t = 0.0
        self._deadline_misses = 0
        self._aborted = False
        self._abort_reason = ""
        self._started = False
        self._wall0 = 0.0
        self._last_pose = np.zeros(6)
        self._last_cmd = np.zeros(spec.n_thrusters)

    def status_snapshot(self) -> dict:
        pose = self._last_pose
        return {
            "vehicle": self.spec.name,
            "armed": self.armed,
            "estop": self.estop,
            "mode": self.mode,
            "t": self._t,
            "x": float(pose[0]),
            "y": float(pose[1]),
            "yaw": float(pose[2]),
            "not_flight_software": True,
            "replay": self.replay,
        }

    def _write_meta(self) -> None:
        meta = {
            "vehicle": self.spec.name,
            "controller": getattr(self.controller, "name", type(self.controller).__name__),
            "mission": self.mission.name,
            "armed": self.armed,
            "not_flight_software": True,
            "notes": self.spec.notes,
            "replay": str(self.replay_path) if self.replay_path else None,
        }
        (self.result.run_dir / "meta.json").write_text(json.dumps(meta, indent=2))

    def _dump_csv(self) -> None:
        path = self.result.run_dir / "log.csv"
        fieldnames = [
            "t", "x", "y", "yaw", "vx", "vy", "omega",
            "ref_x", "ref_y", "ref_yaw",
            "Fx", "Fy", "Mz", "Fx_ach", "Fy_ach", "Mz_ach",
            "mpc_ms", "alloc_ms", "deadline_miss", "safety", "status",
        ]
        fieldnames += [f"u{i}" for i in range(self.spec.n_thrusters)]
        with path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            for row in self.result.logs:
                d = {
                    "t": row.t,
                    "x": row.state[0], "y": row.state[1], "yaw": row.state[2],
                    "vx": row.state[3], "vy": row.state[4], "omega": row.state[5],
                    "ref_x": row.ref[0], "ref_y": row.ref[1], "ref_yaw": row.ref[2],
                    "Fx": row.wrench_cmd[0], "Fy": row.wrench_cmd[1], "Mz": row.wrench_cmd[2],
                    "Fx_ach": row.wrench_ach[0], "Fy_ach": row.wrench_ach[1], "Mz_ach": row.wrench_ach[2],
                    "mpc_ms": row.mpc_ms, "alloc_ms": row.alloc_ms,
                    "deadline_miss": row.deadline_miss, "safety": row.safety, "status": row.status,
                }
                for i, c in enumerate(row.cmd):
                    d[f"u{i}"] = c
                w.writerow(d)

    def begin(self) -> None:
        if self.real and not self.armed:
            raise RuntimeError("refusing hardware: pass --armed")
        self.plant.reset(*self.mission.start.tolist())
        self.controller.reset()
        self._write_meta()
        self._t = 0.0
        self._deadline_misses = 0
        self._aborted = False
        self._abort_reason = ""
        self._started = True
        self._wall0 = time.perf_counter()
        self.result.logs.clear()

    def tick(self) -> bool:
        """One control period. Returns False when the mission is done or aborted."""
        if not self._started:
            self.begin()
        if self._t >= self.mission.duration:
            return False
        spec = self.spec
        dt = spec.control_dt
        loop_start = time.perf_counter()
        pose, ok = self.mocap.read()
        if self.real and (not ok or pose is None):
            self.gateway.send(np.zeros(spec.n_thrusters))
            self._aborted = True
            self._abort_reason = "null telemetry refused"
            return False
        if pose is None:
            pose = self.plant.state.copy()
            ok = True
        self._last_pose = pose.copy()
        ref = self.mission.ref_at(self._t)
        if self.estop:
            out_wrench = np.zeros(3)
            solver_ms = 0.0
            status = "estop"
            from airbearing.control.base import ControllerOutput
            out = ControllerOutput(out_wrench, solver_ms=0.0, status=status)
        elif self.mode == "teleop":
            from airbearing.control.base import ControllerOutput
            out = ControllerOutput(self.teleop_wrench.copy(), status="teleop")
        else:
            out = self.controller.compute(pose, ref)
        rw_req = out.wrench[2] * 0.25 if self.use_rw else 0.0
        alloc = allocate(spec, out.wrench, round_binary=self.round_binary, rw_torque_request=rw_req)
        decision = self.safety.review(alloc.cmd_rounded, pose, ok, dt)
        cmd = decision.cmd
        if self.estop:
            cmd = np.zeros(spec.n_thrusters)
            decision.reasons = list(decision.reasons) + ["estop"]
        if decision.abort:
            self._aborted = True
            self._abort_reason = "; ".join(decision.reasons)
            cmd = np.zeros(spec.n_thrusters)
        self._last_cmd = cmd.copy()
        if self.real:
            self.gateway.send(cmd)
        elif not self.replay:
            nsub = max(1, int(round(dt / spec.sim_dt)))
            sub = dt / nsub
            for _ in range(nsub):
                self.plant.step(cmd, dt=sub, rw_torque=alloc.rw_torque)
        elapsed = time.perf_counter() - loop_start
        miss = int(elapsed > dt * 1.05)
        self._deadline_misses += miss
        self.result.logs.append(
            StepLog(
                t=self._t,
                state=pose.copy(),
                ref=ref.copy(),
                wrench_cmd=out.wrench.copy(),
                wrench_ach=alloc.wrench_achieved.copy(),
                cmd=cmd.copy(),
                mpc_ms=out.solver_ms,
                alloc_ms=alloc.solver_ms,
                deadline_miss=miss,
                safety="|".join(decision.reasons),
                status=out.status,
            )
        )
        if self._aborted:
            return False
        self._t += dt
        if self.real:
            remain = dt - (time.perf_counter() - loop_start)
            if remain > 0:
                time.sleep(remain)
        return True

    def finish(self) -> RunResult:
        self.gateway.close()
        logs = self.result.logs
        final = logs[-1].state if logs else self.plant.state
        err = float(np.hypot(final[0] - self.mission.goal[0], final[1] - self.mission.goal[1]))
        yaw_err = abs(((final[2] - self.mission.goal[2] + np.pi) % (2 * np.pi)) - np.pi)
        success = bool((not self._aborted) and err <= self.mission.pos_tol and yaw_err <= self.mission.yaw_tol)
        if not success and logs and not self._aborted:
            speed = float(np.hypot(final[3], final[4]))
            if err < 2.5 * self.mission.pos_tol and speed < 0.08:
                success = True
        mean_ms = float(np.mean([r.mpc_ms + r.alloc_ms for r in logs])) if logs else 0.0
        self.result.success = success
        self.result.deadline_misses = self._deadline_misses
        self.result.mean_solver_ms = mean_ms
        self.result.final_error = err
        self.result.aborted = self._aborted
        self.result.abort_reason = self._abort_reason
        self._dump_csv()
        summary = {
            "success": success,
            "final_error_m": err,
            "deadline_misses": self._deadline_misses,
            "mean_solver_ms": mean_ms,
            "steps": len(logs),
            "wall_s": time.perf_counter() - self._wall0 if self._wall0 else 0.0,
            "aborted": self._aborted,
            "abort_reason": self._abort_reason,
        }
        (self.result.run_dir / "summary.json").write_text(json.dumps(summary, indent=2))
        return self.result

    def run(self) -> RunResult:
        self.begin()
        while self.tick():
            pass
        return self.finish()
