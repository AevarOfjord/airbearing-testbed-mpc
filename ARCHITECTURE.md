# Architecture (one page)

```
                 vehicles/*.json  ──►  SatelliteSpec (schemas/vehicle.schema.json)
                                              │
   pose (sim plant or mocap) ──► Controller ──┤
                         MPC / PD / LQR       │  body wrench [Fx, Fy, Mz]
                                              ▼
                                        Allocator (n-agnostic QP)
                                              │  cmd ∈ ℝⁿ
                                              ▼
                                        Safety (deadman, table, null telemetry)
                                         /          \
                                      sim            real --armed
                                   Plant.step     serial gateway
                                   mock mocap     refuse null pose
```

**Unified runtime** (`airbearing.runtime.Runtime`): one timed loop. Simulation swaps in `Plant` + `SimulatedMocap`. Hardware swaps in a gateway + HTTP mocap. Controllers and the allocator do not know which.

**Allocator** builds `B ∈ ℝ^{3×n}` from each thruster’s position, force direction, `F_max`, and COM. Bounds depend on type (`[0,1]`, `[-1,1]`). n = 3, 4, 6, 8, … is a JSON problem.

**Linear MPC** (`cvxpy`, OSQP then Clarabel): inertial 6-state, body wrench input, heading frozen over the horizon, rebuilt when yaw drifts. No Gurobi.

**Firmware** (optional): `firmware/solenoid_gateway` (`CMD:<bitmask>:<duration_ms>`) and `firmware/fan_pwm_gateway` (`PWM:d0,d1,…`) with a 100 ms deadman.

**Out of scope:** Windows COM ports, vendor optimizer DLLs, NatNet internals, CAD, fifteen handoff PDFs.

**Visual builder** (`python -m airbearing edit-vehicle`): pygame table; save is restricted to `vehicles/`.
**Live twin** (`view`): same `Runtime`, keyboard teleop, MPC handover, `--record` PNG.
**Identify** (`identify`): mass / Iz / F_max from `log.csv`; layout stays in JSON.
