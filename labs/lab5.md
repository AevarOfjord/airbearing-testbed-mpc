# Lab 5 — Onboard vs external vs fused

No table required. Sim sensors run off the plant.

```bash
make lab5
```

Vehicle JSON `navigation` selects the pose the MPC sees:

| JSON | What the estimator uses |
|------|-------------------------|
| omit `onboard` | external only (mocap / webcam / replay) |
| omit `external` | onboard IMU only |
| both | fuse (EKF: IMU predict, pose update) |
| `"estimator": "passthrough"` | copy the pose source (legacy) |
| `"estimator": "ekf"` | planar EKF |

`examples/vehicles/fan_plus.json` is sim passthrough. `examples/vehicles/fan_plus_fused.json` is noisy mocap + IMU.

`--armed` still zeros and aborts on an invalid estimate (including a timed-out external source). Actuator serial (`--port`) is not the IMU serial (`navigation.onboard.port`).
