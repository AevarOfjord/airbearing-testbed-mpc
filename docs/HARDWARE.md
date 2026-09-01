# Put it on hardware

Two optional Arduino sketches live in `firmware/`. Simulation does not need them. SI units on the host (`F_max` in newtons, pose in metres and radians).

## Compressed-air solenoids

`firmware/solenoid_gateway/solenoid_gateway.ino`

```
CMD:<bitmask>:<duration_ms>\n
```

Bit `i` is thruster `T{i+1}`. If no CMD arrives within **100 ms**, every valve is forced OFF (deadman). `duration_ms` is a second timeout so a stuck high bit still drops.

Example layout: `examples/vehicles/solenoid_octagon.json` (eight on/off jets). **Do not trust JSON `F_max` until you have used a scale.**

## Computer-fan PWM

`firmware/fan_pwm_gateway/fan_pwm_gateway.ino`

```
PWM:<d0>,<d1>,<d2>,...\n
```

Duties in `[-1,1]` (sign → DIR pin, magnitude → PWM). Deadman **100 ms → all zero**.

Typical student build: four bidirectional fans on a plus frame (`examples/vehicles/fan_plus.json`), air-hockey table or glass with a porous bearing, webcam or mocap pose later. `tau` in JSON is the fan spin-up you measure with a step on the scale.

## Host flags

```
airbearing run --vehicle vehicles/mine.json --armed --port /dev/ttyUSB0 --dashboard
```

`--armed` is required. `--port` is the **actuator** gateway. Configure IMU serial under `navigation.onboard.port`.

For a first bring-up, stay in simulation (`type: sim` sensors, or omit `navigation`) until the JSON matches the object you weighed.

## Pose and IMU

Prefer vehicle JSON `navigation` over the legacy `mocap` block.

External pose: `http_mocap` JSON `{"x","y","yaw"}` (optional `vx,vy,omega`), SI; `csv_replay`; `webcam_aruco` (OpenCV extra, skipped if missing); or `sim`. Replay a CSV with `airbearing run --replay path.csv` (schema comment `# airbearing_log schema_version=1 units=SI`).

Onboard IMU: `firmware/onboard_imu` prints `{"ax","ay","gyro_z"}` at ~100 Hz (body frame, SI) on **its own serial port**. Deadman stays on the actuator gateway, not the IMU stream. `navigation.onboard.port` (e.g. `/dev/ttyUSB1`) is not `--port`.

`--armed` zeros commands and aborts on an invalid estimate (timed-out or missing external pose when one is configured). There is no “fire open-loop on hope.”
