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

`--armed` is required. If mocap is enabled and the pose read fails, the runtime **sends zeros and aborts**. There is no “fire open-loop on hope.”

For a first bring-up, keep mocap disabled and stay in simulation until the JSON matches the object you weighed.

## Pose

`mocap.endpoint` expects JSON `{"x","y","yaw"}` (optional `vx,vy,omega`), SI. A mock is used in sim. Replay a CSV with `airbearing run --replay path.csv` (schema comment `# airbearing_log schema_version=1 units=SI`).
