# Hardware

Two optional Arduino sketches live in `firmware/`. Simulation does not need them.

## Compressed-air solenoids (UK-class octagon)

`firmware/solenoid_gateway/solenoid_gateway.ino`

```
CMD:<bitmask>:<duration_ms>\n
```

Bit `i` is thruster `T{i+1}`. If no CMD arrives within **100 ms**, every valve is forced OFF (deadman). `duration_ms` is a second timeout so a stuck high bit still drops.

Typical: 24 kg class, eight valves, ~0.5–1 N each, 6 m table. **Do not trust the JSON `F_max` until you have used a scale.**

## Computer-fan PWM

`firmware/fan_pwm_gateway/fan_pwm_gateway.ino`

```
PWM:<d0>,<d1>,<d2>,...\n
```

Duties in `[-1,1]` (sign → DIR pin, magnitude → PWM). Deadman **100 ms → all zero**.

Cheap student build: four bidirectional fans on a plus frame, air hockey table or glass with a balloon bearing, webcam pose later. `tau` in JSON is the fan spin-up you measure with a step on the scale.

## Host flags

```
python -m airbearing run --vehicle vehicles/uk_solenoid_octagon.json \
  --armed --port /dev/ttyUSB0
```

`--armed` is required. If mocap is enabled and the pose read fails, the runtime **sends zeros and aborts**. There is no “fire open-loop on hope.”

For a first bring-up, keep mocap disabled and stay in simulation until the JSON matches the object you weighed.

## Pose

`mocap.endpoint` expects JSON `{"x","y","yaw"}` (optional `vx,vy,omega`). A mock is used in sim. Wire OptiTrack/webcam yourself; this kit does not vendor-lock NatNet.
