# Describe your satellite

Edit JSON, not Python. Schema: `schemas/vehicle.schema.json` (`additionalProperties: false`).

Shipped layouts live in `examples/vehicles/`. Save **your** file under `vehicles/` (the editor creates the folder).

## Minimum file

```json
{
  "name": "my_fans",
  "mass": 5.0,
  "Iz": 0.09,
  "com": [0.0, 0.0],
  "table_size": 2.0,
  "thrusters": [
    {
      "id": "F1",
      "position": [0.18, 0.0],
      "force_direction": [0.0, 1.0],
      "F_max": 0.3,
      "type": "pwm_fan",
      "bidirectional": true,
      "tau": 0.18,
      "deadman_ms": 100
    }
  ]
}
```

Open the visual builder (`airbearing edit-vehicle`), copy `vehicles/YOUR_SATELLITE.json.example`, or run `airbearing new-vehicle`.

## Fields (SI)

| Field | Meaning |
|-------|---------|
| `mass`, `Iz` | kg, kg m² about COM |
| `com` | body-frame COM `[x, y]` m (offset couples force into torque) |
| `table_size` | side length of the square table, origin at centre |
| `thrusters[].position` | body-frame location, m |
| `thrusters[].force_direction` | force **on the vehicle** (opposite the jet). Normalized on load |
| `thrusters[].F_max` | newtons. **Calibrate** — see LAB.md |
| `type` | `binary_solenoid` \| `pwm_fan` \| `continuous` |
| `min_pulse_ms` | solenoid floor |
| `tau` | fan first-order spin-up (s) |
| `bidirectional` | fans/continuous: command in `[-1, 1]` |
| `reaction_wheel` | yaw wheel; `sim_only: true` unless you write a driver |
| `mocap` | optional HTTP pose; real mode refuses null |

## Controllability

`airbearing check vehicles/my_fans.json` tries to reach **both signs** of `Fx`, `Fy`, `Mz` with the actuator cone. Three *unidirectional* jets cannot positively span `R³` — `examples/vehicles/micro_3thruster.json` exists to prove the allocator is not hard-coded to eight, and to make that warning real. Fix: add a thruster, or set `bidirectional: true`.

## Four-fan student build

Plus layout, bidirectional computer fans, ~4–6 kg, 1.5–2.5 m table. Start from `examples/vehicles/fan_plus.json`, weigh the vehicle, measure `Iz` with a bifilar hang or a yaw tap test, put each fan on a scale (LAB.md), save as `vehicles/my_fans.json`, then `airbearing run --vehicle vehicles/my_fans.json`.
