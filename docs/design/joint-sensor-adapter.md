# Joint sensor adapter interface (design)

This document defines the interface for sensors on robot joints (encoders and joint torque).
The survey (comparison of simulators and real devices) is [joint-sensor-survey.md](joint-sensor-survey.md). The interface follows the same approach as the existing sensor interfaces ([force-torque-adapter.md](force-torque-adapter.md) and others).

This document defines the meaning of the configuration values, and the order and formulas for adding errors. The goal is that, with the same configuration, the same truth, and the same seed, every implementation outputs values with the same meaning.

## 1. Purpose and scope

- This is the interface for a sensor that measures the angle, velocity, and torque of one joint. The values resemble one joint's entry in ROS `sensor_msgs/JointState`.
- One instance is one joint. However many joints of the same model there are, they can use the same configuration file (random numbers are separate per sensor name).
- Revolute joints are assumed (units rad, rad/s, N·m).

## 2. Host and adapter responsibilities

```text
Host: physics truth (joint angle and velocity, actuator force, torque transmitted through the joint)
                                                        │
Adapter (every physics step):                           ▼
        delay
Adapter (at each sample period boundary):
        angle: deflection → backlash → offset → cyclic error → noise → round to counts → wrap per revolution → joint angle
        velocity: truth, or difference of rounded angles → round to the unit
        torque: torque sensor or estimate from current → scale error, bias → noise → saturation → resolution
                                                        │
Host:                                                   ▼
        controller, log, PDU
```

| Owner | Content |
|---|---|
| Host | Physics truth. The gearbox is not included in the physics (the joint angle is the joint-side value) |
| Adapter | The error model in section 4. When measuring on the motor side, the gear ratio, backlash, and deflection are also owned by the adapter |

## 3. Decisions (principles)

### 3.1 Truth

The host passes the following values every physics step.

| Value | Unit | Meaning |
|---|---|---|
| Angle q | rad | Joint angle (the physics joint value) |
| Velocity q̇ | rad/s | Joint angular velocity |
| Actuator force τ_act | N·m | Sum of the forces of the actuators that drive the joint (`qfrc_actuator` for MuJoCo) |
| Torque transmitted through the joint τ_tr | N·m | Torque about the joint axis that the parent side applies to the child side (what a torque sensor on the joint measures). When holding still, it equals τ_act |

When the host cannot compute the torque transmitted through the joint, it passes τ_act and reports this in the capability table (the torque sensor value then does not include friction).

### 3.2 Time, random numbers, reset, log

These are the same as in the existing sensor interfaces.

- The sample period is rounded to an integer multiple of the physics step (with a warning when it does not match). The sensor measures at the period boundaries from time 0, and returns the previous value in between.
- Random numbers come from the seed, the vehicle, and the sensor name, with a separate stream per characteristic (`offset`, `accuracy`, `noise`, `torque_unit`, `torque_noise`).
- `reset()` returns the delay history, the backlash state, the initial angle for incremental encoders, the velocity history, the per-unit errors drawn at creation, and the random streams to their state at creation.

### 3.3 Value flags

- Saturation: the common bit `kSensorFlagSaturated`. It is set when the torque was clipped at the end of the range.

## 4. Error model (formulas)

N is the gear ratio (1 when measuring on the joint side), C is the counts per revolution, and T is the sample period. U(a, b) is the uniform distribution, and N(0, σ) is the normal distribution.

### 4.1 Every physics step

| Stage | Formula or rule |
|---|---|
| Delay | Use the truth from `delay` earlier (truncated to the physics step; while the history is not long enough, the truth at time 0) |

### 4.2 Angle (at each sample period boundary)

From the truth q and τ_tr after the delay, compute in the following order.

| Stage | Formula or rule |
|---|---|
| Deflection | On the motor side, when stiffness K > 0: θ = q + τ_tr / K. Otherwise θ = q (expressed as a joint-side angle) |
| Backlash | On the motor side, when the backlash width b > 0, with play state p: if θ − p > b/2 then p = θ − b/2; if θ − p < −b/2 then p = θ + b/2; otherwise p stays. p = θ at the first value. After this, p is used instead of θ |
| Offset | Absolute: θ ← θ + o. o is drawn at creation from U(−O, O) |
| Shaft angle | s = N · θ (encoder shaft angle) |
| Cyclic error | s ← s + A · sin(s + ψ). A is drawn at creation from U(0, `accuracy`), ψ from U(0, 2π) |
| Incremental | s ← s − s₀. s₀ is the first s measured after creation or after a reset |
| Noise | s ← s + N(0, σ_c · 2π / C) (σ_c in counts) |
| Counts | c = round(s · C / 2π) (integer; 0.5 rounds away from 0) |
| Wrap per revolution | When `wrap` is `single_turn`, c ← c mod C (0 to C−1) |
| Angle value | Angle output = c · 2π / (C · N) |

If `encoder` is not written, the angle is q as is, and the count is 0.

### 4.3 Velocity

| `method` | Formula |
|---|---|
| `true` (default) | v = q̇ (truth after the delay) |
| `difference` | v = (c_k − c_{k−w}) · 2π / (C · N · w · T). c is the count before wrapping per revolution, w is `window`. When the value from w samples earlier does not exist yet, use the oldest available value and divide by the time between them. The first value is 0 |

- When `lsb` > 0, v ← lsb · round(v / lsb).

### 4.4 Torque

| Stage | Formula or rule |
|---|---|
| What is measured | `source` is `sensor`: x = τ_tr. `current`: x = τ_act + F_c · sgn(q̇) + F_v · q̇ (the current also includes the part that overcomes friction; sgn(0) = 0) |
| Scale error, bias | y = x · (1 + g) + b. g is drawn at creation from U(−S, S), b from U(−B, B) |
| Noise | y ← y + N(0, σ) |
| Saturation | If \|y\| > R, clip to ±R and set the saturation flag |
| Resolution | If q_τ > 0, y ← q_τ · round(y / q_τ) |

If `torque` is not written, no torque is output (`has_effort` is false).

## 5. Configuration

Angles can be written as a number (rad) or a string with a unit (`"0.25deg"`, `"15arcmin"`, `"18arcsec"`). Velocity units are a number (rad/s) or `"0.229rpm"`. Ratios are `"2%"` (a ratio to the range R).

| Key | Meaning | When omitted |
|---|---|---|
| `type` | `"joint"` | Required |
| `update_rate` | Sample period [Hz] | Required |
| `delay` | Delay [s] | 0 |
| `encoder.side` | `"joint"` or `"motor"` | `"joint"` |
| `encoder.gear_ratio` | Gear ratio N (used only for `motor`) | 1 |
| `encoder.counts_per_rev` or `encoder.bits` | Counts per revolution C (2^bits for `bits`) | Required (when `encoder` is written) |
| `encoder.kind` | `"absolute"` or `"incremental"` | `"absolute"` |
| `encoder.wrap` | `"none"` or `"single_turn"` | `"none"` |
| `encoder.offset` | Upper limit O of the absolute offset (joint-side angle) | 0 |
| `encoder.accuracy` | Upper limit of the cyclic error amplitude (encoder shaft angle) | 0 |
| `encoder.backlash` | Backlash width b (joint-side angle; only for `motor`) | 0 |
| `encoder.stiffness` | Torsional stiffness K [N·m/rad] (only for `motor`) | 0 (no deflection) |
| `encoder.noise_counts` | Noise σ_c [counts] | 0 |
| `velocity.method` | `"true"` or `"difference"` | `"true"` |
| `velocity.window` | Difference window w [samples] | 1 |
| `velocity.lsb` | Velocity unit | 0 (no rounding) |
| `torque.source` | `"sensor"` or `"current"` | Required (when `torque` is written) |
| `torque.range` | Range R [N·m] | Required (when `torque` is written) |
| `torque.scale_error` | Upper limit S of the scale error | 0 |
| `torque.bias` | Upper limit B of the bias (number or % of range) | 0 |
| `torque.noise` | Noise σ [N·m] | 0 |
| `torque.resolution` | Resolution q_τ [N·m] | 0 |
| `torque.friction` | Friction for `current` (`coulomb` [N·m], `viscous` [N·m·s/rad]) | 0 |

A datasheet "accuracy: ±2 % of maximum torque" is written as `"2%"` in `torque.bias`. Unknown keys, values out of range, and unknown units cause an error at creation.

Example (Feetech STS3215, a joint of the SO-101):

```json
{
  "type": "joint",
  "update_rate": 100,
  "encoder": {
    "side": "joint",
    "bits": 12,
    "kind": "absolute"
  },
  "velocity": { "method": "difference", "window": 1, "lsb": "0.01465rpm" }
}
```

The STS3215 velocity unit is 1 step/s (50 steps/s = 0.732 rpm, so 1 step/s ≈ 0.01465 rpm). The STS3215 datasheet does not state how the velocity is produced, so `method` is provisionally set to `difference`.

## 6. Interface shape

| Name | Content |
|---|---|
| `JointSensorAdapterSetup` | What the host passes at creation (the common `SensorAdapterSetup`) |
| `JointSensorTruth` | Truth every physics step: time, angle, velocity, actuator force, torque transmitted through the joint, whether the transmitted torque could be computed |
| `JointSensorSample` | Output of one step: whether a new value was measured, measurement time, angle, velocity, torque, whether torque is output, count, flags |
| `JointSensorAdapterCapabilities` | Declaration of the supported features |
| `IJointSensorAdapter` | `step`, `capabilities`. `reset`, `log_columns`, and `log_values` are inherited from the common `ISensorAdapter` |

## 7. Capability table

| Item | Meaning |
|---|---|
| `delay` | Delay |
| `encoder_counts` | Rounding to counts |
| `motor_side` | Measuring on the motor side (gear ratio) |
| `backlash`, `stiffness` | Backlash, deflection |
| `offset`, `cyclic_error`, `noise` | Angle offset, cyclic error, noise |
| `incremental`, `wrap` | Incremental, wrap per revolution |
| `velocity_from_difference`, `velocity_lsb` | Velocity from differences, rounding to the velocity unit |
| `torque_sensor`, `torque_from_current` | Torque sensor, estimate from current |
| `torque_errors` | Torque scale error, bias, noise, saturation, resolution |
| `deterministic_with_seed` | Same values with the same seed, the same input, and the same call order |
| `thread_safe_across_instances` | Different instances can be called at the same time from different threads |

## 8. What the host does

- Get the joint angle and velocity, the actuator force, and the torque transmitted through the joint from the physics engine. For MuJoCo, the transmitted torque is the child body's `cfrc_int` moved to the joint position and projected onto the joint axis (computed by `mj_rnePostConstraint`).
- Run any number of joint sensors with one instance per joint.
- Put the output on the joint state PDU (`sensor_msgs/JointState`), collected in joint order.
