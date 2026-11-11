# Joint sensor specification

This is the specification of the adapter for the encoder and joint torque of one revolute joint.
The interface is `joint_sensor_adapter.hpp`. The design is in [joint-sensor-adapter.md](../design/joint-sensor-adapter.md).
This specification states the formulas of that design as numbered requirements that every adapter must follow.

## 1. Requirement IDs and truth

IDs have the form `JNT-ADP-<nnn>` (ADP = adapter).

The truth (`JointSensorTruth`) is defined in the design: joint angle q [rad], joint velocity q̇ [rad/s], actuator force τ_act [N·m] (the total actuator torque on the joint), and transmitted torque τ_tr [N·m] (the torque about the joint axis that the parent side exerts on the child side). If the host cannot compute τ_tr, it passes τ_act and reports this in the capability table. How the host computes the truth from its physics engine is outside this specification.

## 2. Adapter

N is the gear ratio (1 on the joint side), C is the counts per revolution, T is the sample period. U(a, b) is a uniform distribution and N(0, σ) is a normal distribution.

### 2.1 Time, random numbers, reset

| ID | Requirement |
|---|---|
| JNT-ADP-010 | The adapter does not know the physics engine. The host passes the truth as `JointSensorTruth` |
| JNT-ADP-020 | `step` times are 0, step, 2·step, …. Any other time is an error and does not change the state |
| JNT-ADP-030 | The sample period is 1 / `update_rate` rounded to an integer multiple of the physics step (a warning if it differs). The adapter samples on the boundaries and returns the previous value with `updated = false` in between |
| JNT-ADP-040 | Each characteristic has its own random stream (`offset`, `accuracy`, `noise`, `torque_unit`, `torque_noise`). Streams are derived from the seed, the vehicle, the sensor name and the characteristic |
| JNT-ADP-050 | `reset` returns the delay history, the backlash state, the incremental start, the velocity history, the unit errors (the same seed gives the same values), and the random streams to their state at creation |

### 2.2 Every physics step

| ID | Requirement |
|---|---|
| JNT-ADP-110 | Delay: the truth from floor(`delay` / step) steps earlier. While the history is too short, the truth at time 0 |

### 2.3 Angle (every boundary, in this order)

| ID | Requirement |
|---|---|
| JNT-ADP-210 | Deflection: on the motor side with K > 0, θ = q + τ_tr / K (τ_act if there is no transmitted torque). Otherwise θ = q |
| JNT-ADP-220 | Backlash: on the motor side with b > 0, with play state p: if θ − p > b/2 then p = θ − b/2; if θ − p < −b/2 then p = θ + b/2. Initially p = θ. From then on θ = p |
| JNT-ADP-230 | Offset: for an absolute encoder, θ ← θ + o, with o drawn at creation from U(−O, O) |
| JNT-ADP-240 | Shaft angle s = N·θ. Cyclic error: s ← s + A·sin(s + ψ), with A from U(0, `accuracy`) and ψ from U(0, 2π), drawn at creation |
| JNT-ADP-250 | Incremental: s ← s − s₀ (s₀ is the first s sampled after creation or after reset) |
| JNT-ADP-260 | Noise: s ← s + N(0, σ_c·2π/C) |
| JNT-ADP-270 | Count c = round(s·C/2π) (halves round away from 0) |
| JNT-ADP-280 | With `single_turn`, c ← c mod C (0 to C−1) |
| JNT-ADP-290 | Angle output = c·2π/(C·N). The count is also output. Without `encoder`, the angle is q and the count is 0 |

### 2.4 Velocity

| ID | Requirement |
|---|---|
| JNT-ADP-310 | `difference`: v = (c_k − c_{k−w})·2π/(C·N·w·T), where c is the count before wrapping. If the sample w back does not exist yet, the oldest available value and the time since it are used. The first value is 0. `true`: the truth after delay. `difference` requires `encoder` |
| JNT-ADP-320 | When `lsb` > 0, v ← lsb·round(v/lsb) |

### 2.5 Torque

| ID | Requirement |
|---|---|
| JNT-ADP-410 | `sensor`: x = τ_tr (τ_act if there is no transmitted torque). `current`: x = τ_act + F_c·sgn(q̇) + F_v·q̇ (sgn(0) = 0) |
| JNT-ADP-420 | y = x·(1 + g) + b. g (scale error) is drawn at creation from U(−S, S), b (bias) from U(−B, B) |
| JNT-ADP-430 | Noise: y ← y + N(0, σ) |
| JNT-ADP-440 | Saturation: if \|y\| > R, y = ±R and `kSensorFlagSaturated` is set |
| JNT-ADP-450 | Resolution: if q_τ > 0, y ← q_τ·round(y/q_τ). Without `torque`, `has_effort` is false |

### 2.6 Configuration, capabilities, log

| ID | Requirement |
|---|---|
| JNT-ADP-510 | The configuration is JSON. The keys are those of the Configuration section of the [design](../design/joint-sensor-adapter.md) (`type`, `update_rate`, `delay`, `encoder.*`, `velocity.*`, `torque.*`) |
| JNT-ADP-520 | Angles are rad or `"<n>deg"`, `"<n>arcmin"`, `"<n>arcsec"`. Velocities are rad/s or `"<n>rpm"`. Fractions are `"<n>%"` |
| JNT-ADP-540 | Creation fails on: an unknown key (`$schema` is skipped), both or neither of `counts_per_rev` and `bits`, backlash or deflection on the joint side, `difference` without an encoder, an unknown unit, `torque` without `range`, or a gear ratio less than 1 |
| JNT-ADP-550 | The capability table follows the configuration. `deterministic_with_seed` is true when a seed is passed; `thread_safe_across_instances` is true |
| JNT-ADP-610 | Log columns: `true_position`, `true_velocity`, `true_torque`, `encoder_angle`, `backlash_state` |

## 3. Configuration example

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
