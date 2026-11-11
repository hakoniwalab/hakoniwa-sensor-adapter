# Force-torque sensor specification

This is the specification of the 6-axis force-torque sensor adapter (forces and moments).
The interface is `force_torque_sensor_adapter.hpp`. The design is in [force-torque-adapter.md](../design/force-torque-adapter.md).
This specification states the formulas of that design as numbered requirements that every adapter must follow.

The IDs are separate from the range and image sensors (`RIS-*`) because the host supplies the truth force directly.

## 1. Requirement IDs and truth

IDs have the form `FTS-ADP-<nnn>` (FTS = Force-Torque Sensor, ADP = adapter: error model, tare, configuration).

The truth (`ForceTorqueTruth`) is defined in the design ("Meaning of the truth"): the force and moment that the parts beyond the sensor (the tool side) exert on the parts before it (the robot side), in sensor axes (FLU), with the moment about the sensor origin. It includes contact forces, tool weight and tool inertia. The host also passes the sensor attitude and the world gravity. How the host computes the truth from its physics engine is outside this specification.

## 2. Adapter

Axes i = Fx, Fy, Fz, Tx, Ty, Tz. R_i is the measurement range of that axis. U(−a, a) is a uniform distribution and N(0, σ) is a normal distribution.

### 2.1 Time, random numbers, reset

| ID | Requirement |
|---|---|
| FTS-ADP-010 | The adapter does not know the physics engine. The host passes the truth as `ForceTorqueTruth`. The adapter depends on no physics-engine API |
| FTS-ADP-020 | `step` times are 0, step, 2·step, …. Any other time is an error. After an error, continuing with correct times gives the same output as if the error had not happened |
| FTS-ADP-030 | The sample period T is 1 / `update_rate` rounded to an integer multiple of the physics step (a warning is printed if it differs; same as the other sensors). The adapter samples at times 0, T, 2T, … and returns the previous value with `updated = false` in between |
| FTS-ADP-040 | Each characteristic has its own random stream (`calibration`, `nonlinearity`, `bias`, `drift`, `noise`). Streams are derived from the seed, the vehicle name and index, the sensor name and the characteristic (the common derivation). Turning off one characteristic does not change the value sequence of the others |
| FTS-ADP-050 | `reset` returns the delay history, the filter, the drift, the unit errors drawn at creation (the same seed draws the same values again), the tare, and the random streams to their state at creation. The same inputs then give the same outputs as right after creation |
| FTS-ADP-060 | When a seed is passed, the same seed, inputs and call order give the same sequence of values |

### 2.2 Every physics step

| ID | Requirement |
|---|---|
| FTS-ADP-110 | Overload: if \|x_i\| > O_i on any axis (truth, before delay, filter and errors), `kForceTorqueFlagOverloaded` is set on the next sampled value |
| FTS-ADP-120 | Delay: the truth from floor(`delay` / step) steps earlier is used. While the history is too short, the truth at time 0 is used |
| FTS-ADP-130 | Low-pass: second-order Butterworth (Q = 1/√2). With cutoff fc and step h, a bilinear transform with pre-warping (K = tan(π·fc·h)). The initial state is steady at the truth at time 0 (output equals input). If fc is 1/(2h) or more, creation fails |

### 2.3 Every sample-period boundary

The low-pass output x̃ is normalised to u_i = x̃_i / R_i, then the following are applied in this order.

| ID | Requirement |
|---|---|
| FTS-ADP-210 | Nonlinearity: when \|u_i\| ≤ 1, u_i ← u_i + n_i·4·u_i·(1 − \|u_i\|). n_i is drawn at creation from U(−NL_i, NL_i) |
| FTS-ADP-220 | Calibration matrix: u ← C·u, C = I + E. E_ii is from U(−S, S) (scale error); E_ij (i ≠ j) is from U(−X, X) (crosstalk). Drawn at creation in row order (Fx to Tz), then column order |
| FTS-ADP-230 | Back to physical units: y_i = u_i·R_i |
| FTS-ADP-240 | Bias: y_i ← y_i + b_i. b_i is drawn at creation from U(−B_i, B_i) |
| FTS-ADP-250 | Drift: on every sample, d_i ← d_i + N(0, D_i·√(T / 3600 s)), y_i ← y_i + d_i. Initially 0 |
| FTS-ADP-260 | Noise: y_i ← y_i + N(0, σ_i). σ_i is from section 2.4 |
| FTS-ADP-270 | Saturation: if \|y_i\| > R_i, it is clipped to ±R_i and `kForceTorqueFlagSaturated` is set |
| FTS-ADP-280 | Tare: y_i ← y_i − t_i (t from section 2.5) |
| FTS-ADP-290 | Resolution: if q_i > 0, y_i ← q_i·round(y_i / q_i) (halves round away from 0) |
| FTS-ADP-300 | Stages not written in the configuration are not used. If no stage is written, the output is the truth (without delay and filter, exactly the truth on the sampling step) |

### 2.4 Noise conversion

| ID | Requirement |
|---|---|
| FTS-ADP-310 | σ₀: used as is when `definition` is `1sigma`; divided by 6 when it is `6sigma_pp` |
| FTS-ADP-320 | B₀ = `bandwidth_hz`. If absent, `rate_hz` / 2. If neither is given, creation fails |
| FTS-ADP-330 | B = min(fc, `update_rate` / 2) with a low-pass, otherwise `update_rate` / 2 |
| FTS-ADP-340 | σ = σ₀·√(B / B₀) |

### 2.5 Tare

| ID | Requirement |
|---|---|
| FTS-ADP-410 | `tare()` stores as t the mean of the values sampled during the last `tare_window` (default 0.1 s) (after saturation, before tare and resolution). The window holds the values whose time is in (now − `tare_window`, now] |
| FTS-ADP-420 | If no value has been sampled since creation or since `reset`, `tare()` returns false and does not change t |
| FTS-ADP-430 | `clear_tare()` sets t = 0. `reset` also sets it to 0 |
| FTS-ADP-440 | Tare does not stop the drift |

### 2.6 Configuration

| ID | Requirement |
|---|---|
| FTS-ADP-510 | The configuration is JSON. The keys are those of the Configuration section of the [design](../design/force-torque-adapter.md) (`type`, `update_rate`, `range`, `overload`, `delay`, `lowpass_cutoff_hz`, `nonlinearity`, `scale_error`, `crosstalk`, `bias`, `drift_per_hour`, `noise`, `resolution`, `tare_window`) |
| FTS-ADP-520 | Values per axis group (`fxy`, `fz`, `txy`, `tz`) are a number or `"<number>%"`. % is a fraction of that axis's range R_i. `nonlinearity` may also be a single value (the same value for all four groups) |
| FTS-ADP-530 | `scale_error` and `crosstalk` are `"<number>%"` or a fraction (0.05 is 5%) |
| FTS-ADP-540 | Creation fails on: an unknown key (`$schema` is skipped), a missing required key (`type`, `update_rate`, and `range` with all four axis groups), a negative value, a range of 0 or less, an unknown `definition`, or an unreadable % notation |
| FTS-ADP-550 | Capability table: the capabilities of the stages written in the configuration are true. `tare` and `deterministic_with_seed` (when a seed is passed) are true; `overload_offset`, `temperature` and `hysteresis` are false. `thread_safe_across_instances` is true (instances share no state) |

### 2.7 Log

| ID | Requirement |
|---|---|
| FTS-ADP-610 | Log columns: `true_fx` … `true_tz` (the value after delay and filter at the last sample), `drift_fx` … `drift_tz`, `tare_fx` … `tare_tz`. Units are N and N·m |

## 3. Configuration example

```json
{
  "type": "force_torque",
  "update_rate": 500,
  "lowpass_cutoff_hz": 100,
  "range":    { "fxy": 200, "fz": 200, "txy": 10, "tz": 6.5 },
  "overload": { "fxy": "500%", "fz": "500%", "txy": "500%", "tz": "500%" },
  "noise": {
    "definition": "1sigma",
    "rate_hz": 100,
    "fxy": 0.035, "fz": 0.15, "txy": 0.002, "tz": 0.001
  },
  "resolution": { "fxy": 0.2, "fz": 0.8, "txy": 0.01, "tz": 0.002 },
  "nonlinearity": "2%",
  "crosstalk": "5%"
}
```
