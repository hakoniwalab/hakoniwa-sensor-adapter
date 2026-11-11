# Force-torque sensor adapter interface (design)

This document defines the interface for 6-axis force-torque sensors (forces Fx, Fy, Fz and moments Tx, Ty, Tz).
The survey (comparison of simulators and real devices) is [force-torque-survey.md](force-torque-survey.md). The interface follows the same approach as the existing sensor interfaces ([imu-adapter.md](imu-adapter.md), [range-image-adapter.md](range-image-adapter.md)).

This document defines the meaning of the configuration values, and the order and formulas for adding errors. The goal is that, with the same configuration, the same truth, and the same seed, every implementation outputs values with the same meaning.

## 1. Purpose and scope

- This is the interface for a 6-axis force-torque sensor mounted on the wrist of a robot arm or at a tool mount. The ROS type it resembles is `geometry_msgs/WrenchStamped`.
- The host passes the true force, and the adapter returns "the value the sensor measured". The adapter owns the sensor error model.
- The interface does not depend on ROS. Units are N and N·m.
- Joint torque sensors (one value per joint) are not part of this interface. Their errors behave differently, so they are defined in a separate interface.

## 2. Host and adapter responsibilities

```text
Host: physics truth (force transmitted from the parts beyond the sensor) → moved to the sensor axes and reference point
                                                        │
Adapter (every physics step):                           ▼
        overload check (truth) → delay → low-pass
Adapter (at each sample period boundary):
        nonlinearity → calibration matrix (scale, crosstalk) → bias → drift → noise
              → saturation → subtract tare → resolution → output
                                                        │
Host:                                                   ▼
        tool weight compensation (when used) → controller, log, PDU
```

| Owner | Content | Reason |
|---|---|---|
| Host | Physics truth. Transformation to the sensor axes and reference point | To give every implementation the same input. Differences in a comparison then come only from the error models |
| Adapter | The error model in section 4, tare, overload check | This is the sensor error model itself |
| Host | Tool weight and inertia compensation | On real devices, this is done by the controller or the user, not the sensor |

## 3. Decisions (principles)

### 3.1 Meaning of the truth

The host passes the following values every physics step.

| Value | Frame | Unit |
|---|---|---|
| Force | Sensor axes | N |
| Moment (about the sensor origin) | Sensor axes | N·m |
| Sensor orientation (rotation seen from world NWU) | Quaternion | — |
| Gravitational acceleration | World NWU | m/s² |

- **Direction:** The force and moment that the parts beyond the sensor (the tool side) apply to the parts before the sensor (the robot side). When a tool hangs at rest, the force equals the tool weight and points in the direction of gravity.
- **Reference point:** The moment is about the sensor origin.
- **Tool weight and inertia are included.** The value is the sum of the contact force, the tool weight, and the inertial force when the tool accelerates.
- Orientation and gravity are auxiliary values. They are passed for computing tool weight compensation outside the adapter, and for implementations that use the direction of gravity.
- The world is passed in NWU and the sensor in FLU. The host converts from the physics engine conventions (such as MuJoCo's child-to-parent direction) at the interface boundary.

### 3.2 Tare

- When `tare()` is called, the adapter stores the average of the values measured during the preceding `tare_window` (default 0.1 s). It subtracts that average from all later outputs.
  - The average uses the values after saturation and before tare and resolution (the order in section 4).
  - If, after creation or after `reset()`, less than `tare_window` of values has been measured, the adapter averages the values it has. If there are no values, the tare fails and returns false.
  - The adapter stores an average instead of a single value so that the noise of one sample does not remain as a fixed offset.
- Drift does not disappear and keeps progressing. To remove it, call `tare()` again. This is the same as on real devices.
- `clear_tare()` clears the stored value. `reset()` also clears it.

### 3.3 Tool weight compensation is outside the interface

- As on real devices, the sensor values include the tool weight and inertia.
- Compensation from the tool mass and center of gravity is placed outside the sensor, as a function of the host or the controller. It can be switched on and off.
- Removing the tool weight with a tare (a common practice on real devices) works as described in 3.2. However, when the orientation changes, the direction of the weight changes, so an offset appears. This is also the same as on real devices.

### 3.4 Time, random numbers, reset, log

These are the same as in the existing sensor interfaces ([imu-adapter.md](imu-adapter.md) sections 3.4, 3.6, and 3.9).

- The host passes the truth every physics step with the simulation time (ns).
- The sample period T is 1 / `update_rate` rounded to an integer multiple of the physics step. If it does not match exactly, a warning is printed (the same rule as other Hakoniwa sensors). The sensor measures at the sample period boundaries (time 0, T, 2T, ...) and returns the previous value in between.
- Random numbers come from the seed, the vehicle name and index, and the sensor name. A separate stream is created for each characteristic (characteristics: `calibration`, `nonlinearity`, `bias`, `drift`, `noise`). The streams are independent, so turning off one characteristic does not change the sequence of values of the others.
- `reset()` returns the delay history, filter state, drift, the per-unit errors drawn at creation, the tare, and the random streams to their state at creation.
- Log columns (current drift, the value stored by tare, the value right after the filter, and so on) are read with `log_columns()` and `log_values()`.

### 3.5 Value flags

- Saturation: uses the common bit `kSensorFlagSaturated`. It is set when any axis of the value was clipped at the end of the measurable range.
- Overload: adds the force-torque flag `kForceTorqueFlagOverloaded`. It is set when, at any physics step since the previous measurement, any axis of the truth exceeded the overload value. The check uses the truth before delay, filtering, and errors. Overload is about how the device breaks, so it depends on the applied load, not on the measured value.
- Undefined bits are 0.

### 3.6 Instances and threads

- One instance is one force-torque sensor on one vehicle. It is called from only one thread.
- Whether different instances may be called from different threads is declared in the capability table.

## 4. Error model (formulas)

Notation: axis i = Fx, Fy, Fz, Tx, Ty, Tz. R_i is the measurable range of that axis (`range`: the value of `fxy` for Fx and Fy, `fz` for Fz, `txy` for Tx and Ty, `tz` for Tz). x is the truth and y is the measured value. U(−a, a) is the uniform distribution from −a to a, and N(0, σ) is the normal distribution.

### 4.1 Every physics step

| Stage | Formula or rule |
|---|---|
| Overload check | If \|x_i\| > O_i on any axis, set the overload flag on the next measured value. O_i is the overload value |
| Delay | Use the truth from `delay` earlier. Truncated to the physics step (number of delay steps = floor(delay / step)). At the start, when the history is not long enough, use the truth at time 0 |
| Low-pass | 2nd-order Butterworth. Cutoff frequency `lowpass_cutoff_hz`, bilinear transform at the physics step (with prewarping of the cutoff frequency). The initial state is steady at the truth at time 0 |

### 4.2 At each sample period boundary

The low-pass output x̃ is divided by the range per axis to get a dimensionless value u_i = x̃_i / R_i. Then the following are added in order.

| Stage | Formula or rule |
|---|---|
| Nonlinearity | u_i ← u_i + n_i · 4 · u_i · (1 − \|u_i\|). n_i is drawn at creation from U(−NL_i, NL_i) (NL_i is the nonlinearity in %FS ÷ 100). Not added when \|u_i\| > 1. This is an end-point based nonlinearity: the deviation is 0 at 0 and at ±full range, and maximum NL_i at half range |
| Calibration matrix | u ← C · u. C = I + E. Diagonal E_ii from U(−S, S) (S is the scale error in %FS ÷ 100), other E_ij from U(−X, X) (X is the crosstalk in % ÷ 100). Drawn once at creation. When a full-range force acts on axis j, a value of up to X·R_i appears on axis i |
| Back to physical units | y_i = u_i · R_i |
| Bias | y_i ← y_i + b_i. b_i is drawn at creation from U(−B_i, B_i) |
| Drift | y_i ← y_i + d_i. At each measurement, d_i ← d_i + N(0, D_i · √(T / 3600 s)). D_i is the drift per hour (a random walk whose σ reaches D_i in 1 hour). Initially 0 |
| Noise | y_i ← y_i + N(0, σ_i). σ_i is converted as in 4.3 |
| Saturation | If \|y_i\| > R_i, clip to ±R_i and set the saturation flag |
| Tare | y_i ← y_i − t_i. t_i is the value stored in 3.2 (0 if none) |
| Resolution | y_i ← q_i · round(y_i / q_i). q_i is the resolution. If 0, no quantization |

A stage is not used if it is not written in the configuration (the same as a value of 0). If nothing is written, the output has no error.

### 4.3 Noise conversion

The datasheet noise is read as "the σ of the values output in 1 second at rest, at that sensor's output period and filter".

1. Let the datasheet value be σ₀. For `1sigma`, use it as is. For `6sigma_pp` (6σ peak-to-peak width), divide by 6.
2. Determine the datasheet noise bandwidth B₀. If `bandwidth_hz` is given, use that value. Otherwise use `rate_hz` ÷ 2 (the Nyquist frequency of that output period).
3. Determine this adapter's noise bandwidth B. With a low-pass, min(`lowpass_cutoff_hz`, `update_rate` ÷ 2). Without, `update_rate` ÷ 2.
4. As white noise, σ = σ₀ · √(B / B₀).

Noise is added after the low-pass, at each sample period boundary.

## 5. Configuration

The adapter reads only configuration files in its own format. The Hakoniwa adapter configuration is written as follows.

- Per-axis values are written per axis group (`fxy`, `fz`, `txy`, `tz`).
- A value is a number or a string with a unit suffix. `"5%"` is a ratio to the range R_i of that axis (%FS). `"500%"` for range and overload is also a ratio to the range.

| Key | Meaning | Unit or form | When omitted |
|---|---|---|---|
| `type` | `"force_torque"` | — | Required |
| `update_rate` | Sample period | Hz | Required |
| `range` | Measurable range R | N, N·m (axis groups) | Required |
| `overload` | Overload value O | Number or % of range | No check |
| `delay` | Delay | s | 0 |
| `lowpass_cutoff_hz` | Low-pass cutoff frequency | Hz | Not used |
| `nonlinearity` | Nonlinearity | %FS (axis groups or a single value) | 0 |
| `scale_error` | Scale error | %FS (a single value) | 0 |
| `crosstalk` | Crosstalk | % (a single value) | 0 |
| `bias` | Upper limit of the bias | Number or % of range | 0 |
| `drift_per_hour` | Drift per hour | Number or % of range | 0 |
| `noise` | Noise (`definition`, `rate_hz`, `bandwidth_hz`, values per axis group) | 4.3 | 0 |
| `resolution` | Resolution | Number | 0 |
| `tare_window` | Averaging time for tare | s | 0.1 |

- When the datasheet has only a single "accuracy" value, write that value in `scale_error`. Write crosstalk only when the datasheet gives a value.
- Invalid configurations (unknown keys, missing required keys, negative values, a range of 0 or less, unknown unit suffixes, a low-pass at or above half the physics step frequency) cause an error at creation.

Example (values from the OnRobot HEX-E QC datasheet):

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

When the datasheet does not state the output period at which the noise was measured, write the default output period of that model in `rate_hz` (the 100 Hz in the example above is a provisional value).

## 6. Interface shape

| Name | Content |
|---|---|
| `ForceTorqueAdapterSetup` | What the host passes at creation (the common `SensorAdapterSetup`) |
| `ForceTorqueTruth` | Truth every physics step: time, force, moment, sensor orientation, world gravity |
| `ForceTorqueSample` | Output of one step: whether a new value was measured, measurement time, force, moment, flags |
| `ForceTorqueAdapterCapabilities` | Declaration of the supported features |
| `IForceTorqueSensorAdapter` | `step`, `tare`, `clear_tare`, `capabilities`. `reset`, `log_columns`, and `log_values` are inherited from the common `ISensorAdapter` |

The header is placed at `include/hakoniwa/sensor_adapter/force_torque_sensor_adapter.hpp`. How to carry it over the external sensor plugin (the C boundary) is defined separately, because it differs from range and image sensors in that the host passes the truth.

## 7. Capability table

Features the adapter does not have are set to `false`. An adapter does not pretend to have them.

| Item | Meaning |
|---|---|
| `delay`, `lowpass` | Delay, low-pass |
| `nonlinearity` | Nonlinearity |
| `calibration_matrix` | Scale error and crosstalk (6×6) |
| `bias`, `drift`, `noise`, `resolution` | Bias, drift, noise, resolution |
| `range_clamp` | Saturation (sets the flag) |
| `overload` | Overload flag |
| `overload_offset` | Bias that remains after an overload |
| `temperature` | Change with temperature (the host passes the temperature) |
| `hysteresis` | Hysteresis |
| `tare` | Tare |
| `deterministic_with_seed` | Same values with the same seed, the same input, and the same call order |
| `thread_safe_across_instances` | Different instances can be called at the same time from different threads |

The formulas for `overload_offset`, `temperature`, and `hysteresis` are not defined here. An implementation that has them writes the formulas in its own documentation.

Guide for choosing:

| Level | Capabilities |
|---|---|
| No error | Nothing. Returns the physics truth as is |
| Basic | Noise, bias, drift, resolution (the same scope as the Gazebo error model) |
| Close to real devices | Basic, plus nonlinearity, calibration matrix, delay and low-pass, saturation and overload, tare |

## 8. What the host does

- Get the truth from the physics engine and move it to the sensor axes and reference point. For MuJoCo, place a body for the sensor (connected without a joint) between the wrist and the tool, and use the force that this body transmits to its parent.
- Convert the physics engine direction convention to the direction in 3.1.
- When tool weight compensation is used, compute it outside the sensor from the tool mass and center of gravity, and subtract it.
- Make tare callable from the user or the controller (for example as a service).
- Put the output on a Hakoniwa PDU (a type corresponding to `geometry_msgs/WrenchStamped`).
