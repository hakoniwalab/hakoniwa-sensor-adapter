# Force-torque sensor: evaluation results (2026-11-11)

We compared 6-axis force-torque sensors on the same six axes as [Evaluation axes](../drone/evaluation-axes.md). Both are wrapped as adapters of the [force-torque sensor interface](../../design/force-torque-adapter.md) and receive the same truth sequence.

| Implementation | Version |
|---|---|
| Hakoniwa | Hakoniwa force-torque sensor model (an implementation of the equations in this interface's design; the source is not public) |
| Gazebo | gz-sensors 10.2.0 `ForceTorqueSensor` (the hakoniwa-sensor-adapter-gazebo adapter) |

Project AirSim has no force-torque sensor, so it is not compared. The force-torque sensors of physics engines (MuJoCo `force` and `torque`, and so on) and the ROS 2 component (mujoco_ros2_control) are listed in axis 2 for feature coverage only.

Date measured: 2026-11-11. Machine: macOS (arm64). Physics step 1 ms, sample period 500 Hz.

## Summary

| Axis | Hakoniwa | Gazebo |
|---|---|---|
| 0. No-noise agreement | Reference | Agrees (difference at most 1.8e-15; sample times agree in all 5,000 samples) |
| 1. Accuracy | Noise, quantization, bias and drift as configured | Noise and quantization as configured. Bias takes only the two values ± the setting. Steady-state σ of the slowly varying bias is √(τ/2) times the setting |
| 2. Coverage (15 items) | 15 | 4 |
| 3. Repeatability (5 cases) | As specified in all 5 cases | Sequence changes with other sensors, two units with the same seed, and reset |
| 4. Speed (mean per physics step) | 0.05 µs (no errors), 0.10 µs (with noise) | 0.11 µs, 0.22 µs |
| 5. Adoption effort | No extra dependencies (only a JSON header to read the configuration) | gz-sensors and its dependencies (same as for the drone sensors) |

## 0. No-noise agreement

With every error turned off, we passed 10 seconds of truth with sinusoids of different amplitude and frequency added to each of the 6 axes.

| Axis | Max difference, Hakoniwa vs Gazebo | Max difference, Hakoniwa vs truth |
|---|---|---|
| Fx | 4.4e-16 | 4.4e-16 |
| Fy | 4.4e-16 | 4.4e-16 |
| Fz | 1.8e-15 | 1.8e-15 |
| Tx | 2.8e-17 | 2.8e-17 |
| Ty | 2.2e-16 | 2.2e-16 |
| Tz | 1.4e-17 | 1.4e-17 |

- This agreement confirms that the force direction (the force the tool side exerts on the robot side) and the moment axes are aligned. The Gazebo adapter outputs gz-sensors' "force the parent exerts on the child" reversed with the default `child_to_parent`, so it has the same direction as the interface.
- The differences are of the size of floating-point rounding.

## 1. Accuracy

We configured both to have the same meaning and took statistics with stationary truth (0).

### Noise (20,000 samples)

| Axis | Configured σ | Hakoniwa | Gazebo |
|---|---|---|---|
| Fx | 0.1 N | 0.0997 | 0.0991 |
| Fy | 0.1 N | 0.1003 | 0.1003 |
| Fz | 0.2 N | 0.2001 | 0.2008 |
| Tx | 0.01 N·m | 0.01003 | 0.01009 |
| Ty | 0.01 N·m | 0.00998 | 0.01001 |
| Tz | 0.02 N·m | 0.02002 | 0.01991 |

For Hakoniwa, we wrote the datasheet 1σ at its output period (`rate_hz` 500), a setting where the bandwidth conversion factor is 1, and compared.

### Quantization (10,000 samples)

The number of values that were not multiples of the step 0.05 N or 0.005 N·m was 0 on all 6 axes, for both Hakoniwa and Gazebo.

### Bias (40 units with seeds 1 to 40, Fx, setting 0.5 N)

| Implementation | Min | Max | Negative units | Distinct values |
|---|---|---|---|---|
| Hakoniwa | −0.442 | 0.449 | 20 | 40 |
| Gazebo | −0.5 | 0.5 | 21 | 2 |

- Hakoniwa draws one value per unit uniformly from the range ±0.5 N (reading the datasheet value as "within").
- Gazebo only attaches a sign to the configured value with 50 % probability (when `bias_stddev` is 0). The spread between units is written separately with `bias_stddev`.

### Drift (each against its own model)

| Implementation | Model | Setting | Measured | Expected |
|---|---|---|---|---|
| Hakoniwa | Random walk | 60 N per hour | σ of one increment 0.0446 N | 60 × √(0.002 / 3600) = 0.0447 N |
| Gazebo | First-order Gauss-Markov | `dynamic_bias_stddev` 1 N, `dynamic_bias_correlation_time` 10 s | Steady-state σ over 200 units 2.226 N | σ_b·√(τ/2) = 2.236 N by the gz-sensors equation |

Gazebo's `dynamic_bias_stddev` is not the steady-state σ. From the gz-sensors discretization (`sigma_b_d = sqrt(−σ_b²·τ/2·expm1(−2·dt/τ))`), the steady-state σ is σ_b·√(τ/2). To get a steady-state σ of σ, write σ / √(τ/2).

## 2. Coverage

| # | Item | Hakoniwa | Gazebo | MuJoCo (`force`, `torque`) | mujoco_ros2_control |
|---|---|---|---|---|---|
| 1 | Direction and moment reference point are defined | ○ (force the tool exerts on the robot, about the sensor origin) | △ (the SDFormat description and the gz-sim computation differ) | ○ (child to parent, about the site origin) | △ (reverses the MuJoCo value; not documented) |
| 2 | Per-axis noise | ○ | ○ | × (`noise` can be written but is not added) | ○ (σ only) |
| 3 | Datasheet noise definition and bandwidth conversion | ○ | × | × | × |
| 4 | Bias (per unit) | ○ | △ (two values ±; spread written separately) | × | × |
| 5 | Drift | ○ (random walk) | ○ (Gauss-Markov) | × | × |
| 6 | Resolution | ○ | ○ | × | × |
| 7 | Nonlinearity | ○ | × | × | × |
| 8 | Scale error, crosstalk (6×6) | ○ | × | × | × |
| 9 | Delay | ○ | × | ○ (`delay`) | × |
| 10 | Low-pass | ○ | × | △ (history interpolation) | × |
| 11 | Saturation and flag | ○ | × | △ (clipped by `cutoff`; no flag) | △ (same as MuJoCo) |
| 12 | Overload flag | ○ | × | × | × |
| 13 | Tare | ○ | × | × | × |
| 14 | Per-sensor random stream determined by the seed | ○ | × (one per process) | — | × (seed cannot be specified) |
| 15 | Usable independently of the physics truth (also with other physics engines) | ○ | ○ | × | × |

The MuJoCo and mujoco_ros2_control columns were read from documentation and source (for mujoco_ros2_control, `hardware_interface.rst` and `MujocoSystemInterface` on main). They were not run and measured.

## 3. Repeatability

Noise setting, seed 5, 2,000 samples.

| Case | Hakoniwa | Gazebo |
|---|---|---|
| Recreated with the same seed gives the same sequence | Yes | Yes |
| Same sequence even when another sensor runs in between | Yes | No |
| Two units with the same seed run alternately give the same sequence | Yes | No |
| After reset, the same sequence as right after creation | Yes | No |
| Different seed gives a different sequence | Yes | Yes |

Gazebo has one random stream per process, so the sequence changes with the activity of other sensors. The seed is not set again at reset so that the sequences of other sensors are not restarted too (an adapter rule).

## 4. Speed

Median of 5 runs of 60 seconds (60,000 physics steps).

| Setting | Hakoniwa | Gazebo |
|---|---|---|
| No errors | 0.048 µs | 0.107 µs |
| With noise | 0.096 µs | 0.220 µs |

Both are below 0.03 % of the 1 ms physics step.

## 5. Adoption effort

| Item | Hakoniwa | Gazebo |
|---|---|---|
| Dependencies | nlohmann/json to read the configuration (header only) | gz-sensors and its dependencies (gz-math, gz-msgs, gz-transport, sdformat, protobuf, and others; same as for the [drone sensors](../drone/axis5-adoption.md)) |
| Configuration | JSON. Datasheet values are copied with units | SDFormat `<force_torque>` |
| Transport | None | A gz-transport publisher is created (a thread starts) |

## Method

- The comparison program calls the two adapters only through this interface.
- The Gazebo adapter is public, so anyone can reproduce its column.
- The comparison program and the Hakoniwa adapter are not public.
