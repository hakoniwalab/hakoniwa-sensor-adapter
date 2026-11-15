# Gyroscope specification

This is the specification of the gyroscope behind the IMU adapter (`imu_sensor_adapter.hpp`). The design is in [imu-adapter.md](../design/imu-adapter.md).
It carries over the gyroscope requirements of the Hakoniwa drone sensor specification with the same IDs (`SEN-GYR-*`). Rules shared by all drone sensors (axes, mounting pose, rotation formula, processing order, Gauss-Markov drift, noise density) are in the [common drone sensor specification](drone-common.md) (`SEN-COM-*`).

The **Who** column says which side meets a requirement. In this interface the host gives the truth already rotated to the sensor axes (design, 3.2), so the mounting requirements are the host's. Requirements on the connection to an autopilot are the host's as well (for example Hakoniwa Drone PRO).

## 1. What it measures

The angular velocity of the vehicle, in **sensor axes**, in rad/s.

| | Content |
|---|---|
| Input (truth) | Angular velocity ω_body in body axes (FRD), from the vehicle body's free joint |
| Output | Angular velocity ω_sensor in sensor axes (rad/s) |

## 2. Mounting pose

The gyroscope is part of the IMU, so its mounting pose is the IMU's (common specification 2.2). Only the orientation is used: ω_sensor = Rᵀ · ω_body (R from common specification 2.3). The position is not used: on a rigid body the angular velocity is the same everywhere. The position is used by the accelerometer of the same IMU.

Example: with the IMU mounted at `yaw = 90`, a roll of the vehicle (about its forward axis) appears as a rotation about the sensor's −y axis.

## 3. Processing order

```text
ω_body → orientation (Rᵀ) → scale and cross-axis (M) → (IMU filter: optional) → delay → sample → zero-rate offset → noise → drift → resolution → saturation → ω_sensor
```

| Stage | Setting (`components.sensors.gyro.*`) | Content |
|---|---|---|
| Orientation | IMU mounting pose | Rotate to the sensor axes |
| Scale and cross-axis | `sensitivity` | Proportional error and leakage between axes (3.3) |
| IMU filter | `filter` | Low-pass filter applied every physics step, only when enabled explicitly (common specification 4.5) |
| Delay | `delayMsec` | Hardware delay (common specification 4) |
| Sample period | `updatePeriodMsec` | Period of new values (common specification 4). `sampleCount` (averaging) accepts only 1 (common specification 6) |
| Zero-rate offset | `zeroRateOffset` | Constant offset present at rest (3.4) |
| Noise | `noise` | Standard deviation of a normal distribution (rad/s), one value for all axes or [x, y, z] per axis |
| Noise density | `noiseDensity` | White-noise density (rad/s/√Hz) instead of `noise`; conversion in common specification 4.3. Not together with `noise` (`SEN-COM-185`) |
| Drift | `drift` | Slow change of the zero point (3.1) |
| Resolution | `resolutionDegPerSec` | Step of the value (3.5) |
| Saturation | `range.fullScaleDegPerSec` | Measurable range (±deg/s), clipped per axis (3.2) |

The delay between the autopilot receiving a value and using it (`softDelayMsec`) is a setting of the hand-over, not of the sensor value (common specification 4).

### 3.1 Drift

| `drift.mode` | Settings | Behaviour |
|---|---|---|
| `ardupilot-sitl` | `speedDegPerSecPerMin`, `timeMin` | The ArduPilot SITL triangle wave; the same amount on all three axes |
| `random-walk` | `randomWalkDegPerSecSqrtSec`, `seed` | A random walk per axis, reproducible with `seed`; unbounded |
| `gauss-markov` | `stddevDegPerSec`, `correlationTimeSec` | Common specification 4.2, per axis. The variance stays constant over time, close to a real sensor. Typical: σ 0.0005–0.005 deg/s (about 2–18 deg/h, the datasheet bias instability), τ hundreds to thousands of seconds |

The drift is added in sensor axes, after the rotation; for a tilted mounting it appears in another direction in body axes.

### 3.2 Saturation

Each axis is clipped to ±`fullScaleDegPerSec`, last (after the resolution), per sensor axis: a tilted gyroscope can saturate on one axis while no body axis exceeds the range. Saturation is not reported upward; the value stays at the end of the range.

### 3.3 Scale error and cross-axis sensitivity

One 3×3 matrix M represents both, applied after the rotation:

```text
ω' = M · ω
M[i][i] = 1 + scaleErrorPct[i] / 100
M[i][j] = crossAxisPct[i][j] / 100   (i ≠ j)
```

```json
"sensitivity": {
  "scaleErrorPct": [0.5, -0.3, 1.0],
  "crossAxisPct": [[0, 1.0, 0], [0, 0, -0.5], [0.2, 0, 0]]
}
```

`scaleErrorPct` is the scale error per axis (%; 1.0 measures 100 deg/s as 101 deg/s; typical ±0.5–2 %). `crossAxisPct[i][j]` is the fraction of the rotation about axis j leaking into axis i (%; the diagonal is unused and written as 0; typical ±1–2 %).

### 3.4 Zero-rate offset

```json
"zeroRateOffset": { "fixedDegPerSec": [0.5, -1.0, 0.2] }
```
```json
"zeroRateOffset": { "randomStdDegPerSec": 1.0 }
```

`fixedDegPerSec` is a fixed offset per axis (deg/s). `randomStdDegPerSec` draws one offset per axis from a normal distribution once at start-up (reproducible with `simulation.randomSeed`). Typical ±0.5–3 deg/s. An offset that moves over time is drift (3.1).

### 3.5 Resolution

Each axis is rounded to the nearest integer multiple of `resolutionDegPerSec`, before saturation. Example: a 16-bit ±2000 deg/s part has 4000 / 65536 ≈ 0.061 deg/s.

## 4. Hand-off to the autopilot

The value is handed to the autopilot in sensor axes. The autopilot side converts it back to body axes with the IMU mounting-orientation setting.

| Path | Hand-off | Conversion to body axes |
|---|---|---|
| Control Link (PX4) | The host adapter passes the value to the EKF and control | The host adapter applies `SENS_BOARD_ROT` |
| Control Link (ArduPilot) | The host adapter passes the value to EKF3 and control | The host adapter applies `AHRS_ORIENTATION` |
| PX4 SITL | `HIL_SENSOR` | PX4 applies `SENS_BOARD_ROT` |
| ArduPilot SITL | JSON | ArduPilot applies `AHRS_ORIENTATION` |
| Hakoniwa adapter | — | Not supported |

## 5. Mapping to PX4 and ArduPilot parameters

The source distinguishes four relations: matching the installation, estimator assumptions, replacement by SITL, and calibration.

| Hakoniwa setting | Relation | PX4 | ArduPilot |
|---|---|---|---|
| IMU orientation | Match | `SENS_BOARD_ROT`, `SENS_BOARD_X_OFF`, `Y_OFF`, `Z_OFF` | `AHRS_ORIENTATION`, `CUST_ROT_ENABLE`, `CUST_ROT1_ROLL`, `PITCH`, `YAW` |
| IMU orientation | No mapping | — | `SIM_IMU_ORIENT = 0`; Hakoniwa sends values already rotated into sensor axes |
| `noise` | Estimator assumption | `EKF2_GYR_NOISE` | `EK3_GYRO_P_NSE` |
| `noise` | No mapping | — | `SIM_GYR1_RND = 0`; Hakoniwa adds the noise |
| `drift` | Estimator assumption | `EKF2_GYR_B_NOISE` | `EK3_GBIAS_P_NSE` |
| `drift` | Replacement | — | `SIM_GYR1_DRIFT`, `SIM_GYR1_DRIFT_T`, or no direct replacement, according to the drift mode |
| `sensitivity`, `zeroRateOffset` | Calibration | No automatic calibration mapping | No automatic calibration mapping |
| `range`, `resolutionDegPerSec` | — | No mapping | No mapping |
| `updatePeriodMsec` | Related rate | `IMU_INTEG_RATE` | `SCHED_LOOP_RATE` |

For ArduPilot SITL, each replaced characteristic is converted to the table's parameter and is not also added by Hakoniwa. Characteristics without a replacement remain on the Hakoniwa side. `SIM_IMU_ORIENT` and `SIM_GYR1_RND` are 0.

## 6. Acceptance checks

1. With no mounting pose or new settings, the output matches the path without those stages.
2. A 90° mounting rotation produces the expected sensor-axis value.
3. Setting the same orientation in the autopilot gives the same flight behavior as an unrotated mounting; omitting it does not.
4. Saturation is applied independently in sensor axes for a tilted sensor.
5. A fixed seed reproduces the same noise, start-up offset, and drift sequences.

## 7. Unsupported characteristics

| Characteristic | Meaning | Typical | Note |
|---|---|---|---|
| Temperature | Zero point and scale change with temperature | about 0.01 deg/s/°C | Requires a vehicle temperature model |
| g-sensitivity | Offset under acceleration | about 0.1 deg/s/g | Could use the accelerometer in the same IMU |
| Internal filter | The chip's low-pass filter | tens to hundreds of Hz | The optional IMU filter approximates it |
| Vibration and flexing | Vehicle vibration entering the gyroscope | vehicle-dependent | Periodic disturbance can use the host's experimental [vibration option](https://github.com/toppers/hakoniwa-drone-core/blob/221afa40809c9edb476f88c71383a43c134d4b9d/docs/architecture/aircraft/sensor/vibration/README.md); flexing is unsupported |
| Saturation report | Reporting that a value clipped | — | For example, a PX4 clip counter |
| Multiple IMUs | Two or more IMUs | — | One IMU is supported |
| Bandwidth and aliasing | Folding caused by internal sampling and output timing | — | Physics-step values are handled by filtering and delay |

| Can judge | Cannot judge |
|---|---|
| Whether the mounting orientation and autopilot setting are correct | Control stability involving unmodelled vehicle vibration or exact chip filtering |
| Effects of noise, offset, drift, scale error, and saturation | Exact value sequences of a real gyroscope |
| Effects of delay on control | Temperature-dependent errors |

## 8. Requirements

| ID | Requirement | Who | Section |
|---|---|---|---|
| SEN-GYR-010 | Input is angular velocity in body axes (FRD); output is angular velocity in sensor axes (rad/s) | Host (input), adapter (output) | 1 |
| SEN-GYR-020 | Use orientation only: ω_sensor = Rᵀ·ω_body | Host | 2 |
| SEN-GYR-030 | Do not use mounting position | Host | 2 |
| SEN-GYR-040 | Processing order is orientation → M → delay → sample → zero-rate offset → noise → drift → resolution → saturation | Host (input), adapter (output) | 3 |
| SEN-GYR-050 | Noise σ (rad/s) is scalar or [x,y,z]; an axis at 0 or less has no noise | Adapter | 3 |
| SEN-GYR-060 | `ardupilot-sitl` drift is a triangle wave with the same amount on all axes | Adapter | 3.1 |
| SEN-GYR-070 | `random-walk` drift is an independent random walk per axis, reproducible with `seed` | Adapter | 3.1 |
| SEN-GYR-080 | `gauss-markov` drift follows common 4.2 with σ `stddevDegPerSec` | Adapter | 3.1 |
| SEN-GYR-090 | Add drift in sensor axes | Adapter | 3.1 |
| SEN-GYR-100 | Saturation is ±`fullScaleDegPerSec`, after resolution and last | Adapter | 3.2 |
| SEN-GYR-110 | Apply saturation per sensor axis | Adapter | 3.2 |
| SEN-GYR-120 | Do not report saturation upward | Adapter | 3.2 |
| SEN-GYR-130 | Scale and cross-axis error use section 3.3 matrix M after rotation | Adapter | 3.3 |
| SEN-GYR-140 | Without `sensitivity`, M is identity | Adapter | 3.3 |
| SEN-GYR-150 | `zeroRateOffset.fixedDegPerSec` is fixed per axis | Adapter | 3.4 |
| SEN-GYR-160 | Draw `zeroRateOffset.randomStdDegPerSec` once at start-up per axis from a normal distribution | Adapter | 3.4 |
| SEN-GYR-170 | Only one of `fixedDegPerSec` and `randomStdDegPerSec`; omission means no offset | Adapter | 3.4 |
| SEN-GYR-180 | Zero-rate offset does not change during flight | Adapter | 3.4 |
| SEN-GYR-190 | Round to the nearest multiple of `resolutionDegPerSec` before saturation | Adapter | 3.5 |
| SEN-GYR-200 | Halfway values round away from zero; the step is common to all axes | Adapter | 3.5 |
| SEN-GYR-210 | Without `resolutionDegPerSec`, do not round | Adapter | 3.5 |
| SEN-GYR-220 | Hand values to the autopilot in sensor axes and convert to body axes at the section 4 path | Host | 4 |
| SEN-GYR-230 | In ArduPilot SITL, `SIM_IMU_ORIENT = 0` | Host | 5 |
| SEN-GYR-240 | In ArduPilot SITL, `SIM_GYR1_RND = 0` | Host | 5 |
| SEN-GYR-250 | Convert replaced ArduPilot SITL characteristics to section 5 parameters and do not also add them in Hakoniwa | Host | 5 |
