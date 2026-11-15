# Accelerometer specification

This is the specification of the accelerometer behind the IMU adapter (`imu_sensor_adapter.hpp`). The design is in [imu-adapter.md](../design/imu-adapter.md).
It carries over the accelerometer requirements of the Hakoniwa drone sensor specification with the same IDs (`SEN-ACC-*`). Common rules are in [drone-common.md](drone-common.md).

## 1. What it measures

The accelerometer measures specific force at the sensor location, in sensor axes, in m/s².

```text
f = a − g
```

Here `a` is translational acceleration at the sensor and `g` is (0, 0, +g₀) in world NED, where g₀ = 9.80665 m/s². At rest and level, the FRD value is (0, 0, −9.80665) m/s². In free fall it is zero. Physics, control, and sensors use the same g₀. After loading, Earth-like MuJoCo gravity from 9.7 through 9.9 m/s² downward, including the 9.81 default, is set to (0, 0, −g₀) in MuJoCo axes. Other values are unchanged.

## 2. Mounting pose

Both the IMU position and orientation are used. The host computes:

```text
f_mount  = f_origin + α × r + ω × (ω × r)
f_sensor = Rᵀ · f_mount
```

The terms are the tangential and centripetal acceleration at the mounting point. Gravity does not vary across the vehicle.

## 3. Processing order

```text
f_origin → position → orientation → M → optional IMU filter → delay → sample → zero-g offset → noise → drift → resolution → saturation
```

| Stage | Setting (`components.sensors.acc.*`) | Content |
|---|---|---|
| Position and orientation | IMU mounting pose | Move to the sensor location and rotate into sensor axes |
| Scale and cross-axis | `sensitivity` | Proportional error and leakage between axes |
| IMU filter | `filter` | Optional physics-step low-pass filter; see common specification 4.5 |
| Delay | `delayMsec` | Hardware delay |
| Sample period | `updatePeriodMsec` | New-value period; `sampleCount` accepts only 1 |
| Zero-g offset | `zeroGOffset` | Constant output when specific force is zero |
| Noise | `noise` | Normal-distribution σ in m/s², scalar or [x, y, z] |
| Noise density | `noiseDensity` | White-noise density in m/s²/√Hz instead of `noise` |
| Drift | `drift` | `gauss-markov`, with `stddevMg` and `correlationTimeSec` |
| Resolution | `resolutionMps2` | Output step |
| Saturation | `range.fullScaleG` | Per-axis range in ±g |

Accelerometer and gyroscope `updatePeriodMsec` values must be equal and either both present or both absent. Unit conversion uses 1 g = 9.80665 m/s². Rotor vibration, when enabled, is added after the rigid-body position terms and before rotation and sensitivity; see the upstream [vibration specification](https://github.com/toppers/hakoniwa-drone-core/blob/221afa40809c9edb476f88c71383a43c134d4b9d/docs/architecture/aircraft/sensor/vibration/README.md).

### 3.1 Scale error and cross-axis sensitivity

```text
f' = M · f
M[i][i] = 1 + scaleErrorPct[i] / 100
M[i][j] = crossAxisPct[i][j] / 100   (i ≠ j)
```

Without `sensitivity`, M is the identity. `crossAxisPct[i][j]` is leakage from axis j into axis i; diagonal entries are unused.

### 3.2 Zero-g offset

```json
"zeroGOffset": { "fixedMg": [20.0, -15.0, 40.0] }
```
```json
"zeroGOffset": { "randomStdMg": 30.0 }
```

`fixedMg` is a fixed per-axis offset. `randomStdMg` draws one per-axis normal value at start-up. They are mutually exclusive. The offset does not change during flight.

### 3.3 Resolution

Each axis is rounded to the nearest multiple of `resolutionMps2` before saturation. Halfway values round away from zero. Omission means no rounding.

### 3.4 Saturation

Each sensor axis is clipped last to ±`fullScaleG` × 9.80665 m/s². Clipping is not reported upward. Omitting `range` disables clipping.

## 4. Hand-off to the autopilot

The host hands values over in sensor axes and applies the same IMU orientation setting as the gyroscope. PX4 uses `SENS_BOARD_ROT`; ArduPilot uses `AHRS_ORIENTATION`. Hakoniwa preserves the position acceleration and does not subtract it before hand-off.

### 4.1 Position acceleration remains after matching the IMU position

PX4 EKF2 and ArduPilot EKF3 use the configured IMU position to relate sensors and move estimated position and velocity to the center of gravity. They do not remove α×r or ω×(ω×r) from the accelerometer value. `SIM_IMU_POS_X/Y/Z` is therefore 0 in ArduPilot SITL to avoid adding the same effect twice.

## 5. Mapping to PX4 and ArduPilot parameters

| Hakoniwa setting | Relation | PX4 | ArduPilot |
|---|---|---|---|
| IMU orientation | Match | `SENS_BOARD_ROT`, `SENS_BOARD_*_OFF` | `AHRS_ORIENTATION`, custom rotation; `SIM_IMU_ORIENT = 0` |
| IMU position | Match | `EKF2_IMU_POS_*` | `INS_POS1_*`; `SIM_IMU_POS_* = 0` |
| `noise` | Assumption | `EKF2_ACC_NOISE` | `EK3_ACC_P_NSE`; `SIM_ACC1_RND = 0` |
| `zeroGOffset` | Calibration/replacement | `CAL_ACC0_*OFF` | `INS_ACCOFFS_*`; `SIM_ACC1_BIAS_*` |
| `sensitivity` scale | Calibration/replacement | `CAL_ACC0_*SCALE` | `INS_ACCSCAL_*`; `SIM_ACC1_SCAL_*` |
| `drift` | Assumption | `EKF2_ACC_B_NOISE` | `EK3_ABIAS_P_NSE` |

### 5.1 Calibration values

```text
k_i = 1 + scaleErrorPct[i] / 100
offset = zeroGOffset × 0.001 × 9.80665   (m/s²)
scale = 1 / k_i
```

The configuration tool always calculates and records these values. `acc.writeAutopilotCalibration` selects whether they are written, default false. A `randomStdMg` draw is unavailable before start-up, so its calibration offset is 0; the drawn value is logged as `zero_g_offset_*`.

### 5.2 Replacement in ArduPilot SITL

`acc.ardupilotSitlErrorSource` is `hakoniwa` (default) or `ardupilot-sitl`. With `hakoniwa`, `SIM_ACC1_BIAS_*` and `SIM_ACC1_SCAL_*` are 0. With `ardupilot-sitl`, the host removes the two characteristics from the Hakoniwa input configuration and sets bias to the offset and scale to 1/k_i. Nonzero cross-axis terms cannot be reproduced in this mode and are recorded but not added.

## 6. Acceptance checks

The source checks default compatibility, gravity and free fall, both rigid-body position terms, a 90° mounting, matched-position flight behavior, and per-sensor-axis clipping. Detailed upstream test design is outside this translation.

## 7. Unsupported characteristics

Temperature effects, nonlinearity, exact chip filtering, vibration rectification, exact bandwidth and aliasing, clipping reports, and multiple IMUs are unsupported. The model can assess mounting, rigid-body acceleration, and the effects of its configured errors; it cannot reproduce an exact real-sensor sequence.

## 8. Requirements

| ID | Requirement | Who | Section |
|---|---|---|---|
| SEN-ACC-010 | Measure specific force at the sensor location, in sensor axes (m/s²) | Host (input), adapter (output) | 1 |
| SEN-ACC-020 | Specific force is f = a − g. In world NED, g = (0, 0, +g₀), with standard gravity g₀ = 9.80665 m/s² | Host | 1 |
| SEN-ACC-025 | Physics, control, and sensors share g₀. After loading, MuJoCo Earth gravity from 9.7 through 9.9 m/s² downward, including the default 9.81, is set to (0, 0, −g₀). Other values are unchanged | Host | 1 |
| SEN-ACC-030 | Use both mounting position and orientation | Host | 2 |
| SEN-ACC-040 | Specific force at the sensor is f_mount = f_origin + α×r + ω×(ω×r) | Host | 2 |
| SEN-ACC-050 | Orientation is f_sensor = Rᵀ·f_mount | Host | 2 |
| SEN-ACC-060 | Processing order is position → orientation → M → delay → sample → zero-g offset → noise → drift → resolution → saturation | Host (input), adapter (output) | 3 |
| SEN-ACC-070 | Noise σ (m/s²) is a scalar or [x, y, z]; an axis at 0 or less has no noise | Adapter | 3 |
| SEN-ACC-080 | Drift is `gauss-markov` from common specification 4.2, with σ `stddevMg` (mg) and τ `correlationTimeSec` | Adapter | 3 |
| SEN-ACC-085 | The only drift `mode` is `gauss-markov`; any other mode is a configuration error | Adapter | 3 |
| SEN-ACC-090 | Write the same `updatePeriodMsec` for accelerometer and gyroscope | Adapter | 3 |
| SEN-ACC-095 | Different accelerometer and gyroscope `updatePeriodMsec` values, or only one being present, is a configuration error | Adapter | 3 |
| SEN-ACC-100 | Use 1 g = 9.80665 m/s² for unit conversion | Adapter | 3 |
| SEN-ACC-110 | Scale error and cross-axis sensitivity use matrix M from 3.1, applied after rotation | Adapter | 3.1 |
| SEN-ACC-120 | Without `sensitivity`, M is the identity | Adapter | 3.1 |
| SEN-ACC-130 | `zeroGOffset.fixedMg` is a fixed per-axis offset in mg | Adapter | 3.2 |
| SEN-ACC-140 | `zeroGOffset.randomStdMg` is drawn once at start-up per axis from a normal distribution in mg | Adapter | 3.2 |
| SEN-ACC-150 | Only one of `fixedMg` and `randomStdMg` is allowed; omission means no offset | Adapter | 3.2 |
| SEN-ACC-160 | The zero-g offset does not change during flight | Adapter | 3.2 |
| SEN-ACC-170 | Round to the nearest multiple of `resolutionMps2` before saturation | Adapter | 3.3 |
| SEN-ACC-180 | Halfway resolution values round away from zero; the step is common to all axes | Adapter | 3.3 |
| SEN-ACC-190 | Without `resolutionMps2`, do not round | Adapter | 3.3 |
| SEN-ACC-200 | Saturation is ±`fullScaleG` × 9.80665 m/s² and is applied last | Adapter | 3.4 |
| SEN-ACC-210 | Apply saturation per sensor axis | Adapter | 3.4 |
| SEN-ACC-220 | Do not report saturation upward | Adapter | 3.4 |
| SEN-ACC-230 | Without `range`, do not clip | Adapter | 3.4 |
| SEN-ACC-240 | Hand values to the autopilot in sensor axes and convert them to body axes at the table in section 4 with the same IMU orientation setting as the gyroscope | Host | 4 |
| SEN-ACC-250 | Hakoniwa reproduces position acceleration α×r + ω×(ω×r) and does not subtract it before handing the value to the autopilot | Host | 4.1 |
| SEN-ACC-260 | In ArduPilot SITL, `SIM_IMU_POS_X`, `Y`, and `Z` are 0 | Host | 4.1, 5 |
| SEN-ACC-270 | In ArduPilot SITL, `SIM_IMU_ORIENT = 0` | Host | 5 |
| SEN-ACC-280 | In ArduPilot SITL, `SIM_ACC1_RND = 0` | Host | 5 |
| SEN-ACC-290 | In ArduPilot SITL, replacement characteristics (zero-g offset and scale error) are produced either by Hakoniwa or by ArduPilot SITL, never both | Host | 5, 5.2 |
| SEN-ACC-300 | Calibration uses offset = `zeroGOffset` × 0.001 × 9.80665 m/s² and scale = 1/k_i, written to the parameters in 5.1 | Host | 5.1 |
| SEN-ACC-301 | The configuration tool always calculates and records calibration values | Host | 5.1 |
| SEN-ACC-302 | `acc.writeAutopilotCalibration` selects whether calibration is written, default false. The Control Link adapter applies written values with the same formula as the autopilot | Host | 5.1 |
| SEN-ACC-303 | A `randomStdMg` start-up draw is not included in the calibration offset, which is 0. Log the drawn value in accelerometer `zero_g_offset_*` | Host | 5.1 |
| SEN-ACC-305 | The default ArduPilot SITL replacement mode is that Hakoniwa adds and sends the errors | Host | 5.2 |
| SEN-ACC-306 | Select the source with `acc.ardupilotSitlErrorSource` (`hakoniwa` or `ardupilot-sitl`). Other values are errors. It affects only ArduPilot SITL | Host | 5.2 |
| SEN-ACC-310 | When Hakoniwa adds and sends the errors, `SIM_ACC1_BIAS_* = 0` and `SIM_ACC1_SCAL_* = 0` | Host | 5.2 |
| SEN-ACC-320 | When ArduPilot SITL creates the errors, its launch tool removes the offset and scale error from the configuration sent to Hakoniwa, sets `SIM_ACC1_BIAS_*` to the offset, and sets `SIM_ACC1_SCAL_*` to 1/k_i | Host | 5.2 |
| SEN-ACC-325 | In the ArduPilot-SITL-created mode, nonzero `crossAxisPct` cannot be reproduced; record that fact and do not add it on the Hakoniwa side | Host | 5.2 |
