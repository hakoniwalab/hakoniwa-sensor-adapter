# Magnetometer specification

This is the specification of the magnetometer behind `mag_sensor_adapter.hpp`. The design is in [mag-baro-gps-adapter.md](../design/mag-baro-gps-adapter.md).
It carries over the Hakoniwa drone magnetometer requirements with the same IDs (`SEN-MAG-*`). Common rules are in [drone-common.md](drone-common.md).

## 1. What it measures

The output is the magnetic field at the sensor, in sensor axes, in nT. 1 G = 100,000 nT and 1 mG = 100 nT.

### 1.1 Geomagnetic truth

`simulation.location.magneticField` gives intensity F in nT, declination D east of true north in degrees, and inclination I downward from horizontal in degrees. All three values are required and remain constant during flight.

```text
m_ned  = F · (cos I · cos D, cos I · sin D, sin I)
m_body = R_nbᵀ · m_ned
```

The field does not vary across the vehicle.

## 2. Mounting pose

The magnetometer is treated as internal and uses the IMU mounting pose. Only orientation is used: `m_sensor = Rᵀ · m_body`. Position is ignored.

## 3. Processing order

```text
m_body → orientation → soft iron S → hard iron b → delay → sample → noise → drift → resolution → saturation
```

| Stage | Setting (`components.sensors.mag.*`) | Content |
|---|---|---|
| Soft/hard iron | `softIron`, `hardIronNT` | `m' = S·m + b` in sensor axes |
| Delay/sample | `delayMsec`, `updatePeriodMsec` | Hardware delay and new-value period |
| Noise | `noise` or `noiseDensity` | σ in nT or density in nT/√Hz |
| Drift | `drift` | `gauss-markov`, `stddevNT`, `correlationTimeSec` |
| Resolution/range | `resolutionNT`, `range.fullScaleGauss` | Quantization, then per-axis clipping |

### 3.1 Hard iron and soft iron

```text
m' = S · m + b
```

```json
"softIron": { "diagonal": [1.02, 0.98, 1.01], "offDiagonal": [0.01, -0.02, 0.005] },
"hardIronNT": [3000.0, -1500.0, 800.0]
```

S is symmetric. Defaults are diagonal [1,1,1], off-diagonal [0,0,0], and b [0,0,0]. S must be invertible. A determinant magnitude below 1e-9 or a non-finite S or b is an error.

### 3.2 Resolution

Round each axis to the nearest multiple of `resolutionNT` before saturation. Halfway values round away from zero. Omission disables rounding.

### 3.3 Saturation

Clip each sensor axis last to ±`fullScaleGauss` × 100,000 nT. Clipping is not reported upward. Omitting `range` disables clipping.

## 4. Hand-off to the autopilot

Control Link converts sensor axes with `SENS_BOARD_ROT` or `AHRS_ORIENTATION`. PX4 SITL receives gauss in `HIL_SENSOR`. Hakoniwa sends no field to ArduPilot SITL; that SITL creates it and receives replacements for hard iron, soft iron, noise, and delay. Unsupported replacement differences are recorded.

## 5. Mapping to PX4 and ArduPilot parameters

The internal sensor uses `CAL_MAG0_ROT = -1` on PX4 and `COMPASS_EXTERNAL = 0`, `COMPASS_ORIENT = 0` on ArduPilot outside SITL. Noise maps to estimator assumptions and, in ArduPilot SITL, `SIM_MAG_RND = noise/100` mG. Delay maps to `SIM_MAG_DELAY`.

### 5.1 Calibration values

```text
S_c = S⁻¹
PX4 o = b / 100,000 G
ArduPilot o = −b / 100 mG
```

### 5.2 Replacement in ArduPilot SITL

Set the `SIM_MAG1_DIA`/`ODI` matrix to S⁻¹ and `SIM_MAG1_OFS = −b/100` mG. For calibrated flight, allow start-up hard-iron calibration and set `COMPASS_DIA`/`ODI`. For uncalibrated flight, set `COMPASS_OFS_X = 0.001 mG` before start-up to suppress the automatic copy.

### 5.3 Orientation in ArduPilot SITL

Use `SIM_IMU_ORIENT = 0`, `SIM_MAG1_ORIENT = 0`, and set `COMPASS_ORIENT` to the inverse IMU mounting rotation. Use Custom 2 when it has no numbered representation.

## 6. Acceptance checks

The source checks default compatibility, truth at a north-facing level pose, mounting orientation, and calibration with hard- and soft-iron errors.

## 7. Unsupported characteristics

Motor-current disturbance, spatial field variation, nearby ground iron, temperature effects, exact bandwidth, external or multiple magnetometers are unsupported.

## 8. Requirements

| ID | Requirement | Who | Section |
|---|---|---|---|
| SEN-MAG-010 | Output the magnetic field at the sensor in sensor axes, in nT | Host (input), adapter (output) | 1 |
| SEN-MAG-020 | The geomagnetic field is determined by the three `simulation.location.magneticField` values and does not change during flight | Host | 1.1 |
| SEN-MAG-030 | `intensity_nT` is F in nT, `declination_deg` is D east of true north in degrees, and `inclination_deg` is I downward from horizontal in degrees | Host | 1.1 |
| SEN-MAG-035 | All three `magneticField` values are required; omission of any is an error | Host | 1.1 |
| SEN-MAG-040 | m_ned = F·(cos I·cos D, cos I·sin D, sin I) | Host | 1.1 |
| SEN-MAG-050 | In body axes, m_body = R_nbᵀ·m_ned | Host | 1.1 |
| SEN-MAG-060 | The field does not vary at the sensor location, and mounting position is unused | Host | 1.1, 2 |
| SEN-MAG-070 | Treat the magnetometer as internal and use the IMU mounting pose | Host | 2 |
| SEN-MAG-080 | Use orientation only: m_sensor = Rᵀ·m_body | Host | 2 |
| SEN-MAG-090 | Processing order is orientation → soft iron → hard iron → delay → sample → noise → drift → resolution → saturation | Host (input), adapter (output) | 3 |
| SEN-MAG-100 | Noise σ (nT) is a scalar or [x,y,z]; an axis at 0 or less has no noise | Adapter | 3 |
| SEN-MAG-110 | Drift only supports `gauss-markov` from common 4.2, with σ `stddevNT`; other modes are errors | Adapter | 3 |
| SEN-MAG-120 | Hard and soft iron use m' = S·m + b after rotation in sensor axes; S is symmetric 3×3 | Adapter | 3.1 |
| SEN-MAG-130 | `softIron.diagonal` is the diagonal of S; default [1,1,1] | Adapter | 3.1 |
| SEN-MAG-140 | `softIron.offDiagonal` is [S₀₁,S₀₂,S₁₂], with symmetric S; default [0,0,0] | Adapter | 3.1 |
| SEN-MAG-150 | `hardIronNT` is b in nT; default [0,0,0] | Adapter | 3.1 |
| SEN-MAG-160 | S must be invertible | Adapter | 3.1 |
| SEN-MAG-165 | S with determinant magnitude below 1e-9, or non-finite S or b, is an error | Adapter | 3.1 |
| SEN-MAG-170 | Round to the nearest multiple of `resolutionNT` before saturation | Adapter | 3.2 |
| SEN-MAG-180 | Halfway values round away from zero; the step is common to all axes | Adapter | 3.2 |
| SEN-MAG-190 | Without `resolutionNT`, do not round | Adapter | 3.2 |
| SEN-MAG-200 | Saturation is ±`fullScaleGauss` × 100,000 nT and is applied last | Adapter | 3.3 |
| SEN-MAG-210 | Apply saturation per sensor axis | Adapter | 3.3 |
| SEN-MAG-220 | Do not report saturation upward | Adapter | 3.3 |
| SEN-MAG-230 | Without `range`, do not clip; validate `fullScaleGauss` under common 4.0 | Adapter | 3.3 |
| SEN-MAG-240 | In Control Link, the host adapter receives sensor-axis values, converts with `SENS_BOARD_ROT` or `AHRS_ORIENTATION`, and passes them to the EKF | Host | 4 |
| SEN-MAG-250 | PX4 SITL receives gauss in `HIL_SENSOR`; PX4 converts with `SENS_BOARD_ROT` | Host | 4 |
| SEN-MAG-260 | Hakoniwa sends no magnetic field to ArduPilot SITL; ArduPilot SITL creates it | Host | 4 |
| SEN-MAG-270 | In ArduPilot SITL, replace hard iron, soft iron, noise, and delay with SITL parameters | Host | 4 |
| SEN-MAG-275 | ArduPilot SITL does not apply `drift`, `resolutionNT`, or `range`; the configuration tool records them | Host | 4 |
| SEN-MAG-280 | ArduPilot SITL does not replace geomagnetic truth; it determines truth from latitude and longitude | Host | 4 |
| SEN-MAG-290 | The user sets `simulation.location.magneticField` to the world geomagnetic model at the same location | Host | 4 |
| SEN-MAG-295 | Compare `magneticField` with the world model and record a warning for differences over 1% intensity or 0.5° declination/inclination; do not rewrite it | Host | 4 |
| SEN-MAG-300 | For an internal sensor, PX4 uses `CAL_MAG0_ROT = -1`; ArduPilot Control Link and hardware use `COMPASS_EXTERNAL = 0` and `COMPASS_ORIENT = 0` | Host | 5 |
| SEN-MAG-310 | ArduPilot SITL uses `SIM_IMU_ORIENT = 0`, `SIM_MAG1_ORIENT = 0`, and `COMPASS_ORIENT` equal to the inverse IMU mounting rotation, or Custom 2 | Host | 5, 5.3 |
| SEN-MAG-320 | In ArduPilot SITL, `SIM_MAG_RND = noise / 100` mG | Host | 5 |
| SEN-MAG-330 | In ArduPilot SITL, `SIM_MAG_DELAY = delayMsec` ms | Host | 5 |
| SEN-MAG-335 | For per-axis noise, use the maximum of the three σ values for `SIM_MAG_RND` and record the difference | Host | 5 |
| SEN-MAG-340 | Calibration matrix S_c uses `*SCALE`/`COMPASS_DIA_*` for its diagonal and `*ODIAG`/`COMPASS_ODI_*` for [S₀₁,S₀₂,S₁₂] | Host | 5.1 |
| SEN-MAG-350 | Calibration is S_c = S⁻¹, with PX4 o = b/100,000 G and ArduPilot o = −b/100 mG | Host | 5.1 |
| SEN-MAG-360 | Setting calibration removes hard and soft iron; without it, the errors enter estimation | Host | 5.1 |
| SEN-MAG-370 | ArduPilot SITL uses `SIM_MAG1_DIA`/`ODI` matrix = S⁻¹ and `SIM_MAG1_OFS = −b/100` mG | Host | 5.2 |
| SEN-MAG-380 | For calibrated ArduPilot SITL, leave hard iron to automatic start-up calibration and set 5.1 `COMPASS_DIA`/`ODI` for soft iron | Host | 5.2 |
| SEN-MAG-390 | For uncalibrated ArduPilot SITL, set `COMPASS_OFS_X = 0.001 mG` before start-up to stop the start-up copy | Host | 5.2 |
