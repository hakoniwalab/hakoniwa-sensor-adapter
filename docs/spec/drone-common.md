# Common drone sensor specification

This is the common specification for the drone sensors behind `imu_sensor_adapter.hpp`, `mag_sensor_adapter.hpp`, `baro_sensor_adapter.hpp`, and `gps_sensor_adapter.hpp`. The designs are in [imu-adapter.md](../design/imu-adapter.md) and [mag-baro-gps-adapter.md](../design/mag-baro-gps-adapter.md).
It carries over the common Hakoniwa drone sensor requirements with the same IDs (`SEN-COM-*`).

## 1. Coordinates and units

World axes are NED, vehicle axes are FRD, lengths are meters, angular velocity is rad/s, and drone_config angles are degrees.

### 1.1 Physics premise

The host obtains truth from the MuJoCo vehicle body.

### 1.2 Vehicle origin and center of gravity

The vehicle origin is the origin of the MJCF body with the free joint. Mounting positions are relative to it. The center of gravity is used only when deriving autopilot position settings.

## 2. Mounting pose

### 2.1 Form

```json
"mount": {
  "positionM": [0.05, 0.0, -0.02],
  "orientationDeg": { "roll": 0.0, "pitch": 0.0, "yaw": 90.0 }
}
```

Omission means the vehicle origin and zero orientation.

### 2.2 One IMU pose

`components.sensors.imu.mount` applies to accelerometer, gyroscope, and magnetometer; barometer and GPS have separate poses. `acc`, `gyro`, and `mag` must not contain `mount`.

### 2.3 Rotation

```text
R = Rz(yaw) · Ry(pitch) · Rx(roll)
v_sensor = Rᵀ · v_body
v_body   = R  · v_sensor
```

### 2.4 Position effects

```text
position:     p + R_nb · r
velocity:     v + R_nb · (ω × r)
acceleration: a + α × r + ω × (ω × r)
```

## 3. Use by sensor

Gyroscope and magnetometer use orientation only; accelerometer uses both; barometer uses height only; GPS uses position only.

## 4. Processing order

```text
truth → mounting position/orientation → delay → sample → noise → drift → saturation → output
```

New values occur only at sample boundaries. Delay is rounded to physics steps. `softDelayMsec` is host/autopilot software delay. `updatePeriodMsec` is the sensor output period.

### 4.0 Omission and errors

Omitted error characteristics are disabled. Invalid types and non-finite values are errors. Unknown setting names warn. Sensor-specific rules take precedence.

### 4.1 Random numbers and reset

Each characteristic has a separate stream derived from random seed, vehicle name/index, sensor name, and characteristic. Reset restores every state and stream.

### 4.2 Gauss-Markov drift

```text
φ = exp(−Δt/τ)
σ_d = σ·√(1−exp(−2Δt/τ))
b_k = φ·b_(k−1) + N(0,σ_d²)
```

Initial b₀ is N(0,σ²). For SDFormat, `dynamic_bias_stddev = σ·√(2/τ)`.

### 4.3 Noise density

```text
σ = noiseDensity / √Δt_s
```

`noiseDensity` requires `updatePeriodMsec` and is mutually exclusive with `noise`. GPS does not accept it.

### 4.4 Rotor vibration

The optional upstream [vibration model](https://github.com/toppers/hakoniwa-drone-core/blob/221afa40809c9edb476f88c71383a43c134d4b9d/docs/architecture/aircraft/sensor/vibration/README.md) supplies physical-step IMU disturbances without changing physics.

### 4.5 IMU filter

The optional accelerometer and gyroscope filters run separately in sensor axes at every physics step, before delay and sampling. Supported forms are even-order Chebyshev II and Butterworth filters. Do not count their delay again in `delayMsec` or autopilot filters.

## 5. Matching the autopilot

The host derives autopilot orientation and position settings from the mounting pose. Position settings are relative to the MuJoCo center of gravity.

### 5.1 Setting mapping

PX4 uses `SENS_BOARD_ROT`, board offsets, and EKF sensor positions. ArduPilot uses `AHRS_ORIENTATION`, custom rotations, and sensor positions.

### 5.2 Orientation settings

PX4 chooses a numbered rotation plus ±45° offsets or errors if none represents the pose. ArduPilot uses a numbered rotation or Custom 1.

### 5.3 Conversion to body axes

The host/autopilot converts sensor-axis values and supplies sensor positions to its EKF. Hakoniwa's built-in adapter does not compensate `mount`.

### 5.4 Autopilot versions

The behavior was checked against ArduCopter 4.6.3 (`92b0cd788e`) and PX4 `a1726d316a`. Valid autopilot behavior is adopted; SITL bugs are not reproduced.

### 5.5 Thrust allocation

Control Link and PX4 SITL compute rotor positions relative to the MuJoCo center of gravity. ArduPilot does not use rotor positions.

### 5.6 EKF assumptions

When `controller.ekf.fromSensors` applies, host EKF noise assumptions come from sensor settings, respect PX4 minima, override same-named EKF-file values, and are printed at start-up.

## 6. Unsupported behavior

Unsupported common behavior includes vehicle flex, motor-current magnetic disturbance, multiple IMUs, external magnetometers, altimeters, and IMU orientation output. Averaging is disabled unless the optional IMU filter is enabled; `sampleCount` accepts only 1.

## 7. SDFormat mapping

The source maps corresponding pose, Gaussian noise, bias, Gauss-Markov drift, precision, update rate, magnetic truth, and reset concepts. Hakoniwa additionally specifies saturation, sensitivity, hard/soft iron, delay, gyro drift variants, and rigid-body position effects.

## 7.5 Design principle

Sensor calculation consumes truth through its interface and does not depend on physics, communication, or autopilot APIs.

## 7.6 Compatibility and migration

The sensor-model-v2 switch preserves old bitwise output when off except for common g₀. When on, old configurations remain readable but output is intentionally incompatible. Migration requires configuration, flight, tuning, and path-correlation checks.

## 8. Requirements

| ID | Requirement | Who | Section |
|---|---|---|---|
| SEN-COM-010 | World is NED and vehicle is FRD; length is m, angular velocity rad/s, and drone_config angles degrees | Host (input), adapter (output) | 1 |
| SEN-COM-020 | Obtain sensor truth from the MuJoCo vehicle body | Host | 1.1 |
| SEN-COM-030 | Position reference is the vehicle origin, the MJCF free-joint body origin | Host | 1.2 |
| SEN-COM-040 | Write mounting pose as `positionM` from the origin in FRD meters and `orientationDeg` in degrees | Host | 2.1 |
| SEN-COM-050 | Omitting `mount` means the origin with vehicle-aligned axes | Host | 2.1 |
| SEN-COM-060 | One IMU pose affects accelerometer, gyroscope, and magnetometer (orientation only for magnetometer); barometer and GPS are separate | Host | 2.2 |
| SEN-COM-070 | `mount` under `acc`, `gyro`, or `mag` is an error | Host | 2.2 |
| SEN-COM-080 | R = Rz(yaw)·Ry(pitch)·Rx(roll) | Host | 2.3 |
| SEN-COM-090 | Sensor-axis value is Rᵀ·v_body; body-axis value is R·v_sensor | Host | 2.3 |
| SEN-COM-100 | Use one rotation function for every sensor | Host | 2.3 |
| SEN-COM-110 | Derive sensor-location position, velocity, and acceleration from origin motion with rigid-body formulas, without the center of gravity | Host | 2.4 |
| SEN-COM-120 | Use position and orientation per the section 3 table | Host | 3 |
| SEN-COM-130 | Common stage order is mounting pose → delay → sample → noise → drift → saturation | Host (input), adapter (output) | 4 |
| SEN-COM-140 | Create values only at sample boundaries and repeat them between boundaries; control and logs read the same value | Adapter | 4 |
| SEN-COM-150 | Apply mounting pose first | Host | 4 |
| SEN-COM-160 | Round delay to an integer physics-step multiple and output the first input until history fills | Adapter | 4 |
| SEN-COM-170 | Retired (averaging; sensors do not average; see SEN-COM-176) | Adapter | — |
| SEN-COM-175 | At `updatePeriodMsec` boundaries 0,T,2T,… sample the post-delay value once; without it, sample every physics step | Adapter | 4 |
| SEN-COM-176 | Do not smooth by default; smooth only with an explicitly enabled IMU filter. `sampleCount` accepts 1 and rejects 2 or more | Adapter | 6 |
| SEN-COM-177 | If `updatePeriodMsec` is not an integer physics-step multiple, round to the nearest multiple of at least one step and warn with sensor name and rounded period | Adapter | 4 |
| SEN-COM-180 | Add independent zero-mean normal noise per axis and value; σ is scalar or [x,y,z] | Adapter | 4 |
| SEN-COM-185 | Specify noise as recommended density `noiseDensity` or σ `noise`; σ=`noiseDensity`/√Δt_s where Δt_s is `updatePeriodMsec` | Adapter | 4.3 |
| SEN-COM-186 | Converted density σ behaves identically to directly specified `noise` σ | Adapter | 4.3 |
| SEN-COM-187 | Both noise forms together, or `noiseDensity` on GPS, is an error | Adapter | 4.3 |
| SEN-COM-188 | `noiseDensity` requires `updatePeriodMsec` | Adapter | 4.3 |
| SEN-COM-190 | Clamp values outside the measurable range to its endpoint | Adapter | 4 |
| SEN-COM-200 | `softDelayMsec` is delay from autopilot receipt until use | Host | 4 |
| SEN-COM-210 | `updatePeriodMsec` is the period at which the sensor measures and emits a new value | Adapter | 4 |
| SEN-COM-211 | Omitted or nonpositive noise σ means no noise; non-finite values and [x,y,z] for scalar quantities are errors | Adapter | 4.0 |
| SEN-COM-212 | Omitted resolution means no rounding; nonpositive or non-finite resolution is an error | Adapter | 4.0 |
| SEN-COM-213 | Omitted range fields do not clip; nonpositive, min≥max, or non-finite fields are errors | Adapter | 4.0 |
| SEN-COM-214 | Omitted zero offset means none; both forms together or non-finite values are errors | Adapter | 4.0 |
| SEN-COM-215 | Sensor-specific rules prevail; wrong types are errors everywhere | Adapter | 4.0 |
| SEN-COM-216 | Unknown setting names warn instead of erroring | Adapter | 4.0 |
| SEN-COM-217 | Configuration tools record unconvertible differences | Host | 4.0 |
| SEN-COM-218 | Omitted drift means none; unknown mode, negative σ, nonpositive correlation time, or non-finite value is an error | Adapter | 4.0 |
| SEN-COM-219 | Omitted pose means origin/aligned; wrong length/type, non-finite values, or pose on a forbidden sensor is an error | Host | 4.0 |
| SEN-COM-220 | Use separate random streams by characteristic, derived from randomSeed, vehicle name/index, sensor name, and characteristic; equal seeds reproduce sequences | Adapter | 4.1 |
| SEN-COM-230 | Reset delay, sample state/phase, drift, start-up offset, and random streams | Adapter | 4.1 |
| SEN-COM-240 | Slow drift is section 4.2 first-order Gauss-Markov with stationary σ | Adapter | 4.2 |
| SEN-COM-250 | Draw initial Gauss-Markov state from N(0,σ²) | Adapter | 4.2 |
| SEN-COM-260 | Add fixed offset and drift separately; fixed offset remains constant in flight | Adapter | 4.2 |
| SEN-COM-270 | Matching orientation in the autopilot yields flight like an aligned installation | Host | 5 |
| SEN-COM-280 | Matching position in the autopilot yields approximately origin-mounted flight | Host | 5 |
| SEN-COM-290 | The host derives autopilot position as `positionM` minus center of gravity | Host | 5.1 |
| SEN-COM-295 | Autopilot estimated position/velocity is at the center of gravity; test IMU position with rotation | Host | 5.1 |
| SEN-COM-300 | Control Link receives adapter parameters and SITL receives start-up parameters; derive mounting geometry once for tuning and reuse it | Host | 5.1 |
| SEN-COM-310 | Derive PX4 orientation by section 5.2; unrepresentable orientation is an error | Host | 5.2 |
| SEN-COM-320 | Use an ArduPilot numbered orientation when matched, otherwise Custom 1 | Host | 5.2 |
| SEN-COM-330 | Set orientation before simulation starts | Host | 5.2 |
| SEN-COM-340 | PX4 Control Link converts with `SENS_BOARD_ROT`/`SENS_BOARD_*_OFF` before EKF/control and passes IMU/GPS positions to EKF | Host | 5.3 |
| SEN-COM-350 | ArduPilot Control Link converts with `AHRS_ORIENTATION` and passes IMU/GPS positions to EKF | Host | 5.3 |
| SEN-COM-360 | Hakoniwa's built-in adapter does not compensate `mount` | Host | 5.3 |
| SEN-COM-370 | When exporting SDFormat, `dynamic_bias_stddev = σ·√(2/τ)` | Host | 4.2, 7 |
| SEN-COM-390 | With the switch off, output is bitwise identical to the prior model, using common standard gravity g₀ | Host | 7.6 |
| SEN-COM-400 | With the switch on, old settings remain readable but output is not identical | Host | 7.6 |
| SEN-COM-410 | On migration, repeat configuration, flight, tuning, and correlation checks from 7.6 | Host | 7.6 |
| SEN-COM-380 | Sensor calculation only consumes truth and does not depend on physics, communication, or autopilot hand-off | Adapter | 7.5 |
| SEN-COM-420 | Autopilot-matching behavior was checked against section 5.4 versions and must be rechecked on upgrade | Host | 5.4 |
| SEN-COM-430 | Adopt correct autopilot behavior, do not reproduce SITL bugs, and treat them as SITL differences in correlation | Host | 5.4 |
| SEN-COM-440 | Match Control Link IMU delivery: ArduPilot interval average without coning correction; PX4 latest sample | Host | 5.4 |
| SEN-COM-450 | Compute Control Link/PX4 SITL thrust allocation from rotor positions relative to MuJoCo center of gravity; ArduPilot is excluded | Host | 5.5 |
| SEN-COM-460 | Apply IMU filters to accelerometer/gyroscope only when explicitly enabled; default off | Adapter | 4.5 |
| SEN-COM-462 | Update IMU filters in sensor axes every physics step before delay and sampling | Adapter | 4.5 |
| SEN-COM-464 | Configure accelerometer and gyroscope filters separately; they work without vibration generation | Adapter | 4.5 |
| SEN-COM-466 | Treat filter amplitude/delay as IMU behavior and repeat PID tuning and correlation when enabled | Host | 4.5 |
| SEN-COM-468 | Do not include filter delay in `delayMsec` or duplicate autopilot software filtering | Host (input), adapter (output) | 4.5 |
| SEN-COM-470 | Without filters and vibration generator, output is bitwise identical to v2 | Adapter | 4.5 |
| SEN-COM-500 | Derive EKF noise assumptions from drone_config sensor values | Host | 5.6 |
| SEN-COM-510 | Use barometer as height reference and sensor σ for barometer, GPS position/velocity, and magnetometer, with table defaults for no-noise sensors | Host | 5.6 |
| SEN-COM-520 | Do not go below PX4 minimum assumptions | Host | 5.6 |
| SEN-COM-530 | Select with `controller.ekf.fromSensors`; when omitted, enable only for `configVersion` 5.0 | Host | 5.6 |
| SEN-COM-540 | Override same-named EKF-file values and print derived values at start-up | Host | 5.6 |
| SEN-COM-550 | Apply only in sensor-model-v2 builds; warn and ignore true elsewhere | Host | 5.6 |
