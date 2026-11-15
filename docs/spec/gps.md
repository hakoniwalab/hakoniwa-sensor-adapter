# GPS specification

This is the specification of GPS behind `gps_sensor_adapter.hpp`. The design is in [mag-baro-gps-adapter.md](../design/mag-baro-gps-adapter.md).
It carries over the Hakoniwa drone GPS requirements with the same IDs (`SEN-GPS-*`). Common rules are in [drone-common.md](drone-common.md).

## 1. What it measures

Output WGS-84 latitude/longitude in degrees, altitude above sea level in meters, NED velocity in m/s, horizontal ground speed, course clockwise from true north, reported accuracies, satellite count, and fixed 3D fix. Course alone becomes invalid below 0.1 m/s horizontal speed.

## 2. Truth

### 2.1 Antenna position and velocity

```text
p_ant = p + R_nb · r
v_ant = v + R_nb · (ω × r)
```

### 2.2 Conversion to latitude, longitude, and altitude

Use `simulation.location` as the origin and the WGS-84 meridian and prime-vertical curvature radii at that origin. The source uses the local tangent approximation. It does not model a geoid and uses sea-level altitude in place of ellipsoid height in the radii.

## 3. Mounting pose

Use antenna mounting position, not orientation.

## 4. Processing order

```text
antenna position/velocity (NED) → delay → sample → noise → drift → geodetic conversion → output
```

Errors are added in NED meters and m/s before conversion. Ground speed and course come from the noisy velocity. GPS-specific settings require `updatePeriodMsec`; existing configurations without those settings remain readable.

### 4.1 Noise and drift

```json
"positionNoise": { "horizontalM": 0.5, "verticalM": 1.0 },
"velocityNoise": { "horizontalMps": 0.1, "verticalMps": 0.2 },
"drift": { "mode": "gauss-markov", "horizontalM": 1.0, "verticalM": 2.0, "correlationTimeSec": 120 }
```

Horizontal noise and drift are independent on north and east. Vertical applies downward. Drift uses the common Gauss-Markov process.

### 4.2 Reported accuracy

```json
"accuracy": { "horizontalM": 1.5, "verticalM": 3.0, "speedMps": 0.3 },
"satellitesVisible": 12
```

Defaults are 0.5 m horizontal, 0.75 m vertical, 0.5 m/s speed, and 10 satellites. Satellite count is 0 through 254.

## 5. Hand-off to the autopilot

Control Link passes SI units to PX4 EKF or ArduPilot EKF3. PX4 SITL uses `HIL_GPS` integer units and 65535 for invalid course. Hakoniwa sends no GPS measurement to ArduPilot SITL; it sends true vehicle-origin state for that SITL to build its own GPS. ArduPilot Control Link truncates through u-blox receiver units and uses correct mm/s speed accuracy.

## 6. Mapping to PX4 and ArduPilot parameters

Position maps to the autopilot antenna offsets and `SIM_GPS_POS_*`. Delay maps to estimator delay and `SIM_GPS_LAG_MS`. Velocity noise maps to `SIM_GPS_VERR_* = σ√3`; position error has no corresponding ArduPilot SITL model. Accuracy, satellites, and period map to `SIM_GPS_ACC`, `SIM_GPS_NUMSATS`, and rounded `SIM_GPS_HZ`.

## 7. SDFormat mapping

SDFormat `<navsat>` has related position/velocity noise, dynamic bias, and pose fields. It lacks reported accuracy, satellite count, and delay. gz-sensors 8/10 adds horizontal position noise in degrees despite SDFormat's meter meaning; Hakoniwa adds it in meters.

## 8. Acceptance checks

The source checks compatibility, coordinate conversion, antenna lever arm, noise units, course validity, and stable flight with matched estimator assumptions.

## 9. Unsupported characteristics

Satellite geometry/occlusion, detailed multipath, changing fix type, time to first fix, geoid height, long-range curvature accuracy, dual-antenna heading, interference/spoofing, and multiple GPS receivers are unsupported.

## 10. Requirements

| ID | Requirement | Who | Section |
|---|---|---|---|
| SEN-GPS-010 | Output latitude and longitude in WGS-84 degrees | Adapter | 1 |
| SEN-GPS-020 | Output altitude above sea level in meters | Adapter | 1 |
| SEN-GPS-030 | Output north, east, down velocity in m/s | Adapter | 1 |
| SEN-GPS-040 | Ground speed is horizontal speed in m/s | Adapter | 1 |
| SEN-GPS-050 | Course is horizontal-velocity direction, clockwise from true north in [0,360) degrees; 360 is 0. It is invalid below 0.1 m/s horizontal speed | Adapter | 1 |
| SEN-GPS-060 | Horizontal and vertical position accuracy in meters is receiver-reported, not actual error | Adapter | 1 |
| SEN-GPS-070 | Speed accuracy in m/s is receiver-reported, not actual error | Adapter | 1 |
| SEN-GPS-080 | Output the configured satellite count | Adapter | 1 |
| SEN-GPS-090 | Fix type is fixed at 3D | Adapter | 1 |
| SEN-GPS-095 | Only course becomes invalid; all other outputs always have valid values | Adapter | 1 |
| SEN-GPS-100 | Compute antenna position and velocity from vehicle-origin motion with common 2.4 rigid-body formulas p+R_nb·r and v+R_nb·(ω×r) | Host | 2.1 |
| SEN-GPS-110 | Convert latitude, longitude, and altitude from `simulation.location` using section 2.2 and WGS-84 curvature radii at the origin | Host | 2.2 |
| SEN-GPS-120 | Do not model a geoid; use sea-level altitude in place of ellipsoid height in curvature-radius calculations | Host | 2.2 |
| SEN-GPS-130 | Use mounting position | Host | 3 |
| SEN-GPS-140 | Do not use mounting orientation | Host | 3 |
| SEN-GPS-150 | Processing order is antenna position/velocity → delay → sample → noise → drift → conversion | Host (input), adapter (output) | 4 |
| SEN-GPS-155 | Add all errors in NED meters and m/s before latitude/longitude conversion | Adapter | 4 |
| SEN-GPS-160 | Compute ground speed and course from velocity after noise so they remain consistent | Adapter | 4 |
| SEN-GPS-170 | Produce a new value each `updatePeriodMsec` and repeat the preceding value between updates | Adapter | 4 |
| SEN-GPS-175 | `updatePeriodMsec` is required when using this specification's GPS fields; existing configurations without them remain readable | Adapter | 4 |
| SEN-GPS-180 | Add independent normal position noise: horizontal σ to north/east and vertical σ to down, in meters | Adapter | 4.1 |
| SEN-GPS-190 | Add independent normal velocity noise: horizontal σ to north/east and vertical σ to down, in m/s | Adapter | 4.1 |
| SEN-GPS-200 | `gauss-markov` drift follows common 4.2 with separate horizontal north/east and vertical σ and shared `correlationTimeSec` | Adapter | 4.1 |
| SEN-GPS-210 | Omitted or nonpositive noise disables that component. Omitted or zero drift disables it; negative drift σ or nonpositive correlation time is an error | Adapter | 4.1 |
| SEN-GPS-220 | `accuracy.horizontalM` and `verticalM` report position accuracy; defaults are 0.5 m and 0.75 m | Adapter | 4.2 |
| SEN-GPS-230 | `accuracy.speedMps` reports speed accuracy; default 0.5 m/s | Adapter | 4.2 |
| SEN-GPS-240 | `satellitesVisible` is 0–254, default 10; outside the range is an error | Adapter | 4.2 |
| SEN-GPS-250 | Control Link passes position in m, velocity in m/s, position accuracy in m, and speed accuracy in m/s to PX4 EKF or ArduPilot EKF3 | Host | 5 |
| SEN-GPS-260 | PX4 SITL receives section 5 units through `HIL_GPS`; invalid course is 65535 and speed accuracy is not carried | Host | 5 |
| SEN-GPS-270 | Hakoniwa sends no GPS to ArduPilot SITL; it sends true vehicle-origin position and velocity as vehicle state | Host | 5 |
| SEN-GPS-280 | The Hakoniwa adapter uses section 1 outputs and ignores mounting pose | Host | 5 |
| SEN-GPS-290 | In ArduPilot SITL, `SIM_GPS_POS_X/Y/Z = positionM` from the vehicle origin in FRD | Host | 6 |
| SEN-GPS-300 | `SENS_GPS0_DELAY` and `GPS1_DELAY_MS` equal `delayMsec` in ms | Host | 6 |
| SEN-GPS-310 | In ArduPilot SITL, `SIM_GPS_LAG_MS = delayMsec` | Host | 6 |
| SEN-GPS-320 | In ArduPilot SITL, `SIM_GPS_VERR_X/Y/Z` equals north/east/down velocity-noise σ × √3 | Host | 6 |
| SEN-GPS-330 | In ArduPilot SITL, `SIM_GPS_NOISE = 0` and `SIM_GPS_DRIFTALT = 0` | Host | 6 |
| SEN-GPS-340 | In ArduPilot SITL, `SIM_GPS_ACC = accuracy.horizontalM`; `verticalM` cannot be replaced separately | Host | 6 |
| SEN-GPS-350 | Map `satellitesVisible` to ArduPilot SITL `SIM_GPS_NUMSATS` | Host | 6 |
| SEN-GPS-360 | In ArduPilot SITL, `SIM_GPS_HZ` is 1000/`updatePeriodMsec` rounded to the nearest integer | Host | 6 |
| SEN-GPS-370 | ArduPilot SITL does not reproduce position noise or drift; test GPS position error with Control Link or PX4 SITL | Host | 6 |
| SEN-GPS-380 | ArduPilot Control Link truncates GPS values toward zero in ArduPilot SITL u-blox units and passes them through equivalent receiver processing to EKF3; speed accuracy uses correct mm/s and does not reproduce the SITL 1/10 bug | Host | 5 |
| SEN-GPS-385 | In ArduPilot Control Link, when `GPS1_DELAY_MS = 0`, use the required `runtime.gps_receiver_default_lag_ms` as ArduPilot does | Host | 6 |
