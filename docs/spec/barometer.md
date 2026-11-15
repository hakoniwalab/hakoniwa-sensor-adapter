# Barometer specification

This is the specification of the barometer behind `baro_sensor_adapter.hpp`. The design is in [mag-baro-gps-adapter.md](../design/mag-baro-gps-adapter.md).
It carries over the Hakoniwa drone barometer requirements with the same IDs (`SEN-BAR-*`). Common rules are in [drone-common.md](drone-common.md).

## 1. What it measures

The outputs are static pressure (Pa), pressure altitude (m), and temperature (°C). Differential pressure is 0.

## 2. Truth: standard atmosphere

```text
h = simulation.location.altitude − z_sensor
H = r₀ h / (r₀ + h)        h = r₀ H / (r₀ − H)
```

`z_sensor` is the sensor's downward world-NED coordinate. r₀ = 6,356,766 m. Sea-level P₀ and T₀ come from the environment and may change; defaults are 101,325 Pa and 15 °C. Temperatures in formulas are kelvin.

| Geopotential altitude | Temperature | Pressure |
|---|---|---|
| H ≤ 11,000 m | T = T₀ + L·H | P = P₀·(T₀/T)^(g·M/(R·L)) |
| 11,000–20,000 m | T = T₁₁ | P = P₁₁·exp(−g·M·(H−11,000)/(R·T₁₁)) |
| Above 20,000 m | T = T₁₁ | Clamp formula altitude to 20,000 m and warn once |

L = −0.0065 K/m, M = 0.0289644 kg/mol, R = 8.31432 J/(mol·K), and g = 9.80665 m/s². Invalid environmental weather is ignored with a warning and the last valid values remain in use.

## 3. Mounting pose

Use mounting position only for height. Do not use orientation.

## 4. Processing order

```text
h → delay → sample → zero offset → noise → drift → pressure conversion → resolution → saturation
```

Errors through drift are in meters. Resolution and range are in pressure. Pressure altitude is inverted from final pressure; temperature is evaluated at true height. Delay applies to height, not sea-level weather.

### 4.1 Zero offset

`zeroOffset.fixedM` is a fixed meter offset, positive upward. `randomStdM` is drawn once at start-up. They are mutually exclusive and remain fixed during flight.

## 5. Hand-off to the autopilot

PX4 Control Link averages pressure at `SENS_BARO_RATE` and converts it relative to `SENS_BARO_QNH`. ArduPilot Control Link reads the newest pressure at `runtime.baro_update_hz`, performs the configured warm-up and calibration sample counts, and sends relative altitude to EKF3. PX4 SITL receives hPa, pressure altitude, and temperature in `HIL_SENSOR`. Hakoniwa sends no pressure to ArduPilot SITL.

## 6. Mapping to PX4 and ArduPilot parameters

P₀ maps to `SENS_BARO_QNH`; noise maps to estimator noise and `SIM_BARO_RND = noise·√3`; delay maps to estimator delay and `SIM_BARO_DELAY`; `SIM_BARO_DRIFT = 0`. Unrepresentable ArduPilot SITL characteristics are recorded.

## 7. SDFormat mapping

SDFormat `<air_pressure>` has corresponding reference altitude, pressure noise, and precision, but differs in axes, error units, weather, outputs, bias semantics, and range support.

## 8. Acceptance checks

The source checks default compatibility, atmosphere values, pressure-altitude inversion, mounting height, noise units, and flight with matched estimator assumptions.

## 9. Unsupported characteristics

Propeller-flow and ground-effect disturbance, sensor-temperature effects, temperature output error, differential pressure, atmosphere above 20 km, humidity, exact internal filtering, and multiple barometers are unsupported.

## 10. Requirements

| ID | Requirement | Who | Section |
|---|---|---|---|
| SEN-BAR-010 | Output pressure (Pa), pressure altitude (m), and temperature (°C) | Adapter | 1 |
| SEN-BAR-020 | Output differential pressure as 0 | Adapter | 1 |
| SEN-BAR-030 | Compute pressure and temperature with the standard-atmosphere form at h = `simulation.location.altitude` − z_sensor in meters above sea level | Host (input), adapter (output) | 2 |
| SEN-BAR-040 | z_sensor is the downward world-NED coordinate of the sensor position computed by common 2.4 | Host | 2 |
| SEN-BAR-050 | Receive sea-level P₀ and T₀ from the environment; they may change during flight. Use kelvin in formulas | Host (input), adapter (output) | 2 |
| SEN-BAR-060 | Without P₀/T₀ input, use 101,325 Pa and 15 °C | Adapter | 2 |
| SEN-BAR-070 | At and below 11,000 m, including below sea level, T=T₀+L·h and P=P₀·(T₀/T)^(g·M/(R·L)) | Adapter | 2 |
| SEN-BAR-080 | From 11,000 through 20,000 m, T=T₁₁ and P=P₁₁·exp(−g·M·(h−11,000)/(R·T₁₁)) | Adapter | 2 |
| SEN-BAR-085 | Above 20,000 m, clamp formula altitude at 20,000 m, keep P=P₂₀, and warn only once | Adapter | 2 |
| SEN-BAR-090 | Use L=−0.0065 K/m, M=0.0289644 kg/mol, R=8.31432 J/(mol·K), and g=g₀=9.80665 m/s² | Adapter | 2 |
| SEN-BAR-095 | Use geopotential altitude H=r₀h/(r₀+h), r₀=6,356,766 m, for formulas and layer boundaries; convert inverted altitude back to geometric height | Adapter | 2 |
| SEN-BAR-100 | Require P₀>0, T₀>−273.15 °C, and T>0 K in every layer; pressure decreases with height in this range | Adapter | 2 |
| SEN-BAR-105 | For invalid environmental P₀/T₀ (P₀≤0 or T₀≤−201.65 °C), keep the preceding valid values and warn; do not error | Adapter | 2 |
| SEN-BAR-110 | Invert pressure with the first formula for P≥P₁₁ and the second for P₂₀≤P<P₁₁ | Adapter | 2 |
| SEN-BAR-120 | Use mounting position only for height; tilting the vehicle changes sensor height | Host | 3 |
| SEN-BAR-130 | Do not use mounting orientation | Host | 3 |
| SEN-BAR-140 | Process h → delay → sample → zero offset → noise → drift → pressure conversion → resolution → saturation. Add errors in meters; apply resolution and saturation in Pa | Host (input), adapter (output) | 4 |
| SEN-BAR-150 | `noise` is normal-distribution σ in meters; 0 or less means no noise | Adapter | 4 |
| SEN-BAR-160 | Drift is `gauss-markov`, with σ `stddevM` and τ `correlationTimeSec` | Adapter | 4 |
| SEN-BAR-170 | Round resolution to the nearest `resolutionPa` step; halfway values round away from zero | Adapter | 4 |
| SEN-BAR-180 | Clamp outside `range.minHPa` through `range.maxHPa` to the endpoint | Adapter | 4 |
| SEN-BAR-184 | `noise` is scalar; [x,y,z] is an error | Adapter | 4 |
| SEN-BAR-186 | Only `gauss-markov` drift mode is allowed | Adapter | 4 |
| SEN-BAR-190 | Invert pressure altitude from pressure after saturation with section 2 formulas | Adapter | 4 |
| SEN-BAR-200 | Temperature is at true sensor height and is unaffected by pressure error; add no temperature error | Adapter | 4 |
| SEN-BAR-210 | Delay height, not sea-level weather; use current weather at conversion | Adapter | 4 |
| SEN-BAR-220 | `zeroOffset.fixedM` is a fixed meter offset; positive reads higher | Adapter | 4.1 |
| SEN-BAR-230 | Draw `zeroOffset.randomStdM` once at start-up from a normal distribution in meters | Adapter | 4.1 |
| SEN-BAR-240 | Only one of `fixedM` and `randomStdM`; omission means no offset | Adapter | 4.1 |
| SEN-BAR-250 | Zero offset does not change during flight | Adapter | 4.1 |
| SEN-BAR-325 | PX4 Control Link averages pressure at `SENS_BARO_RATE`, converts relative to `SENS_BARO_QNH` with PX4 formulas, and passes height to the EKF | Host | 5 |
| SEN-BAR-326 | ArduPilot Control Link reads newest pressure at `runtime.baro_update_hz`, calibrates ground pressure using the required warm-up and sample counts, converts with the 1976 standard atmosphere, and passes height to EKF3 | Host | 5 |
| SEN-BAR-270 | PX4 SITL receives pressure in hPa, pressure altitude, and temperature in `HIL_SENSOR` | Host | 5 |
| SEN-BAR-280 | Hakoniwa sends no pressure to ArduPilot SITL | Host | 5 |
| SEN-BAR-290 | The Hakoniwa adapter uses pressure altitude and ignores mounting pose | Host | 5 |
| SEN-BAR-300 | Set PX4 `SENS_BARO_QNH` to start-up P₀ in hPa | Host | 6 |
| SEN-BAR-310 | In ArduPilot SITL, `SIM_BARO_RND = noise × √3` | Host | 6 |
| SEN-BAR-320 | `EKF2_BARO_DELAY` and `EK3_HGT_DELAY` equal `delayMsec`; they do not add another delay | Host | 6 |
| SEN-BAR-330 | In ArduPilot SITL, `SIM_BARO_DELAY = delayMsec` | Host | 6 |
| SEN-BAR-340 | In ArduPilot SITL, `SIM_BARO_DRIFT = 0` | Host | 6 |
| SEN-BAR-350 | Record ArduPilot SITL characteristics that do not appear in pressure: position, `zeroOffset`, drift, resolution, range, and weather changes | Host | 6 |
