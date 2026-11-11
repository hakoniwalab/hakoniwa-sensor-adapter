# Sensor evaluation axes

This page defines what to measure, and how, when we compare the Hakoniwa Drone PRO sensors (the Hakoniwa sensor adapter) with the Gazebo sensors (the gz-sensors adapter) through the same interface and with the same truth.
Pass/fail criteria are not set here. We list the numbers and the reasons for the differences.

## 1. Assumptions

### 1.1 Only the sensor error models are compared

The physics stays in Hakoniwa (MuJoCo). Only the sensor part is swapped. The host passes the same truth to both implementations.

```text
Host: truth → mounting pose (rigid-body equations, rotate to sensor axes) → vibration (optional)
                                │
           ┌────────────────────┴────────────────────┐
           ▼                                         ▼
  Hakoniwa sensor adapter                    Gazebo adapter
  (sensor v2 model)                          (gz-sensors)
           │                                         │
           └──────────── line up outputs ────────────┘
```

- The host computes the effects of the mounting position (ω²r, α×r) and the orientation. They are not part of the performance of either implementation. In Gazebo too, this computation is done by the physics side (gz-sim), not by gz-sensors.
- We check that the host computation is correct separately from the comparison, against values computed by hand.

### 1.2 Truth is generated from prescribed motion

Truth is generated as a function of time (for example constant rotation, sinusoidal swaying, constant climb). No physics simulation is run.
Expected values are computed by hand from the equations, independently of both implementations.

### 1.3 Versions compared

| Implementation | Version |
|---|---|
| Hakoniwa | The Hakoniwa Drone PRO sensor v2 models wrapped as adapters of this interface (the same computation as Drone PRO; the source is not public) |
| Gazebo | gz-sensors 10.2.0 (Jetty), standard sensors and noise models. No custom plugins |
| Project AirSim | The core_sim sensors (Imu, Magnetometer, Barometer, Gps) compiled as is (the native adapter of hakoniwa-sensor-adapter-airsim). Pinned to upstream commit `92032a6c`; the sha256 of the 140 files used is checked before building |

## 2. Evaluation axes

| Axis | Question |
|---|---|
| 0. No-noise agreement | With all errors turned off, do they output the same values? If not, where and why do they differ? |
| 1. Accuracy | With error settings of the same meaning, do they give the same statistics? |
| 2. Coverage | How far can the properties of real sensors be expressed by configuration? |
| 3. Repeatability | With the same seed, do they give the same value sequence? |
| 4. Speed | How much time does one physics step take? |
| 5. Adoption effort | The effort and weight needed before it can be used |

We go through the axes in order from axis 0, because a mismatch in an earlier axis causes differences in later axes.

## 3. Axis 0: no-noise agreement

We turn off all noise, bias, drift, resolution, saturation, filtering and delay in both.
The sensor output is then just the truth transformed by fixed equations. No random numbers are used, so **the values can be compared directly** (value by value, not statistically).

This shows:

- whether unit and frame conversions are wrong
- whether the sample times, and the rule for holding values between them, are the same
- differences in the fixed equations inside the sensor (atmosphere model, conversion to latitude and longitude, and so on)

| Sensor | Value compared | Expected value | Expected difference |
|---|---|---|---|
| Gyro | Angular velocity | Same as truth | None (should agree) |
| Accelerometer | Specific force | Same as truth | Gazebo adds and subtracts gravity again, so only a rounding difference (about 1e-15 m/s²) |
| Magnetometer | Magnetic field | World field rotated to the sensor axes | Gazebo computes in tesla, so only rounding from the unit conversion |
| Barometer | Pressure | Standard atmosphere equations (computed independently) | The atmosphere equations differ. Hakoniwa uses geometric height with g = 9.81; Gazebo uses geopotential altitude with different constants. Record the difference as a curve for heights 0 to 3000 m |
| GPS | Latitude, longitude, altitude, velocity | Position and velocity passed by the host | Hakoniwa computes latitude and longitude itself from the local position (no geoid, curvature approximation). Gazebo uses the latitude and longitude passed by the host as is. Record the difference against distance from the origin (0 to 10 km) |
| All | Sample times and `updated` | Time 0, T, 2T, ... | Whether the period is held in Hz or in ms. Compare how the boundary is decided for a period that is not an integer multiple of the physics step (for example 250 Hz with a 3 ms physics step) |

## 4. Axis 1: accuracy

We add error settings one at a time and compare statistics. When one matches, we add the next.

| Stage | Error added | Statistic measured | Expected value |
|---|---|---|---|
| 1 | Noise σ (5 sensors) | Mean and standard deviation (N = 20,000) | Configured σ |
| 2 | Bias | Mean | Configured bias |
| 3 | Bias drawn at start-up | Distribution of values per instance | Configured standard deviation. Gazebo flips the sign with 50% probability |
| 4 | Slowly varying bias (Gauss-Markov) | Variance, autocorrelation (does it reach e⁻¹ at τ) | Hakoniwa: σ. Gazebo: the value converted with `dynamic_bias_stddev = σ·√(2/τ)` |
| 5 | Resolution | Whether output values are integer multiples of the step, rounding mode | Nearest step; ties away from 0 |
| 6 | Sample period | Interval between sample times | Configured period |

The settings are rewritten from drone_config into both formats (Hakoniwa keeps drone_config; Gazebo uses SDFormat). Items that cannot be rewritten go to axis 2.

## 5. Axis 2: coverage

For each property of real sensors, we list whether it can be expressed by configuration. The list is built from the adapter capability tables and checked against the source.

| Property | Example |
|---|---|
| Measurement range (saturation) | Gyro ±2000°/s, accelerometer ±16 g |
| Sensor value delay | Barometer 60 ms, GPS 100 ms |
| Scale error, crosstalk | Gyro ±0.5 to 2 % |
| IMU filter | 6th order, 125 Hz |
| Hard iron, soft iron | Magnetometer |
| Weather | Sea-level pressure and temperature |
| Temperature and pressure altitude output | Barometer |
| Receiver reports | GPS accuracy estimate, number of satellites, fix type, heading |
| Noise unit | Whether GPS horizontal noise is in metres or degrees |

What cannot be expressed is recorded as "not available" and kept, not removed.

## 6. Axis 3: repeatability

| Case | What is checked |
|---|---|
| Same instance | After reset, does it return to the same value sequence? |
| New instance | Recreated with the same seed, does it give the same value sequence? |
| Adding sensors | Does adding other sensors leave one sensor's value sequence unchanged? |
| Different OS | Same value sequence on Mac, Linux and Windows? |
| Multiple threads | Can different instances run at the same time from different threads? |
| Multiple processes on the same PC | Are values from other processes kept out? |

## 7. Axis 4: speed

- Run the five sensors every physics step (1 ms) and measure the time per step (median and 99th percentile).
- Also measure the breakdown by sensor, and memory.
- Measure on the same PC, same OS, without a display.

## 8. Axis 5: adoption effort

| Item | What is measured |
|---|---|
| Dependencies | Number and size of packages installed; number and size of libraries needed at run time |
| OSes it builds on | Mac, Linux, Windows |
| Side effects | Transport threads, network ports, shared state in the process |

## 9. Known before comparing

Known Gazebo behaviour is recorded separately, not used as the comparison reference.

- GPS horizontal noise is added to latitude and longitude in **degrees**, not metres.
- The magnetometer, barometer and GPS call the noise function with a time step of 0, so the time-varying bias does not advance.
- There is one random number generator per process. The order in which sensors are created and called changes the values of other sensors.
- The barometer has no function to read its value; the value comes out only as a transport message.
- On a physics step where no sample was taken, the getter can return the last input passed instead of the last measured value.

## 10. Outputs

For each axis, the following are kept under this directory:

- the test program (which feeds the same truth to both adapters)
- result tables (numbers) and plots
- explanation of the differences (why they differ)
