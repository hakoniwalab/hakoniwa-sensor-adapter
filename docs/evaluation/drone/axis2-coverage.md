# Axis 2: coverage (2026-11-01)

Project AirSim was checked with the core_sim sensors compiled as is.

This axis is whether the properties of real sensors can be expressed by configuration. We checked with the adapter capability tables, the source, and the axis 1 measurements.

- ○: can be expressed (confirmed by measurement)
- △: can be expressed, with conditions
- ×: cannot be expressed

| # | Property | Hakoniwa | Gazebo | Gazebo notes | Project AirSim (core_sim) | Project AirSim notes |
|---|---|---|---|---|---|---|
| 1 | Noise (5 sensors) | ○ | ○ | | △ | GPS has no noise |
| 2 | Bias | ○ | △ | Sign flips with probability one half (`bias_mean`) | △ | Possible for the IMU (turn-on-bias) and the barometer (QNH). Not for the magnetometer |
| 3 | Turn-on bias drawn at start-up | ○ | ○ | | △ | Magnetometer only (drawn with mean −b, standard deviation b). The seed is fixed, so the value is the same every time |
| 4 | Slowly varying bias (IMU) | ○ | ○ | Autocorrelation slightly short (axis 1) | △ | Random walk (bias-stability), not Gauss-Markov |
| 5 | Slowly varying bias (magnetometer, barometer, GPS) | ○ | × | Noise is added with a time step of 0, so the bias does not advance | △ | Barometer only (pressure factor). Steady-state spread is about 1/10 of σ (axis 1) |
| 6 | Resolution | ○ | △ | Steps smaller than 1e-6 are ignored. For the magnetometer (tesla) a real step cannot be written | × |  |
| 7 | Sample period (per sensor) | ○ | ○ | | ○ |  |
| 8 | Separate periods for gyro and accelerometer | ○ | × | One `update_rate` per IMU | × |  |
| 9 | Sensor value delay | ○ | × | | × | The barometer delay setting is read but not used (the adapter refuses non-zero values) |
| 10 | Saturation (measurement range) | ○ | × | | × |  |
| 11 | Scale error, crosstalk | ○ | × | | △ | Scalar scale factor of the magnetometer only |
| 12 | IMU filter | ○ | × | | × |  |
| 13 | Hard iron, soft iron (magnetometer) | ○ | × | A bias can approximate hard iron, but the sign can flip ([axis 1](axis1-accuracy.md)). No soft iron | × |  |
| 14 | Weather (sea-level pressure and temperature) | ○ | × | Fixed to the standard atmosphere | × | Fixed to the standard atmosphere |
| 15 | Temperature and pressure altitude output | ○ | × | Pressure only | △ | Pressure altitude only (the inverse is off by up to 2.65 m). No temperature output |
| 16 | GPS receiver reports (accuracy, number of satellites, heading) | ○ | × | Always a 3D fix | △ | Fix type and EPH/EPV (values not in metres) |
| 17 | GPS horizontal noise specified in metres | ○ | × | Specified in degrees. Shrinks by cos(latitude) in the longitude direction | × | GPS has no noise |
| 18 | Autopilot calibration values (accelerometer) | ○ | × | The bias in use is not exposed | × |  |

## In Gazebo but not in Hakoniwa

| Property | Description |
|---|---|
| Noise mean (`mean`) | Hakoniwa expresses this as a bias |
| Altimeter (`altimeter`) | Not in the Hakoniwa sensor specification |
| IMU orientation output (`enable_orientation`) | Not in the Hakoniwa sensor specification |
| Location-dependent magnetic field | Decided on the gz-sim side (world settings and location). Outside the comparison of sensor error models |
