# Drone sensors: evaluation results (2026-11-19)

These are the results for the six axes in [Evaluation axes](evaluation-axes.md). We compared the Hakoniwa sensors ([hakoniwa-sensor-pro](https://github.com/hakoniwalab/hakoniwa-sensor-pro) v0.1.0, the sensor models of Hakoniwa Drone PRO), the Gazebo sensors (gz-sensors 10.2.0), and the Project AirSim sensors (core_sim compiled as is), through the same interface and with the same truth.

We ran them on macOS (arm64). We do not set pass/fail criteria. We list only the numbers and the reasons for the differences.

| Axis | Hakoniwa | Gazebo | Project AirSim (core_sim) | Details |
|---|---|---|---|---|
| 0. No-noise agreement | Reference | Gyro, accelerometer, magnetometer, barometer (US 1976) and sample times agree. GPS maps onto the Earth differently | Gyro and accelerometer differ by 32-bit rounding; magnetometer and sample times agree. Barometer is within 0.09 Pa of US 1976; pressure altitude is off by up to 2.65 m. GPS treats the Earth as a sphere and outputs latitude and longitude as 32-bit values | [axis0-no-noise.md](axis0-no-noise.md) |
| 1. Accuracy | All 21 items as configured | Sign of the bias flipped with 50 % probability; magnetometer, barometer and GPS dynamic bias does not move; magnetometer resolution has no effect; GPS east noise shrinks | Noise, bias and random walk as configured. The seed is fixed, so every instance gives the same value sequence (the magnetometer turn-on bias is also the same every time). Barometer slowly varying bias is 1/10 of σ. GPS has no noise | [axis1-accuracy.md](axis1-accuracy.md) |
| 2. Coverage (18 items) | 18 | 4 (2 conditional) | 1 (8 conditional) | [axis2-coverage.md](axis2-coverage.md) |
| 3. Repeatability (5 cases) | Same in all 5 cases | Sequence changes with other sensors, threads and reset | Same in all 5 cases (because the seed is always the same; the host seed is not used) | [axis3-repeatability.md](axis3-repeatability.md) |
| 4. Speed (mean per physics step, 5 sensors) | 0.11 to 0.17 µs | 0.62 to 0.72 µs | 0.71 to 0.74 µs | [axis4-speed.md](axis4-speed.md) |
| 5. Adoption effort | No extra dependencies | 37 runtime libraries, about 29 MB, a transport thread | nng 1.11, msgpack-cxx, Eigen, nlohmann/json, and 140 files from a pinned commit. No transport port is opened (threads that only wait are started) | [axis5-adoption.md](axis5-adoption.md) |

## Method

- The comparison program calls the three adapters only through this interface (`IImuSensorAdapter` and others), passes the same truth sequence, and lines up the outputs.
- Everything used is public, so anyone can reproduce every column:
  - the comparison program: [tools/compare](../../../tools/compare) in this repository (build and commands in its README);
  - Hakoniwa: the `hakoniwa` implementation of the hakoniwa-sensor-pro v0.1.0 release archive (binaries), `hakoniwa-sensor-pro-0.1.0-macos-arm64.zip`, SHA-256 `82040299ea2eed32139626b8c30a787fb5bc7c44d0660f303edc2155158deb23`, reached by name through the common interface;
  - Gazebo and Project AirSim: the public adapters [hakoniwa-sensor-adapter-gazebo](https://github.com/hakoniwalab/hakoniwa-sensor-adapter-gazebo) and [hakoniwa-sensor-adapter-airsim](https://github.com/hakoniwalab/hakoniwa-sensor-adapter-airsim).
- Date measured: 2026-11-19 (first measured on 2026-11-01). Machine: macOS (arm64).
- On 2026-11-19 the Hakoniwa outputs of axes 0, 1 and 3 were bit-identical to those of 2026-11-01, measured with the Drone PRO sources: the release archive computes the same as Drone PRO. The one change since then is a Gazebo sample-time case of axis 0, from an update of the Gazebo adapter ([axis 0](axis0-no-noise.md)).
