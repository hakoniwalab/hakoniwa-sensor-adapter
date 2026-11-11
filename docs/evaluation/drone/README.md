# Drone sensors: evaluation results (2026-11-01)

These are the results for the six axes in [Evaluation axes](evaluation-axes.md). We compared the Hakoniwa Drone PRO sensors (sensor v2), the Gazebo sensors (gz-sensors 10.2.0), and the Project AirSim sensors (core_sim compiled as is), through the same interface and with the same truth.

We ran them on macOS (arm64). We do not set pass/fail criteria. We list only the numbers and the reasons for the differences.

| Axis | Hakoniwa | Gazebo | Project AirSim (core_sim) | Details |
|---|---|---|---|---|
| 0. No-noise agreement | Reference | Gyro, accelerometer, magnetometer, barometer (US 1976) and sample times agree. GPS maps onto the Earth differently | Gyro and accelerometer differ by 32-bit rounding; magnetometer and sample times agree. Barometer is within 0.09 Pa of US 1976; pressure altitude is off by up to 2.65 m. GPS treats the Earth as a sphere and outputs latitude and longitude as 32-bit values | [axis0-no-noise.md](axis0-no-noise.md) |
| 1. Accuracy | All 21 items as configured | Sign of the bias flipped with 50 % probability; magnetometer, barometer and GPS dynamic bias does not move; magnetometer resolution has no effect; GPS east noise shrinks | Noise, bias and random walk as configured. The seed is fixed, so every instance gives the same value sequence (the magnetometer turn-on bias is also the same every time). Barometer slowly varying bias is 1/10 of σ. GPS has no noise | [axis1-accuracy.md](axis1-accuracy.md) |
| 2. Coverage (18 items) | 18 | 4 (2 conditional) | 1 (8 conditional) | [axis2-coverage.md](axis2-coverage.md) |
| 3. Repeatability (5 cases) | Same in all 5 cases | Sequence changes with other sensors, threads and reset | Same in all 5 cases (because the seed is always the same; the host seed is not used) | [axis3-repeatability.md](axis3-repeatability.md) |
| 4. Speed (mean per physics step, 5 sensors) | 0.11 to 0.18 µs | 0.68 to 0.77 µs | 0.79 to 0.85 µs | [axis4-speed.md](axis4-speed.md) |
| 5. Adoption effort | No extra dependencies | 37 runtime libraries, about 29 MB, a transport thread | nng 1.11, msgpack-cxx, Eigen, nlohmann/json, and 140 files from a pinned commit. No transport port is opened (threads that only wait are started) | [axis5-adoption.md](axis5-adoption.md) |

## Method

- The comparison program calls the three adapters only through this interface (`IImuSensorAdapter` and others), passes the same truth sequence, and lines up the outputs.
- The Gazebo (hakoniwa-sensor-adapter-gazebo) and Project AirSim (hakoniwa-sensor-adapter-airsim) adapters are public, so anyone can reproduce those columns.
- The Hakoniwa columns were measured with the Hakoniwa Drone PRO sensor models. The comparison program and the Hakoniwa adapter are part of Hakoniwa Drone PRO (not public).
- Date measured: 2026-11-01. Machine: macOS (arm64).
