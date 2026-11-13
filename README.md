# hakoniwa-sensor-adapter

hakoniwa-sensor-adapter is a ROS-independent C++ interface, plus a C plugin ABI, between a simulator host and implementations of sensor error models ("adapters"). The host passes the truth from its physics (and applies the mounting pose); the adapter returns what the sensor measures: noise, bias, drift, resolution, saturation, delay, sample period and so on. Frames follow the Hakoniwa convention: the world is NWU (x North, y West, z Up) and the body and sensor are FLU (x Forward, y Left, z Up). Values are in SI units. The host may pass a random seed, and each adapter reports a capability table that declares which sensor properties it supports, including whether the same seed gives the same value sequence (`deterministic_with_seed`). The library is header-only (C++17).

## Sensors and headers

All headers are in `include/hakoniwa/sensor_adapter/`.

| Sensor | Header | Main interface |
|---|---|---|
| Common base (setup, frames, flags, log columns) | `sensor_adapter.hpp` | `ISensorAdapter` |
| IMU (gyro and accelerometer) | `imu_sensor_adapter.hpp` | `IImuSensorAdapter` |
| Magnetometer | `mag_sensor_adapter.hpp` | `IMagSensorAdapter` |
| Barometer | `baro_sensor_adapter.hpp` | `IBaroSensorAdapter` |
| GPS | `gps_sensor_adapter.hpp` | `IGpsSensorAdapter` |
| Common types for range and image sensors (ray caster, renderer, sensor pose) | `range_image_sensor_adapter.hpp` | `IRayCaster`, `IImageRenderer` |
| Rangefinder | `rangefinder_sensor_adapter.hpp` | `IRangefinderSensorAdapter` |
| LiDAR (2D and 3D) | `lidar_sensor_adapter.hpp` | `ILidarSensorAdapter` |
| Depth camera and RGB camera | `camera_sensor_adapter.hpp` | `IDepthCameraSensorAdapter`, `IRgbCameraSensorAdapter` |
| 6-axis force-torque sensor | `force_torque_sensor_adapter.hpp` | `IForceTorqueSensorAdapter` |
| Joint sensors (encoder and joint torque) | `joint_sensor_adapter.hpp` | `IJointSensorAdapter` |
| Implementation descriptor (how a host obtains adapters from an implementation) | `sensor_adapter_implementation.hpp` | `SensorAdapterImplementation`, entry point `hako_sensor_adapter_implementation_<name>` |
| External sensor plugin (C ABI) | `external_sensor_plugin.h` | entry point `hako_external_sensor_plugin_v1` |

Each implementation publishes one `SensorAdapterImplementation`: its name and one creation function per sensor type it provides (`nullptr` for the others). The entry point name is fixed by the implementation name, so a host can register an implementation from its name alone, for example from a build option that lists the linked implementations.

The external sensor plugin ABI lets a host load range and image sensors as a shared library. Only C types cross the boundary, so the host and the plugin may be built with different compilers, C runtimes or build configurations.

## Implementations

- **Reference implementation** (in this repository, implementation name `reference`): returns the true values, nothing else. See [Reference implementation](#reference-implementation).
- **Hakoniwa sensor models.** Hakoniwa's own sensor error models, used in Hakoniwa products (for example Hakoniwa Drone PRO). Their sources are not public.
- **Gazebo gz-sensors adapter** (public): [hakoniwa-sensor-adapter-gazebo](https://github.com/hakoniwalab/hakoniwa-sensor-adapter-gazebo).
- **Project AirSim core_sim adapter** (public): [hakoniwa-sensor-adapter-airsim](https://github.com/hakoniwalab/hakoniwa-sensor-adapter-airsim).

## Reference implementation

`include/hakoniwa/sensor_adapter/reference/` holds an implementation that returns the true values and adds no error of any kind. It is for hosts that have no other implementation, as the ideal baseline to compare implementations against, and as an example of how an implementation meets this interface. It is not a model of a real sensor: noise, delays, scan timing and the other error models belong to the other implementations.

| Sensor | Returns | Refuses (names the key) |
|---|---|---|
| LiDAR | The true distance of each ray of an instant scan (`scan_mode: "instant"`), points with ring and object id; +inf when nothing is hit, -inf nearer than the minimum (REP 117) | `scan_mode` other than `instant` (including the default, rotating), `accuracy`, `range.resolution`, `no_return: "limit"` |
| Force-torque | The true wrench | `range` and every error stage (`overload`, `delay`, `lowpass_cutoff_hz`, `nonlinearity`, `scale_error`, `crosstalk`, `bias`, `drift_per_hour`, `noise`, `resolution`, `tare_window`); `tare()` returns false |
| Joint (encoder, joint torque) | The true position and velocity, and the joint torque as the effort (the transmitted torque, or the actuator torque when the host has none) | `delay`, `encoder`, `velocity`, `torque` |

Values are sampled at `update_rate` and held in between, as for every implementation. The capability tables declare every error model false. The LiDAR measures the world only through the host's `IRayCaster`.

It depends on this library and nlohmann/json (header-only). A host compiles `src/reference/reference_implementation.cpp` (or links the CMake target `hakoniwa_sensor_adapter_reference`, built when nlohmann/json is found as a package or given with `HAKONIWA_SENSOR_ADAPTER_NLOHMANN_JSON_INCLUDE_DIR`) and registers it by name: `HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(reference)()`.

Other implementations live in their own repositories; this one is here because it depends on nothing else and shows the interface in use.

## Comparison results

The implementations were compared through this interface with the same truth, on six axes: no-noise agreement, accuracy, coverage, repeatability, speed and adoption effort. No pass/fail criteria are set; the pages list the numbers and the reasons for the differences. See [docs/evaluation](docs/evaluation/README.md).

| Sensor | Coverage | Repeatability | Speed (mean per physics step) | Details |
|---|---|---|---|---|
| Drone sensors (IMU, magnetometer, barometer, GPS) | 18 items: Hakoniwa 18, Gazebo 4 (2 conditional), Project AirSim 1 (8 conditional) | 5 cases: Hakoniwa same in all 5; Gazebo changes with other sensors, threads and reset; Project AirSim same in all 5 (the seed is always the same) | Hakoniwa 0.11 to 0.18 µs, Gazebo 0.68 to 0.77 µs, Project AirSim 0.79 to 0.85 µs | [Drone sensors](docs/evaluation/drone/README.md) |
| 6-axis force-torque sensor | 15 items: Hakoniwa 15, Gazebo 4 | 5 cases: Hakoniwa as specified in all 5; Gazebo changes with other sensors, two units with the same seed, and reset | Hakoniwa 0.05 µs (no errors), 0.10 µs (with noise); Gazebo 0.11 µs, 0.22 µs | [Force-torque sensor](docs/evaluation/force-torque/README.md) |
| Joint sensors (encoder, joint torque) | 13 items: Hakoniwa 13, Gazebo 0, MuJoCo 2, mujoco_ros2_control 1, Webots 3 | Hakoniwa: same sequence with the same seed and after reset, a separate sequence per joint name | Hakoniwa 0.024 µs (STS3215 settings), 0.10 µs (every error) | [Joint sensors](docs/evaluation/joint/README.md) |

Measured on macOS (arm64). The Gazebo and Project AirSim columns can be reproduced with the public adapters.

## Documentation

| Directory | Contents |
|---|---|
| [docs/design](docs/design) | Interface design for each sensor family and surveys of other simulators and real devices |
| [docs/spec](docs/spec/README.md) | Sensor behaviour specification |
| [schemas](schemas/sensor-definition.schema.json) | JSON Schema of the sensor definition files (rangefinder, LiDAR, cameras, force-torque, joint); check files with `tools/check_sensor_definition.py` |
| [docs/evaluation](docs/evaluation/README.md) | Comparison results |

## Build and contract tests

The library is an INTERFACE target (`hakoniwa_sensor_adapter`) that needs C++17. Contract tests for each interface, including a fake C plugin loaded by a C++ host, are built when `HAKONIWA_SENSOR_ADAPTER_BUILD_TESTS` is `ON`; the reference implementation's test is built when nlohmann/json is available (for example `-DHAKONIWA_SENSOR_ADAPTER_NLOHMANN_JSON_INCLUDE_DIR=<dir with nlohmann/json.hpp>`).

```bash
cmake -S . -B build -DHAKONIWA_SENSOR_ADAPTER_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build
```

## License

MIT License. See [LICENSE](LICENSE).
