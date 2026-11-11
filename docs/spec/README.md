# Sensor behaviour specification

This specification defines the normative behaviour of the sensors behind the public interfaces of this repository.
The interfaces themselves (headers under `include/hakoniwa/sensor_adapter/`) define the types and calls.
This specification defines what an adapter behind those interfaces must do: what it measures, its error model, its configuration, and its outputs.

Implementations may differ internally. They must produce the behaviours written here.

The design rationale is in the design documents:

- [Range and image sensor adapter](../design/range-image-adapter.md)
- [Force-torque sensor adapter](../design/force-torque-adapter.md)
- [Joint sensor adapter](../design/joint-sensor-adapter.md)
- [IMU adapter](../design/imu-adapter.md)
- [Magnetometer, barometer and GPS adapters](../design/mag-baro-gps-adapter.md)
- [External sensor plugin](../design/external-sensor-plugin.md)

## Index

| Sensor | File | IDs |
|---|---|---|
| Common rules for range and image sensors | this file | `RIS-COM-*` |
| Rangefinder | [rangefinder.md](rangefinder.md) | `RIS-RNG-*` |
| LiDAR (2D and 3D) | [lidar.md](lidar.md) | `RIS-LID-*` |
| Depth camera | [depth-camera.md](depth-camera.md) | `RIS-DEP-*` |
| RGB camera | [rgb-camera.md](rgb-camera.md) | `RIS-RGB-*` |
| Force-torque sensor | [force-torque.md](force-torque.md) | `FTS-ADP-*` |
| Joint sensor (encoder and joint torque) | [joint.md](joint.md) | `JNT-ADP-*` |

The drone sensors (gyro, accelerometer, magnetometer, barometer, GPS, vibration) are specified in the public hakoniwa-drone-core repository:
<https://github.com/toppers/hakoniwa-drone-core/tree/main/docs/architecture/aircraft/sensor> (`SEN-*` IDs).

## 1. Requirement IDs

Each requirement has a stable ID. IDs never change meaning and are never reused, so implementations and tests can cite them.
IDs are not consecutive: gaps are intentional.

| Prefix | Meaning |
|---|---|
| `RIS-<part>-<nnn>` | Range and image sensors |
| `FTS-ADP-<nnn>` | Force-torque sensor adapter. Separate from `RIS` because the host supplies the truth force directly |
| `JNT-ADP-<nnn>` | Joint sensor adapter |

Parts of `RIS`:

| Part | Meaning |
|---|---|
| `COM` | Common to all range and image sensors |
| `RNG` | Rangefinder |
| `LID` | LiDAR (2D and 3D) |
| `DEP` | Depth camera |
| `RGB` | RGB camera |

The rest of this file is the common rules (`RIS-COM-*`). They apply to the rangefinder, LiDAR, depth camera and RGB camera.

## 2. Units and axes

| ID | Requirement |
|---|---|
| RIS-COM-010 | Distances and positions are in m, angles in rad, times in ns. Datasheet values in millimetres or % are converted when the configuration is read (configurations are written in m and rad) |
| RIS-COM-020 | Sensor axes are FLU (x forward, y left, z up). The ray origins and directions the adapter passes to the host are in sensor axes. Point-cloud points are also output in sensor axes |
| RIS-COM-030 | The adapter does not know the physics engine. It measures the world only through the host's ray caster (`IRayCaster`) and image renderer (`IImageRenderer`), passed at creation. The adapter depends on no physics-engine or graphics API |
| RIS-COM-040 | The adapter declares which of the two it needs in its capability table (`requires_ray_caster`, `requires_renderer`). If the host does not pass a required one, creation fails (nullptr and an error string) |

## 3. Time and sample period

| ID | Requirement |
|---|---|
| RIS-COM-050 | `step` times are 0, dt, 2dt, … (dt = `physics_step_ns`). Any other time is an error (false and an error string) and does not advance the internal state |
| RIS-COM-060 | The sample period is the period of `update_rate` [Hz] rounded to a whole number of physics steps (round half up, at least 1). If the period is not an exact multiple of the physics step, a warning is written to standard error and the adapter continues (same rule as the drone sensors) |
| RIS-COM-070 | Starting at time 0, the adapter samples on the physics steps whose index is a multiple of the period (LiDAR rotation follows the [LiDAR specification](lidar.md)) |
| RIS-COM-080 | On steps without a sample, the adapter returns `updated = false` and the last sampled value unchanged (output arrays stay valid until the next sample) |

## 4. Random numbers and reset

| ID | Requirement |
|---|---|
| RIS-COM-090 | The random seed is derived from the host seed, the vehicle name and index, the sensor name, and the characteristic name (FNV-1a, 32 bit; the same derivation as the drone sensors). If the host passes no seed, each run uses a different seed |
| RIS-COM-100 | The same seed, configuration and inputs give the same sequence of values. Instances do not share random state |
| RIS-COM-110 | `reset` returns the time, the period phase, any in-progress scan state, and the random streams (including a cached Gaussian value) to their state at creation. The next `step` starts at time 0 |

## 5. Range error model

This model is shared by the rangefinder, LiDAR and depth camera. The "true range" is the value defined per sensor (for the rangefinder, the distance projected on the sensor axis; for the depth camera, the planar or along-ray distance).

| ID | Requirement |
|---|---|
| RIS-COM-120 | Out-of-range handling (`no_return`). `inf` (default): no hit or farther than the maximum gives +inf; nearer than the minimum gives −inf (REP 117). `limit`: the far side gives the maximum, the near side gives the minimum |
| RIS-COM-130 | Accuracy (`accuracy`) is written per range band, as either σ [m] (`stddev`) or a percentage of the range (`percent`). Both or neither is an error. The first band that contains the true range is used. If no band contains it, no noise is added |
| RIS-COM-140 | Noise is added only to true ranges inside the range, as a Gaussian with mean 0 and standard deviation σ |
| RIS-COM-150 | Resolution (`range.resolution` > 0): after noise, the value is rounded to the nearest multiple of the resolution. 0 means no rounding |
| RIS-COM-160 | After rounding, the value is clamped to [minimum, maximum]. If clamped, `kSensorFlagSaturated` is set (for LiDAR and the depth camera, if any value is clamped) |
| RIS-COM-170 | The range must satisfy 0 ≤ minimum < maximum. Resolution must be 0 or more. Violations are errors |

## 6. Configuration and errors

| ID | Requirement |
|---|---|
| RIS-COM-180 | The configuration is one JSON file. If `type` does not match the sensor kind, a required item is missing, or a value is out of range, creation fails (nullptr and an error string). No exception crosses the interface |
| RIS-COM-190 | The log columns are fixed at creation and do not change |

## 7. Pinhole camera (depth camera and RGB camera)

| ID | Requirement |
|---|---|
| RIS-COM-200 | Pinhole model with square pixels. fx = fy = (width / 2) / tan(horizontal FOV / 2), cx = (width − 1) / 2, cy = (height − 1) / 2 (pixel centres are at integer coordinates). The horizontal FOV is greater than 0 and less than π |
| RIS-COM-210 | The ray of pixel (u, v) is (xn, yn, 1) in optical axes (xn = (u − cx) / fx, yn = (v − cy) / fy), and (1, −xn, −yn) normalised to unit length in sensor axes. Image rows go from the top, columns from the left |
| RIS-COM-220 | `CameraInfo`: K = [fx 0 cx; 0 fy cy; 0 0 1], R = identity, P = [K \| 0], no distortion (empty model name). The rotation from sensor axes to optical axes is the value fixed by the interface |
