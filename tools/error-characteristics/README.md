# Error characteristics

One sensor error at a time, without a physics engine. A rigid body moves along a motion written as a formula; the
host helpers of this repository (`host/sensor_host.hpp`) apply the mounting pose and build the sensor truth; the truth
goes through an implementation's adapter, chosen by name through the common interface
(`sensor_adapter_implementation.hpp`). The plots lay each error over the truth or over the value the configuration
asks for.

The point is what the drone sensors share: the same processing order
(truth → mounting → delay → sample → noise → drift → saturation, [drone-common.md](../../docs/spec/drone-common.md)),
the same random streams, the same drift and noise-density formulas. Each figure turns on one of them.

## IMU figures

| Figure | Configuration (`config/imu/`) | Motion | Shows |
|---|---|---|---|
| `imu-1-mount` | `truth.json` | turn at 180 deg/s; a step about body x | the lever arm (−ω²r on the accelerometer) and the axes of a turned mounting |
| `imu-2-noise` | `noise.json`, `noise-density-{2,4,8}ms.json` | at rest | the spread against the configured σ; σ = density / √period |
| `imu-3-offset` | `offset.json` | at rest, 1000 seeds | one zero-rate / zero-g offset per unit, spread as configured |
| `imu-4-drift` | `drift.json`, `noise-drift.json` | at rest, 30 min | Gauss-Markov drift (σ 0.005 deg/s, τ 100 s); the Allan deviation of noise plus drift |
| `imu-5-delay-period` | `period.json`, `period-delay.json` | a step between two samples | the value held for the update period; the same samples, delayed |
| `imu-6-resolution-range` | `resolution-range.json` | slow and fast ramps | 0.061 deg/s steps; clipping at 2000 deg/s and the saturated flag |

The values are illustrative settings within the ranges the specification gives, not those of a particular chip.

## Build and run

The Hakoniwa implementation is the `hakoniwa` entry of a
[hakoniwa-sensor-pro](https://github.com/hakoniwalab/hakoniwa-sensor-pro) release archive (binaries; free for
non-commercial use). The IMU and GNSS parts need no MuJoCo.

```bash
cmake -S tools/error-characteristics -B build-errors -DCMAKE_BUILD_TYPE=Release \
  -DERROR_CHARACTERISTICS_IMPLEMENTATIONS=hakoniwa \
  "-DERROR_CHARACTERISTICS_LIBRARIES=$SENSOR_PRO/lib/libhakoniwa_mujoco_sensors_adapters.a;$SENSOR_PRO/lib/libhakoniwa_imu_gnss_sensor_adapters.a;$SENSOR_PRO/lib/libhakoniwa_imu_gnss_sensor_models.a"
cmake --build build-errors
python3 tools/error-characteristics/imu_figures.py --tool build-errors/error-characteristics --out errors-out
```

`SENSOR_PRO` is the extracted archive, for example `hakoniwa-sensor-pro-0.1.0-macos-arm64`. The figures need numpy and
matplotlib; all six take about half a minute.

The tool on its own:

```bash
build-errors/error-characteristics --implementation hakoniwa --sensor imu --config tools/error-characteristics/config/imu/noise.json \
  --motion rest --duration 10 --out noise.csv [--seed N] [--mount x,y,z,roll,pitch,yaw] [--rate R] [--t0 S] [--every N]
```

Columns: `time`, the truth (`true_gx` … `true_az`, sensor axes, specific force), and per part `*_updated`, the value
and the flags (bit 0: saturated).
