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

## Magnetometer, barometer and GPS figures

The same kinds of figure for the other drone sensors (`mag_baro_gps_figures.py`, configurations in `config/mag`,
`config/baro`, `config/gps`). The world is fixed and illustrative: Tokyo, a 46000 nT field (declination −7.5°,
inclination 49.5°), the standard atmosphere.

| Figure | Shows |
|---|---|
| `mag-1-mount-iron` | one turn: a turned mounting shifts the phase; hard and soft iron shift and squash the circle |
| `mag-2-noise-drift` | the noise against σ 30 nT; Gauss-Markov drift (σ 100 nT, τ 60 s) of three units |
| `mag-3-delay-resolution` | a quarter turn at once, held for the 20 ms period and 50 ms late; 15 nT steps; a 0.3 G range clipping z |
| `baro-1-mount-offset` | a sensor 0.5 m higher reads about 6 Pa less; one zero offset per unit (σ 4 m, 1000 units) |
| `baro-2-noise-drift` | the noise against σ 0.1 m; the Allan deviation of noise plus drift (0.3 m, τ 30 s) |
| `baro-3-delay-resolution` | a 10 m step, held and 60 ms late; 0.5 Pa (about 4 cm) steps; the 300 hPa range |
| `gps-1-mount-noise` | an antenna 0.5 m forward draws a circle while the vehicle turns in place; the position noise against σ 0.3 m |
| `gps-2-drift` | the fix wandering with a Gauss-Markov drift (0.5 m, τ 60 s); the Allan deviation of north |
| `gps-3-delay` | moving off at 5 m/s: 10 fixes a second, and the same fixes 100 ms late |

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
python3 tools/error-characteristics/mag_baro_gps_figures.py --tool build-errors/error-characteristics --out errors-out
```

`SENSOR_PRO` is the extracted archive, for example `hakoniwa-sensor-pro-0.1.0-macos-arm64`. The figures need numpy and
matplotlib; each script takes about half a minute.

The tool on its own:

```bash
build-errors/error-characteristics --implementation hakoniwa --sensor imu --config tools/error-characteristics/config/imu/noise.json \
  --motion rest --duration 10 --out noise.csv [--seed N] [--mount x,y,z,roll,pitch,yaw] [--rate R] [--t0 S] [--every N]
```

`--sensor` is `imu`, `mag`, `baro` or `gps`; `--motion` is `rest`, `turn`, `step`, `ramp`, `yawstep`, `hstep`, `climb`
or `move` (see the head of `error_characteristics.cpp`). Each CSV has `time`, the truth (`true_*`, sensor axes for the
IMU and the magnetometer, NWU for GPS) and the measurement with its `updated` flag and `flags` (bit 0: saturated).
