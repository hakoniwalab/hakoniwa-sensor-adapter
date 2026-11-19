# Error characteristics

One sensor error at a time, without a physics engine. A rigid body moves along a motion written as a formula; the
host helpers of this repository (`host/sensor_host.hpp`) apply the mounting pose and build the sensor truth; the truth
goes through an implementation's adapter, chosen by name through the common interface
(`sensor_adapter_implementation.hpp`).

The point is what the drone sensors share: the same processing order, the same random streams, the same drift and
noise-density formulas ([drone-common.md](../../docs/spec/drone-common.md)). The results are on the
[error characteristics page](../../docs/evaluation/error-characteristics/README.md).

## What is here

| Path | Content |
|---|---|
| `error_characteristics.cpp` | the tool: one sensor, one configuration, one motion → a CSV of truth and measurement |
| `config/{imu,mag,baro,gps}/` | configurations that each turn on one error (illustrative values within the specification's ranges, not those of a particular chip) |
| `cards_mount.py` | measurement cards of the mounting section |
| `cards_offset_noise.py` | measurement cards of the per-unit offset, iron and noise sections |
| `cards_drift_timing_range.py` | measurement cards of the drift, period-and-delay and resolution-and-range sections |
| `cards_common.py`, `ec_common.py` | shared by the card scripts: the card layout, running the tool, the Allan deviation |
| `concepts/` | the concept figures, one per stage of the processing order; illustrative data made with numpy, not measured |

A measurement card is one measurement: on the left a sketch of how the sensor was mounted and moved, with the
configured values; on the right the result, with the axes, the expected values and the measured numbers on the plot.
Every card and concept figure is drawn in English and in Japanese (`-en.png`, `-ja.png`).

The world is fixed and illustrative: Tokyo, a 46000 nT field (declination −7.5°, inclination 49.5°), the standard
atmosphere.

## Build and run

The Hakoniwa implementation is the `hakoniwa` entry of a
[hakoniwa-sensor-pro](https://github.com/hakoniwalab/hakoniwa-sensor-pro) release archive (binaries; free for
non-commercial use). The IMU and GNSS parts need no MuJoCo.

```bash
cmake -S tools/error-characteristics -B build-errors -DCMAKE_BUILD_TYPE=Release \
  -DERROR_CHARACTERISTICS_IMPLEMENTATIONS=hakoniwa \
  "-DERROR_CHARACTERISTICS_LIBRARIES=$SENSOR_PRO/lib/libhakoniwa_mujoco_sensors_adapters.a;$SENSOR_PRO/lib/libhakoniwa_imu_gnss_sensor_adapters.a;$SENSOR_PRO/lib/libhakoniwa_imu_gnss_sensor_models.a"
cmake --build build-errors
```

`SENSOR_PRO` is the extracted archive, for example `hakoniwa-sensor-pro-0.1.0-macos-arm64`.

The cards and the concept figures, in both languages, into the documentation folder:

```bash
for lang in en ja; do
  for cards in cards_mount cards_offset_noise cards_drift_timing_range; do
    python3 tools/error-characteristics/$cards.py --tool build-errors/error-characteristics \
      --out docs/evaluation/error-characteristics --lang $lang
  done
done
python3 tools/error-characteristics/concepts/make_all.py --out docs/evaluation/error-characteristics
```

The scripts need numpy and matplotlib, and the Japanese text a Japanese font (Hiragino Sans on macOS). The tool's CSV
files go to a temporary directory (`--work` to choose one). `--only <card>` draws single cards by function name. All
cards in one language take a few minutes; most of it is the 1000-unit offsets and the 600 runs of the integrated
angle.

The tool on its own:

```bash
build-errors/error-characteristics --implementation hakoniwa --sensor imu --config tools/error-characteristics/config/imu/noise.json \
  --motion rest --duration 10 --out noise.csv [--seed N] [--mount x,y,z,roll,pitch,yaw] [--rate R] [--t0 S] [--every N]
```

`--sensor` is `imu`, `mag`, `baro` or `gps`; `--motion` is `rest`, `turn`, `step`, `ramp`, `yawstep`, `hstep`, `climb`
or `move` (see the head of `error_characteristics.cpp`); `--mount` is in body FLU axes (x forward, y left, z up). Each
CSV has `time`, the truth (`true_*`, sensor axes for the IMU and the magnetometer, NWU for GPS) and the measurement
with its `updated` flag and `flags` (bit 0: saturated).
