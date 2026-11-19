# Sensor comparisons

The programs behind [Drone sensors: evaluation results](../../docs/evaluation/drone/README.md) and
[Force-torque sensor: evaluation results](../../docs/evaluation/force-torque/README.md): the Hakoniwa, Gazebo
and Project AirSim sensors behind the same interface (`IImuSensorAdapter` and others), fed the same truth, with the
outputs lined up. The axes are defined in [Evaluation axes](../../docs/evaluation/drone/evaluation-axes.md).

| Program | Axis |
|---|---|
| `axis0_no_noise.cpp` | 0. No-noise agreement (`plot_axis0.py` draws its figure) |
| `axis1_accuracy.cpp` | 1. Accuracy |
| `axis3_repeatability.cpp` | 3. Repeatability |
| `axis4_speed.cpp` | 4. Speed (one implementation per process) |
| `force_torque.cpp` | The force-torque sensor, axes 0, 1, 3 and 4 (Hakoniwa and Gazebo; Project AirSim has no force-torque sensor) |
| `ft_series.cpp`, `cards_force_torque.py` | Time series of one force-torque sensor (`ft-series`) and the measurement cards of the force-torque page drawn from them (`python3 cards_force_torque.py --tool build/ft-series --out ../../docs/evaluation/force-torque --lang en`) |

Axes 2 (coverage) and 5 (adoption effort) are assessments, not programs.

## The three implementations

- **Hakoniwa:** the `hakoniwa` implementation of a [hakoniwa-sensor-pro](https://github.com/hakoniwalab/hakoniwa-sensor-pro)
  release archive (binaries; free for non-commercial use). The programs reach it by name through the common interface
  (`hakoniwa_entry.hpp`, `sensor_adapter_implementation.hpp`) and link the archive's static libraries.
- **Gazebo:** [hakoniwa-sensor-adapter-gazebo](https://github.com/hakoniwalab/hakoniwa-sensor-adapter-gazebo) on
  gz-sensors 10.2.0.
- **Project AirSim:** [hakoniwa-sensor-adapter-airsim](https://github.com/hakoniwalab/hakoniwa-sensor-adapter-airsim),
  the core_sim sensors compiled as is from the sources its lock file pins.

## Build and run

Check out the two adapters next to this repository:

```text
<workspace>/hakoniwa-sensor-adapter          this repository
<workspace>/hakoniwa-sensor-adapter-gazebo
<workspace>/hakoniwa-sensor-adapter-airsim
```

Extract a hakoniwa-sensor-pro release archive and check its SHA-256 against the release. The environment
(gz-sensors and the libraries core_sim pins) comes from [pixi](https://pixi.sh) and `pixi.toml` in this directory.
The `configure` task fetches the Project AirSim sources first.

```bash
cd tools/compare
export HAKONIWA_SENSOR_PRO_DIR=/path/to/hakoniwa-sensor-pro-0.1.0-macos-arm64
pixi run build
pixi run axis0
pixi run axis1
pixi run axis3
pixi run axis4
pixi run force-torque
```

Each task writes its CSV files under `results/<axis>/` and prints a summary. Axis 4 measures time, so run it on an idle
machine and repeat it; the published page gives the range of three runs.

Without pixi, configure with CMake directly, with gz-sensors 10.2.0, gz-math, Boost headers, nng 1.11, msgpack-cxx,
Eigen and nlohmann/json found through `CMAKE_PREFIX_PATH`:

```bash
cmake -S tools/compare -B build-compare -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=$CONDA_PREFIX -DHAKONIWA_SENSOR_PRO_DIR=$HAKONIWA_SENSOR_PRO_DIR -DCOMPARE_WITH_AIRSIM=ON
cmake --build build-compare
build-compare/compare_axis1_accuracy results/axis1
```

`COMPARE_WITH_AIRSIM=OFF` leaves Project AirSim out. `HAKONIWA_SENSOR_ADAPTER_GAZEBO_ROOT` and
`HAKONIWA_SENSOR_ADAPTER_AIRSIM_ROOT` point to the adapter checkouts when they are elsewhere.
