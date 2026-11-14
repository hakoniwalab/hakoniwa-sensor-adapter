# Sensor logs

A sensor host can log what each IMU, magnetometer, barometer and GPS adapter received and gave on every physics step:
the truth the host passed (already at the sensor's mount), the adapter's output, and the adapter's own log columns.
With the truth and the output side by side, a run can be explained sensor by sensor: how much noise, delay, drift or
saturation each sensor added. The log works the same with every implementation, because the host writes it around
the adapter.

The host part is `include/hakoniwa/sensor_adapter/host/sensor_log.hpp` (CMake target `hakoniwa_sensor_adapter_log`,
built when nlohmann/json is available). A host creates each adapter with `create_logged_<type>_sensor_adapter`,
passing the implementation's factory, in place of calling the factory itself.

Requirements are numbered `LOG-<part>-<3 digits>`: `CFG` the configuration, `COL` the columns, `FMT` the file formats.

## 1. Configuration

A sensor's configuration file is either the implementation's own, or a sensor log configuration:

```json
{
  "model": "imu-model.json",
  "log": {"path": "logs/imu0.bin", "format": "binary"}
}
```

| Key | Meaning |
| --- | --- |
| `model` | the implementation's configuration file for this sensor. A relative path is relative to this file |
| `log.path` | the log file. A relative path is relative to this file; missing directories are made |
| `log.format` | `csv` (default) or `binary` |

- **LOG-CFG-010** A configuration file without `model` is the implementation's own: it is passed to the implementation
  as it is, and nothing is logged.
- **LOG-CFG-020** With `model` and without `log`, the implementation gets the `model` file, and nothing is logged.
- **LOG-CFG-030** With `log`, the implementation gets the `model` file, and the host writes the log to `log.path` in
  `log.format`.
- **LOG-CFG-040** An unknown key, a format other than `csv` and `binary`, a `log` without `path`, and a file that can
  not be read are errors when the adapter is created.

Each sensor has its own configuration file, so two sensors of the same type write two logs.

## 2. Columns

- **LOG-COL-010** One row per physics step. The first column is `time_usec`, the simulation time of the step [µs].
- **LOG-COL-020** Then the truth, then the output, then the adapter's own columns (`log_columns()`), with their names
  unchanged.
- **LOG-COL-030** The output is the value the adapter holds after the step; `updated` is 1 on the steps it measured.

Axes are the sensor's (FLU), the world is NWU, and units are SI unless the name says otherwise.

| Sensor | Truth | Output |
| --- | --- | --- |
| IMU | `truth_angular_velocity_x/y/z` [rad/s], `truth_specific_force_x/y/z` [m/s²] | `gyro_x/y/z`, `accel_x/y/z`, `gyro_updated`, `accel_updated`, `gyro_flags`, `accel_flags` |
| Magnetometer | `truth_magnetic_field_x/y/z` [nT] | `magnetic_field_x/y/z` [nT], `updated`, `flags` |
| Barometer | `truth_altitude_m`, `truth_sea_level_pressure_pa`, `truth_sea_level_temperature_c` | `pressure_pa`, `temperature_c`, `pressure_altitude_m`, `updated`, `flags` |
| GPS | `truth_latitude_deg`, `truth_longitude_deg`, `truth_altitude_m`, `truth_position_nwu_x/y/z`, `truth_velocity_nwu_x/y/z` | `latitude_deg`, `longitude_deg`, `altitude_m`, `velocity_nwu_x/y/z`, `ground_speed_mps`, `course_deg`, `course_valid`, `horizontal_accuracy_m`, `vertical_accuracy_m`, `speed_accuracy_mps`, `satellites_visible`, `fix_type`, `updated`, `flags` |

What the adapter's own columns hold is the implementation's choice. An implementation that logs every stage of its
model (after the delay, the measured value, the noise, the drift, the saturation) lets each error be seen on its own.

## 3. File formats

- **LOG-FMT-010** CSV: one header line, then one line per step; values with 9 significant digits.
- **LOG-FMT-020** Binary: Hakoniwa Drone's binary log, HKBLOG version 1.
  - Header: `HKBLOG\0\0`; the version (1) and the flags (0) as two little-endian uint16; the column count as uint32;
    for each column its type (8: uint64 for `time_usec`, 10: double for the others), the name's length as uint16 and
    the name.
  - Row: its size in bytes as uint32, then the values, little-endian.
- **LOG-FMT-030** Rows reach the file at least every 256 steps, so a run stopped without its cleanup loses at most
  the last ones.

`tools/read_sensor_log.py` reads both formats and writes a binary log as CSV.
