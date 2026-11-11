# Magnetometer, barometer, and GPS sensor adapter interfaces (design)

This document defines the magnetometer, barometer, and GPS interfaces, with the same approach as the IMU interface ([imu-adapter.md](imu-adapter.md)).
With these, all 5 sensors of Hakoniwa Drone PRO (accelerometer, gyroscope, magnetometer, barometer, GPS) can be replaced by external adapters.

## 1. Common part

What the 5 sensors have in common is collected in `include/hakoniwa/sensor_adapter/sensor_adapter.hpp`.

| Name | Content |
|---|---|
| `SensorAdapterSetup` | What the host passes at creation: the adapter's own configuration file, vehicle name and index, sensor name, physics step, random seed |
| `SensorFlag` | Flags attached to values. Only `kSensorFlagSaturated` (saturated) is defined |
| `LogColumn` | Name and unit of a log column |
| `ISensorAdapter` | What every sensor adapter has: `reset`, `log_columns`, `log_values` |

Each sensor's adapter (`IImuSensorAdapter`, `IMagSensorAdapter`, `IBaroSensorAdapter`, `IGpsSensorAdapter`) inherits `ISensorAdapter` and adds `step` and `capabilities`.

The common contract is the same as for the IMU.

- Frames follow the Hakoniwa convention. The world is NWU (x north, y west, z up), and the vehicle and the sensor are FLU (x forward, y left, z up).

- One instance is one sensor on one vehicle. It is called from only one thread.
- `step` is called every physics step, at time 0, dt, 2dt, .... If time does not advance by the step, it is an error.
- Values are measured at the adapter's own period boundaries, and the previous value is returned in between. Each value carries whether it was newly measured (`updated`) and the measurement time.
- The error model belongs entirely to the adapter. The configuration is written in the adapter's own format.
- Features an adapter does not have are declared `false` in the capability table.

## 2. Magnetometer (`IMagSensorAdapter`)

| Truth (`MagTruth`) | Frame | Unit |
|---|---|---|
| Magnetic field | Sensor axes | nT |
| World magnetic field, rotation from world to sensor | World NWU, quaternion | nT |

- The host creates the world magnetic field from the geomagnetic field at the simulation location (intensity, declination, inclination), rotates it to the sensor axes with the mounting orientation, and passes it.
- The world magnetic field and the rotation are auxiliary values. They are passed for implementations that rotate the world field themselves.
- The output (`MagSample`) is the measured magnetic field (nT, sensor axes). Conversion to the autopilot's unit (gauss or other) is done on the host side.
- Capability table: delay, noise, hard iron, soft iron, drift, resolution, saturation.

## 3. Barometer (`IBaroSensorAdapter`)

| Truth (`BaroTruth`) | Unit |
|---|---|
| Altitude of the sensor location (above mean sea level, geometric altitude) | m |
| Sea-level pressure, sea-level temperature (weather) | Pa, °C |

- The atmosphere model (the formulas that give pressure and temperature from altitude) belongs to the adapter as part of the sensor model. The host passes only the altitude and the weather.
- The output (`BaroSample`) is the measured pressure (Pa), the temperature at the sensor location (°C), and the altitude derived from the measured pressure (pressure altitude, m).
- An implementation that does not use weather (one that uses only a fixed standard atmosphere) sets `sea_level_weather` in the capability table to `false`. An implementation that does not output temperature or pressure altitude sets `temperature` and `pressure_altitude` to `false` and sets those values to 0.
- Capability table: weather, temperature, pressure altitude, delay, noise, bias, drift, resolution, saturation.

## 4. GPS (`IGpsSensorAdapter`)

At creation, the origin of the local NWU frame (latitude, longitude, and altitude of the simulation location) is passed in `GpsAdapterSetup::origin`.

| Truth (`GpsTruth`) | Frame | Unit |
|---|---|---|
| Antenna position | NWU from the origin | m |
| Antenna velocity | NWU | m/s |
| The same antenna position (latitude, longitude, altitude) | WGS-84 (height above the ellipsoid, no geoid) | deg, m |

- The host passes the position and velocity after the mounting position (the antenna position) has been applied.
- The same position is passed both in NWU and in latitude and longitude. The host does the conversion in one place, so every implementation can start from the same truth. This is because some implementations add errors in local coordinates and others in latitude and longitude.
- The output (`GpsSample`) is as follows.
  - Measured position (latitude, longitude, altitude), velocity (NWU)
  - Ground speed, course (0 to 360 degrees from north) and its validity flag
  - Accuracy estimates reported by the receiver (horizontal, vertical, velocity), number of visible satellites, fix type (0: none, 2: 2D, 3: 3D)
- Capability table: delay, position noise, velocity noise, drift, course, receiver status (accuracy, number of satellites, fix type).

## 5. Relation to the IMU interface

- The IMU interface structs and functions can be used as they are. `ImuAdapterSetup` inherits `SensorAdapterSetup`, with the default sensor name set to `imu`.
- `IImuSensorAdapter` also inherits `ISensorAdapter`. `reset`, `log_columns`, and `log_values` are declared in `ISensorAdapter`.
- `kImuFlagSaturated` is the same bit as the common `kSensorFlagSaturated`.

`test/mag_baro_gps_adapter_contract.cpp` contains reference adapters with no error and tests of how the host calls them.
