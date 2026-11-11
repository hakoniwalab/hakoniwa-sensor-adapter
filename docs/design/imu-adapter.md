# IMU sensor adapter interface (design)

This document defines the contract for exchanging IMU (gyroscope and accelerometer) values between Hakoniwa Drone PRO (the host) and an external sensor implementation (the adapter).
The adapter is built as a shared library (.so / .dylib / .dll), and the host loads it at run time.

This interface is defined **to fit the needs of Hakoniwa Drone PRO**. External implementations (such as Gazebo gz-sensors) are wrapped inside the adapter to fit this interface. Features an adapter does not have are declared in the capability table.

## 1. Purpose and scope

- The host passes the truth, and the adapter returns "the value the sensor measured".
- The adapter owns the sensor error model.
- Physics computation, applying the mounting pose, passing values to the autopilot, and communication are outside this interface. The host owns them.
- Only the IMU is defined first. Magnetometer, barometer, and GPS are added in the same form after the IMU interface is settled.

## 2. Host and adapter responsibilities

The sensor processing order of Hakoniwa Drone PRO is split between the host and the adapter as follows.

```text
Host: truth (physics) → mounting pose (position → orientation) → add vibration (optional)
                                                         │
Adapter:                                                 ▼
        filter (optional) → delay → sample → scale, crosstalk, bias → noise → drift → resolution → saturation → output
                                                         │
Host:                                                    ▼
        software delay → autopilot, log
```

| Owner | Content | Reason |
|---|---|---|
| Host | Physics truth, mounting pose (rigid-body formulas and rotation to the sensor axes), rotor-induced vibration | To give every implementation the same input. Differences in a comparison then come only from the error models |
| Adapter | IMU filter, delay, sample period, scale error, crosstalk, bias, noise, drift, resolution, saturation | This is the sensor error model itself. Implementations may differ here |
| Host | Software delay (from when the autopilot receives a value until it uses it) | Outside the sensor |

Vibration is treated as physical shaking at the sensor location. The host adds it to the truth before passing it.

## 3. Decisions (principles)

### 3.1 A C++ abstract class (the same form as Control Link)

- The interface is the C++ abstract class `IImuSensorAdapter` (`include/hakoniwa/sensor_adapter/imu_sensor_adapter.hpp`).
- Values are exchanged as structs with default values (`ImuTruth`, `ImuSample`, `ImuCalibration`, and others).
- Failures are returned as a return value (false) and an error string. No exceptions are thrown.
- The mechanism for loading a shared library at run time (plugin table and version check) is defined separately, in the same form as Control Link. It assumes that the host and the adapter are built with the same compiler and the same C++ standard library.

### 3.2 Truth: values at the sensor location, in sensor axes

The host passes the truth every physics step, after the mounting pose has been applied.

| Value | Frame | Unit |
|---|---|---|
| Angular velocity | Sensor axes | rad/s |
| Specific force (the value an ideal accelerometer measures; includes gravity) | Sensor axes | m/s² |
| Sensor orientation (rotation seen from world NWU) | Quaternion | — |
| Gravitational acceleration | World NWU | m/s² |

- The quantity that Hakoniwa Drone PRO uses internally (specific force) is passed as is.
- Orientation and gravity are auxiliary values. They are passed for implementations that take acceleration without gravity as input instead of specific force (a = f + R⁻¹·g).
- By Hakoniwa convention, the world is passed in NWU (x north, y west, z up), and the vehicle and the sensor in FLU (x forward, y left, z up). At rest and level, the specific force is +g on the sensor z axis.
- The host converts from the frames inside Hakoniwa Drone PRO (NED, FRD) at the interface boundary.

### 3.3 Gyroscope and accelerometer are sampled separately

In Hakoniwa Drone PRO, the gyroscope and the accelerometer can have separate periods, delays, and filters.

- The adapter returns gyroscope values and accelerometer values separately.
- Each carries a flag "a new value was produced at this step" and the measurement time.

### 3.4 The host decides the time

- The host passes the truth every physics step with the simulation time (ns, integer). The step is constant at the value given at creation.
- The sample period boundaries are time 0, T, 2T, .... In between, the adapter keeps returning the previous measured value.
- If time does not advance by the step (it goes back or jumps), it is an error. To restart, call `reset` and then pass values from time 0.

### 3.5 Configuration: the adapter reads only its own format

This follows the same approach as Control Link.

- The host does not pass its own configuration (drone_config) to the adapter. It passes only the location of the adapter's own configuration file.
- Error model values (noise, bias, period, and so on) are written in the adapter's format. For the Gazebo adapter, this is SDFormat `<sensor type="imu">`.
- **Hakoniwa Drone PRO has a tool (Python) that creates the adapter configuration from drone_config.** This is the same approach used to create the PX4 and ArduPilot parameters from drone_config.
  - Items with the same meaning are converted with the formulas in "Mapping to SDFormat" in the Hakoniwa sensor specification (for example: drift σ → `dynamic_bias_stddev = σ·√(2/τ)`, period in ms → `update_rate` in Hz).
  - Items that cannot be converted (saturation, delay, scale error, and so on) are not dropped silently. They are listed and reported. If needed, the tool stops.
  - This way, both the Hakoniwa model and an external model can be run from the same drone_config and compared.

Because the interface side does not know the content of the configuration, many implementations can be connected without changing the interface.

### 3.6 Random numbers and reset

- At creation, the host passes the following:
  - The random seed (`simulation.randomSeed`; "no seed" if absent)
  - The vehicle name and index
  - The sensor name
- The Hakoniwa model creates a separate random stream for each characteristic from these. The adapter receives the same inputs.
- `reset` returns all internal state to its state at creation. This covers the delay history, the measured values and period boundaries, drift, the bias drawn at start-up, and the random streams.
- Whether the same seed and the same input give the same sequence of values is declared in the capability table.

### 3.7 Values for autopilot calibration

Hakoniwa Drone PRO writes out the accelerometer bias and scale error as autopilot calibration values (PX4 `CAL_ACC0_*`, ArduPilot `INS_ACCOFFS_*` and `INS_ACCSCAL_*`).

- After every `reset`, the adapter can return the calibration values it uses (bias, scale error).
- An adapter that cannot return them declares this in the capability table. In that case, the host does not write out calibration values.

### 3.8 Value flags

- Gyroscope and accelerometer values (`ImuChannel`) each carry `flags`.
- The only bit defined now is "saturated" (`kImuFlagSaturated`). It is set when any axis of the measured value was clipped at the end of the measurable range.
- An implementation without saturation does not set the bit and sets `range_clamp` in the capability table to `false`. The host uses the capability table to distinguish "not saturated" from "not checked".
- Undefined bits are 0. Bits can be added later (the interface version is not raised).
- How the host uses the flags (writing them to the log, telling the autopilot, and so on) is defined separately in the Hakoniwa Drone PRO specification.

### 3.9 Values for logging (getters)

Values inside the sensor (current drift, the bias drawn at start-up, the value right after the filter, and so on) differ between implementations.
So that the host can log them without knowing their content, the interface has **getters that read column names and values**.

- `log_columns()`: returns the names and units of the log columns. They do not change after the instance is created.
- `log_values()`: returns the value (double) of each column after the last `step()`, in the same order as the columns.
- The adapter decides the column names (for example `gyro_bias_x`, `accel_drift_z`). The host uses the names as they are for CSV columns and similar.
- An implementation without log columns returns empty columns.
- These only read; they do not change the adapter state. Not reading them does not change the sensor values.

### 3.10 Instances and threads

- One instance is one IMU on one vehicle.
- One instance is called from only one thread.
- Whether different instances may be called from different threads is declared in the capability table.

## 4. Interface shape

`include/hakoniwa/sensor_adapter/imu_sensor_adapter.hpp` is the authoritative definition.

| Name | Content |
|---|---|
| `ImuAdapterSetup` | What the host passes at creation (the common `SensorAdapterSetup`): the adapter's own configuration file, vehicle name and index, sensor name, physics step, random seed |
| `ImuTruth` | Truth every physics step: time, angular velocity, specific force, rotation from world to sensor, world gravity |
| `ImuChannel` | Values of one sensor: whether a new value was measured, measurement time, value, flags |
| `ImuSample` | Output of one step: the gyroscope and accelerometer `ImuChannel` |
| `ImuCalibration` | Autopilot calibration values: accelerometer bias, scale |
| `ImuAdapterCapabilities` | Declaration of the supported features |
| `LogColumn` | Name and unit of a log column (common) |
| `IImuSensorAdapter` | `step`, `capabilities`, `calibration`. `reset`, `log_columns`, and `log_values` are inherited from the common `ISensorAdapter` |

`test/imu_sensor_adapter_contract.cpp` contains a reference adapter with no error and an example of how the host calls it.

## 5. Capability table

Capabilities are returned in the struct `ImuAdapterCapabilities`. Features the adapter does not have are set to `false`. An adapter does not pretend to have them.

| Item | Meaning |
|---|---|
| `filter` | IMU filter |
| `delay` | Delay of the sensor value itself |
| `separate_gyro_accel_timing` | Separate periods and delays for gyroscope and accelerometer |
| `noise`, `bias`, `drift`, `resolution` | Noise, bias, drift, resolution |
| `range_clamp` | Saturation (sets `kImuFlagSaturated`) |
| `calibration_values` | `calibration()` can return calibration values |
| `deterministic_with_seed` | Same values with the same seed, the same input, and the same call order |
| `thread_safe_across_instances` | Different instances can be called at the same time from different threads |

## 6. What the host (Hakoniwa Drone PRO) does

- Make the IMU implementation selectable in drone_config (the adapter library and the location of its configuration file). The default remains the Hakoniwa model.
- Provide a tool that creates the adapter configuration from drone_config (SDFormat for Gazebo). It also outputs the list of items that could not be converted.
- When an external adapter is selected, both the gyroscope and the accelerometer sensor components use the same single adapter instance.
- Mounting pose, vibration, software delay, logging, and passing values to the autopilot go through the same path as when the Hakoniwa model is selected.
