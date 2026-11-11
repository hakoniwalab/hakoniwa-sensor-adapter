# External sensor plugin interface (design)

This is the interface through which a simulator (the host) loads range and image sensors (rangefinder, LiDAR, depth camera, RGB camera) as shared libraries (plugins). The sensors measure the host's own physics world.
The header is `include/hakoniwa/sensor_adapter/external_sensor_plugin.h` (C).

## 1. Responsibilities

| | Host | Plugin |
|---|---|---|
| Configuration | Passes the plugin location, the sensor configuration file location, the mounting, and the output period | Reads the sensor configuration file |
| World | Passes the physics world (`mjModel` and `mjData` for MuJoCo), the body that carries the sensor, and the mounting pose | Measures the world (rays, rendering). Does not modify the world |
| Time | Calls the functions in lifecycle order, and calls `step` after each physics step | Sample period, scan progression |
| Threads | On request, calls `main_thread_poll` on the main thread | Does work that needs the main thread there (for example camera rendering on macOS) |
| Output | Does nothing | Writes Hakoniwa PDUs and answers services |

Inside the plugin, you can use the adapters of the C++ interfaces in this repository (`IRangefinderSensorAdapter` and others). The C boundary is a wrapper around them.

## 2. Only C types cross the boundary

- Only C structs, numbers, and string pointers cross the boundary. C++ objects, standard library types, and exceptions do not. Memory allocated on one side is never freed on the other side (what the plugin creates, the plugin destroys).
- Because of this, nothing breaks when the host and the plugin use different compilers, different C runtime libraries (MSVC /MT and /MD), or different Debug and Release builds.
- The entry point is one C function, `hako_external_sensor_plugin_v1`. It returns a function table stored in static memory.
- The host checks the table's `struct_size`, `abi_version`, and `interface_revision`, and the physics engine name and version (`world_engine`, `world_engine_version`). If they do not match, the host refuses to load the plugin.
- The creation arguments (`HakoExternalSensorCreateV1`) also have `struct_size`. The plugin refuses them if the size is smaller than the size it knows.
- Failures are returned as a return value (non-zero) and an error string written to a buffer that the host passes. The string is cut to the buffer size and always ends with a terminating 0.

## 3. Lifecycle

| Order | Function | When | What the plugin does |
|---|---|---|---|
| 1 | `create` | While stopped | Reads the configuration and prepares to measure the world |
| 2 | `register_services` | After Hakoniwa asset registration, before the first start | Writes its service configuration file to `service_config_dir` and registers it with `hako_asset_service_initialize` |
| 3 | `on_start` | On the start notification (before the start reply) | Creates the servers for its services |
| 4 | `step` | While running, every step after the physics step, on the simulation thread | Measures, writes PDUs, answers requests |
| 5 | `on_stop` | On stop | Discards pending requests. Releases a waiting `step` |
| 6 | `reset` | On reset | Returns to the state right after creation |
| 7 | `destroy` | Last | Cleans up |
| — | `main_thread_poll` | Between 1 and 7, on the main thread, any number of times | Does work that needs the main thread |

- To start again after a stop, begin from 3 (the service registration remains).
- A sensor with `needs_main_thread` set to 1 depends on `main_thread_poll`. Its `step` may wait until the main thread finishes the work. While it waits, the host does not advance physics. So rendering sees the state of that step.
- `on_stop` and `destroy` release a waiting `step` (`step` does not wait forever; it returns an error). Stopping does not hang.
- For a sensor with `needs_main_thread` set to 1, `destroy` is also called on the main thread. Objects created on the main thread (for example an OpenGL window on macOS) must be cleaned up there.

## 4. Time

- `sim_time_ns` in `step` is the time of the physics state that the sensor measures. Because `step` is called after the physics step, for MuJoCo this is `mjData.time` (ns) after the step.
- Time advances by the physics step (`physics_step_ns`). The first `step` after start has the time after the first physics step. After a reset, time again starts from the time after the first physics step.
- The time of the values that the plugin outputs (PDU time, service response time) is the `sim_time_ns` at which the value was measured.

## 5. Mounting

- The host passes the body that carries the sensor (the physics engine body index), and the mounting position and orientation seen from that body's frame (in that body's frame axes). Every step, the plugin computes the sensor pose from the current pose of that body.
- That body and the bodies below it are the vehicle that carries the sensor. The sensor does not measure them.
- You can attach sensors without modifying the model (MJCF or other).

## 6. How it is verified

`test/external_sensor_plugin_contract.cpp` loads a fake plugin written only in C (`test/external_sensor_plugin/fake_plugin.c`) as a shared library. It checks the following:

- The entry function is exported
- The table size and version
- The lifecycle order (`step` before start is refused; the plugin can start again after a stop)
- Sensors that need the main thread
- Errors (unknown type, arguments that are too small, how the error string is cut)
