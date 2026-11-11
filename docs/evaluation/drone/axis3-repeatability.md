# Axis 3: repeatability (2026-11-01)

We checked whether the same seed gives the same value sequence. We compare the first 500 values in a stationary state for the IMU (gyro noise, turn-on bias, slowly varying bias).

Method: as in "Method" in [Drone sensor results](README.md).

| Case | Hakoniwa | Gazebo | Project AirSim (core_sim) |
|---|---|---|---|
| Same instance after reset | Same | **Different** (the seed is not set again; see below) | Same |
| Instance recreated with the same seed | Same | Same | Same |
| Running together with other sensors | Same | **Different** | Same |
| Resetting another sensor midway | Same | **Different** | Same |
| Two instances run at the same time on two threads | Same | **Different** | Same |
| Barometers run in two processes on the same PC | (not applicable) | Not mixed (after the fix; mixed before the fix) | (no transport is used, so not mixed) |
| Different OS | Not checked (measured on macOS only) | Not checked | Not checked |

## Explanation

- **Project AirSim (core_sim)**: Each instance has its own random number generator, started with a fixed seed (1, 2, 3; 42 for the barometer). It is not affected by other sensors or threads. However, it does not use the host seed, so changing the seed does not change the value sequence. Any number of vehicles side by side get the same noise.

- **Hakoniwa**: The random stream is built per sensor and per property from the seed, the vehicle name and index, and the sensor name. It is not affected by other sensors or threads.
- **Gazebo**: There is one random number generator per process (`gz::math::Rand`). When another sensor draws a random number, the order shifts. Drawing from threads at the same time causes contention.
  - The Gazebo adapter sets the seed only at creation. Setting it again at every reset would restart the random sequence of the other sensors in the same process too (this was fixed after the first measurement). In exchange, after a reset the value sequence is not the same as right after creation.
  - Creating another sensor sets that sensor's seed, so the sequence of a sensor created earlier also changes (this is why "Resetting another sensor midway" differs).
- **Two processes on the same PC**: We ran barometers at heights 0 m and 3000 m in separate processes at the same time. In the first measurement, although the heights were constant, the pressure in both processes jumped by 31,204 Pa (the pressure difference between 0 m and 3000 m). The Gazebo adapter receives the barometer value over transport, but the transport name was decided only within the process, so it matched the name in the other process and the adapter received the other process's values. The name now includes the process ID, and the jump is 0.
