# Axis 4: speed (2026-11-19)

We ran the five sensors with a 1 ms physics step for 60 seconds (60,000 steps) and measured the time per step. The sample periods are IMU 250 Hz, magnetometer 50 Hz, barometer 50 Hz, GPS 10 Hz.
We measured two cases, without errors and with errors (noise, IMU slowly varying bias), each implementation in a separate process. Project AirSim is the core_sim sensors compiled as is. macOS (Apple Silicon). The clock resolution is about 0.04 µs. The tables are the first of three runs; the summary gives the range of the three.

Method: as in "Method" in [Drone sensor results](README.md).

## Time per physics step (µs)

| Implementation | Errors | Sensors | Median | 99 % | Mean |
|---|---|---|---|---|---|
| Hakoniwa | Off | Total of 5 | 0.125 | 0.250 | 0.117 |
| Hakoniwa | On | Total of 5 | 0.125 | 0.666 | 0.170 |
| Gazebo | Off | Total of 5 | 0.084 | 7.0 | 0.629 |
| Gazebo | On | Total of 5 | 0.084 | 7.5 | 0.694 |
| Project AirSim (core_sim) | Off | Total of 5 | 0.083 | 11.5 | 0.715 |
| Project AirSim (core_sim) | On | Total of 5 | 0.083 | 11.5 | 0.707 |

Mean per sensor (µs, with errors):

| Sensor | Hakoniwa | Gazebo | Project AirSim |
|---|---|---|---|
| IMU | 0.097 | 0.334 | 0.212 |
| Magnetometer | 0.025 | 0.045 | 0.298 (two run; see below) |
| Barometer | 0.025 | 0.286 (5.5 at 99 %) | 0.131 |
| GPS | 0.023 | 0.029 | 0.065 |

## Summary

- Means of the five sensors over three runs: Hakoniwa 0.11 to 0.17 µs, Gazebo 0.62 to 0.72 µs, Project AirSim 0.71 to 0.74 µs (2026-11-01: 0.11 to 0.18, 0.68 to 0.77, 0.79 to 0.85).
- On average, Gazebo takes about 4 to 6 times as long as Hakoniwa. The medians are the same. The difference comes from cost paid only on the physics steps where a sample is taken (the IMU samples once every 4 steps, and so on).
- The Gazebo barometer crosses the transport thread once and waits for a message at every sample, so the 99 % value jumps to several µs.
- All three are below 0.1 % of the 1 ms physics step. The sensor computation does not limit the simulation speed.
- Project AirSim (core_sim) is about as fast as Gazebo on average (about 4 to 6 times Hakoniwa). At every sample it packs the result into a message (including JSON) and sends it out, and the adapter reads it back. Its 99 % value also jumps to about 12 µs.
- For the Project AirSim magnetometer, the field cannot be set from the host, so two instances run, one as configured and one without noise, and the difference is taken. This makes it heavier than the other sensors.
- Peak memory: Hakoniwa 25.5 MB, Gazebo 47.3 MB, Project AirSim 25.8 MB (whole measurement process).
