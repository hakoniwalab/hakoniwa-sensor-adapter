# Axis 4: speed (2026-11-01)

We ran the five sensors with a 1 ms physics step for 60 seconds (60,000 steps) and measured the time per step. The sample periods are IMU 250 Hz, magnetometer 50 Hz, barometer 50 Hz, GPS 10 Hz.
We measured two cases, without errors and with errors (noise, IMU slowly varying bias), each implementation in a separate process. Project AirSim is the core_sim sensors compiled as is. macOS (Apple Silicon). The clock resolution is about 0.04 µs.

Method: as in "Method" in [Drone sensor results](README.md).

## Time per physics step (µs)

| Implementation | Errors | Sensors | Median | 99 % | Mean |
|---|---|---|---|---|---|
| Hakoniwa | Off | Total of 5 | 0.125 | 0.250 | 0.113 |
| Hakoniwa | On | Total of 5 | 0.125 | 0.750 | 0.183 |
| Gazebo | Off | Total of 5 | 0.125 | 8.4 | 0.676 |
| Gazebo | On | Total of 5 | 0.125 | 9.4 | 0.774 |
| Project AirSim (core_sim) | Off | Total of 5 | 0.083 | 13.8 | 0.850 |
| Project AirSim (core_sim) | On | Total of 5 | 0.083 | 12.7 | 0.787 |

Mean per sensor (µs, with errors):

| Sensor | Hakoniwa | Gazebo | Project AirSim |
|---|---|---|---|
| IMU | 0.104 | 0.351 | 0.260 |
| Magnetometer | 0.026 | 0.050 | 0.317 (two run; see below) |
| Barometer | 0.027 | 0.340 (7.2 at 99 %) | 0.144 |
| GPS | 0.026 | 0.032 | 0.066 |

## Summary

- On average, Gazebo takes about 4 to 6 times as long as Hakoniwa. The medians are the same. The difference comes from cost paid only on the physics steps where a sample is taken (the IMU samples once every 4 steps, and so on).
- The Gazebo barometer crosses the transport thread once and waits for a message at every sample, so the 99 % value jumps to several µs.
- All three are below 0.1 % of the 1 ms physics step. The sensor computation does not limit the simulation speed.
- Project AirSim (core_sim) is about as fast as Gazebo on average (about 5 times Hakoniwa). At every sample it packs the result into a message (including JSON) and sends it out, and the adapter reads it back. Its 99 % value also jumps to about 13 µs.
- For the Project AirSim magnetometer, the field cannot be set from the host, so two instances run, one as configured and one without noise, and the difference is taken. This makes it heavier than the other sensors.
- Peak memory: Hakoniwa 26.4 MB, Gazebo 49.6 MB, Project AirSim 25.9 MB (whole measurement process).
