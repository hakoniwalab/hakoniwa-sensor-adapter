# Sensor comparison

These are the results of comparing Hakoniwa's sensors with the sensors of other simulators, on top of this interface (hakoniwa-sensor-adapter).

## How we compare

Each implementation is wrapped as an adapter of the same interface. The host passes the same sequence of truth values to each, and we line up the outputs. The physics is the same, so the only difference is the difference between the sensor error models.

```text
truth sequence ─┬─▶ Hakoniwa adapter               ─┐
                ├─▶ Gazebo (gz-sensors) adapter    ├─▶ line up outputs
                └─▶ Project AirSim adapter         ─┘
```

- There are six axes of comparison: no-noise agreement, accuracy, coverage, repeatability, speed, and adoption effort. They are defined in [Evaluation axes](drone/evaluation-axes.md). They were defined for the drone sensors, and the same axes are used for the force-torque sensor.
- We do not set pass/fail criteria. We list the numbers and the reasons for the differences.
- The Gazebo and Project AirSim adapters are public (hakoniwa-sensor-adapter-gazebo, hakoniwa-sensor-adapter-airsim). Anyone can reproduce those columns.
- The Hakoniwa columns were measured with the sensor models that ship in Hakoniwa products (Hakoniwa Drone PRO and others). Their sources are not public.

## Results by sensor type

| Sensor | Compared | Results |
|---|---|---|
| Drone sensors (IMU, magnetometer, barometer, GPS) | hakoniwa-sensor-pro v0.1.0 (the Hakoniwa Drone PRO sensor v2 models), gz-sensors 10.2.0, Project AirSim core_sim | [Drone sensors](drone/README.md) |
| Drone IMU against real X500 flight logs | Public PX4 Flight Review logs (20 distinct X500 V2 flights) and PX4 v1.17.0 in an athrill3 closed loop | [X500 IMU real-data comparison](imu-x500/README.md) ([日本語](imu-x500/README.ja.md)) |
| 6-axis force-torque sensor | hakoniwa-sensor-pro v0.1.0 (the Hakoniwa force-torque sensor model), gz-sensors 10.2.0. Feature coverage also for MuJoCo and mujoco_ros2_control | [Force-torque sensor](force-torque/README.md) |
| Joint sensors (encoder, joint torque) | Hakoniwa joint sensor model, and the values (truth) output by Gazebo, MuJoCo, mujoco_ros2_control and Webots | [Joint sensors](joint/README.md) |

## Error characteristics, one at a time

How each error of the drone sensors (gyroscope, accelerometer, magnetometer, barometer, GPS) behaves on its own,
measured with the released hakoniwa-sensor-pro binaries through this interface, without a physics engine:
[Error characteristics](error-characteristics/README.md) ([日本語](error-characteristics/README.ja.md)). The tool is
in [tools/error-characteristics](../../tools/error-characteristics).
