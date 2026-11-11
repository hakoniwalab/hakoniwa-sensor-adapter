# Joint sensors: evaluation results (2026-11-11)

We compared joint sensors (encoder and joint torque) along [Evaluation axes](../drone/evaluation-axes.md).

## How we compare

For joints, we cannot do the comparison used for the force-torque sensor, where the same truth is fed to two implementations and the numbers are compared. Gazebo's gz-sensors has neither an encoder nor any joint sensor.

| Implementation | What it outputs for a joint |
|---|---|
| Gazebo (gz-sim `JointStatePublisher`) | Physics truth (angle, velocity). No noise, resolution or delay |
| MuJoCo (`jointpos`, `jointvel`, `jointactuatorfrc`) | Physics truth. `noise` can be written but is not added. Delay and period can be expressed |
| mujoco_ros2_control | Physics truth (`qpos`, `qvel`, `qfrc_actuator`) |
| Webots (`PositionSensor`) | Angle with noise added to the truth and rounded to the resolution. No velocity output |

So we pass the same motion through the Hakoniwa implementation of the [joint sensor interface](../../design/joint-sensor-adapter.md), line it up with "the values output by the other simulators (truth)", and show what looks different.

| Implementation | Version |
|---|---|
| Hakoniwa | Hakoniwa joint sensor model (an implementation of the equations in this interface's design; the source is not public) |

Date measured: 2026-11-11. Machine: macOS (arm64). Physics step 1 ms.

## How they look different

![Joint sensor comparison](joint-comparison.png)

| Plot | Scene | Other simulators (grey) | Hakoniwa |
|---|---|---|---|
| Top left | An SO-101 joint (Feetech STS3215, 12 bit, 100 Hz) moved slowly. Amplitude 1.15°, period 4 s | Smooth angle | Steps of 0.088° |
| Top right | Velocity of the same motion | Smooth velocity (max 1.8°/s) | Built from the angle difference, so 8.8°/s only in the periods where the count changes by one, and 0 otherwise. This is how the low-speed velocity of a real servo looks |
| Middle left, middle right | A joint measured on the motor side (gear ratio 353.5, backlash 15 arcmin, stiffness 300 N·m/rad, load 1 N·m) moving back and forth | Measured angle = true angle | Lags by the backlash at every reversal, plus the deflection under load (about 11.5 arcmin). The difference is 4 to 19 arcmin and alternates between two values depending on the direction |
| Bottom left | Torque estimated from the motor current (resolution of Dynamixel XM430-W350, friction 0.15 N·m + 0.3 N·m·s/rad) | Torque through the joint | Offset by the friction, with a jump where the velocity changes direction |

## The six axes

| Axis | Hakoniwa | Gazebo, MuJoCo, mujoco_ros2_control |
|---|---|---|
| 0. No-noise agreement | With every error off, the output is the truth itself (a design rule, checked by tests) | Truth |
| 1. Accuracy | Count, backlash, deflection, bias, cyclic error, difference velocity and each torque error follow the design equations (checked against independently computed expected values and statistics of 20,000 samples) | — |
| 2. Coverage (table below) | 13 | Gazebo 0, MuJoCo 2 (delay and period), mujoco_ros2_control 1 (linear conversion), Webots 3 |
| 3. Repeatability | Same sequence with the same seed, same sequence after reset, a separate sequence per joint name | No random numbers used (MuJoCo, Gazebo). mujoco_ros2_control uses no random numbers for joints |
| 4. Speed (mean per physics step) | 0.024 µs (STS3215 settings), 0.10 µs (settings with every error) | — |
| 5. Adoption effort | Only a JSON header to read the configuration | — |

### 2. Coverage

| # | Item | Hakoniwa | Gazebo | MuJoCo | Webots | mujoco_ros2_control |
|---|---|---|---|---|---|---|
| 1 | Encoder count (resolution) | ○ | × | × | ○ (`resolution`) | × |
| 2 | Measured on the motor side (gear ratio) | ○ | × | △ (linear conversion of `actuatorpos`) | × | △ (`transmission`) |
| 3 | Backlash | ○ | × | × | × | × |
| 4 | Torsional deflection | ○ | × | × | × | × |
| 5 | Zero-position bias (per unit) | ○ | × | × | × | × |
| 6 | Cyclic error (encoder accuracy) | ○ | × | × | × | × |
| 7 | Incremental, wrap per revolution | ○ | × | × | × | × |
| 8 | Angle noise | ○ | × | × (can be written but is not added) | ○ | × |
| 9 | Velocity built from differences | ○ | × | × | × | × |
| 10 | Delay, period | ○ | △ (period only) | ○ | △ (period only) | × |
| 11 | Joint torque sensor (torque through the joint) | ○ | × (no part that writes the value was found) | × (actuator force only) | △ (motor torque) | × (actuator force) |
| 12 | Torque estimated from current (including friction) | ○ | × | × | × | × |
| 13 | Torque errors (scale, bias, noise, saturation, resolution) | ○ | × | × | × | × |

The columns other than Gazebo were read from documentation and source. They were not run and measured.

## Method

- The comparison program passes truth from prescribed motion (a function of time) through the Hakoniwa joint sensor, writes it to CSV, and plots it.
- The comparison program and the Hakoniwa implementation are not public.
