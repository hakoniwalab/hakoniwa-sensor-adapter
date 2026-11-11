# Joint sensors: comparison of simulators and real devices (design survey)

This is a survey for defining the interface for sensors on robot joints (encoders and joint torque).
We read simulator source code and documentation, and real device datasheets, and compared what is measured and how errors are represented.

Versions and sources surveyed:

- Gazebo: gz-sensors 10.2.0, gz-sim (`JointStatePublisher`, the physics part)
- MuJoCo: 3.17.0 `jointpos`, `jointvel`, `actuatorpos`, `actuatorvel`, `actuatorfrc`, `jointactuatorfrc` (`engine_sensor.c`, XML documentation)
- Webots: `PositionSensor`, `Motor` torque feedback (documentation and source)
- mujoco_ros2_control: `MujocoSystemInterface` and `hardware_interface.rst` on main
- Isaac Sim 5.x (joint sensors), PyBullet (`getJointState`): documentation
- Real devices: Feetech STS3215, ROBOTIS Dynamixel XM430-W350 and XL430-W250, Franka FR3, KUKA LBR iiwa, Universal Robots UR5e, DOBOT CR5, RLS AksIM-2 (absolute encoder), Harmonic Drive CSF (gearbox)

## 1. Decisions made first

- Units are SI (rad, rad/s, N·m). Datasheet values (bits, counts, degrees, arcmin, rpm, %) are converted where the configuration is read.
- The interface does not depend on ROS. The values resemble `sensor_msgs/JointState`.
- Time, random seed, `reset`, and log columns follow the same rules as the existing sensor interfaces.

## 2. Simulators

| | Gazebo | MuJoCo | Webots | mujoco_ros2_control | Isaac Sim | PyBullet |
|---|---|---|---|---|---|---|
| Angle | Physics truth (`JointStatePublisher`) | `jointpos` = `qpos`; `actuatorpos` is multiplied by the gear ratio | `PositionSensor`: adds noise to the truth, then rounds to the resolution | `qpos` as is | Physics truth | Truth |
| Velocity | Physics truth (not a difference) | `jointvel` = `qvel` (truth) | Not output | `qvel` as is | Truth | Truth |
| Torque | There is a field for it, but no code that writes the value was found (suspected empty or 0) | `jointactuatorfrc` = sum of the actuator forces on that joint. Does not include constraints, friction, or contact | Motor torque (does not include springs, damping, or constraints) | `qfrc_actuator` | Joint force projected onto the joint axis | Commanded motor torque |
| Noise | None | `noise` can be written but is not added | Gaussian on the angle (absolute σ) | None for joints | None | None |
| Resolution | None | None (only clipping with `cutoff`) | `resolution` (suspected asymmetric rounding for negative values) | None | None | None |
| Delay, period | Period only | `delay`, `interval`, interpolation | Period only | None | None | None |
| Gearbox, backlash | None | Linear gear ratio only (`gear`) | None | Linear conversion with the ros2_control `transmission` | None | None |
| Random numbers | None | None | The seed can be set | None for joints | None | None |

Observations:

- **Nothing outputs encoder counts:** None represents counts, incremental encoders, values that wrap per revolution, or multi-turn values. Only Webots `resolution` does any rounding.
- **Velocity is always the physics truth:** Many real devices produce velocity from differences of rounded angles. None reproduces the resulting steps at low speed or the delay.
- **No form for measuring on the motor side:** Measuring before the gearbox makes the angle resolution finer by the gear ratio, but the value deviates from the true joint angle by the backlash and deflection. All of them stop at a linear conversion.
- **The meaning of torque varies:** What is measured differs: the force the actuator produced (MuJoCo), motor torque (Webots), the force transmitted through the joint (Isaac Sim), the command (PyBullet). None has the noise, bias, or bandwidth of a torque sensor.

## 3. Real devices

| | Angle resolution | Absolute or incremental | Gear ratio, backlash | Velocity | Torque |
|---|---|---|---|---|---|
| Feetech STS3215 | 12 bits (4096 counts/rev, 0.088°) | Magnetic absolute. Multi-turn ±7 turns (the turn count is lost when power is off) | 1:345 (backlash not stated) | Unit only (50 steps/s = 0.732 rpm). How it is produced is not stated | Load (output duty, 1000 = 100 %). Not a torque measurement. Current 6.5 mA/count |
| Dynamixel XM430-W350 | 12 bits (AS5045) | Absolute | 353.5:1, backlash 15 arcmin (0.25°) | 0.229 rpm/count | Current 2.69 mA/count, torque constant 1.78 to 1.81 N·m/A |
| Dynamixel XL430-W250 | 12 bits (AS5601) | Absolute | 258.5:1 | 0.229 rpm/count | No current sensor. Load estimated from the output (0.1 %) |
| Franka FR3 | Not published | — | — | Output at 1 kHz | Torque sensors on all 7 axes (range ±87 and ±12 N·m). Resolution and accuracy not published |
| KUKA LBR iiwa | Not published | — | — | — | Torque sensors on all axes, accuracy ±2 % of maximum torque |
| UR5e, DOBOT CR5 | Not published (only end-effector repeatability ±0.02 to 0.03 mm) | — | — | — | — |
| RLS AksIM-2 | Up to 20 bits | Absolute (multi-turn available) | — | Sample rate 18 kHz, delay under 10 µs | — |
| Harmonic Drive CSF | — | — | Backlash 18 to 60 arcsec, hysteresis 1 to 2 arcmin, torsional stiffness as a 3-segment piecewise-linear curve | — | — |

Sources of the numbers (retrieved 2026-11-11):

| Model | Source |
|---|---|
| Feetech STS3215 | https://www.mantech.co.za/datasheets/products/sts3215-200620a-1.pdf |
| Dynamixel XM430-W350 | https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/ |
| Dynamixel XL430-W250 | https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/ |
| Franka FR3 | Datasheet R02212 v2.6 |
| KUKA LBR iiwa | Product brochure |
| UR5e | https://www.universal-robots.com/manuals/latest/en/datasheets/ur5e/ |
| RLS AksIM-2 | Datasheet MBD01_07 https://cdn.rls.si/documents/datasheets/MBD01_07_ds.pdf |
| Harmonic Drive CSF | Gear unit catalog https://global.harmonicdrive.net/_hd/content/documents1/CSG-CSF_GearUnits.pdf |

### Items that appear in datasheets

| Item | Where it appears | How it is written |
|---|---|---|
| Angle resolution | Servos, encoders | Bits, counts per revolution, degrees per count |
| Absolute or incremental, multi-turn | Most | In words |
| Gear ratio | Servos, gearboxes | N:1 |
| Velocity, current, and load units | Servos | rpm, mA, or % per count |
| Backlash, hysteresis | Gearboxes, some servos | arcmin, arcsec |
| Torsional stiffness | Gearboxes | N·m/rad (piecewise linear) |
| Torque sensor accuracy | Some collaborative robots | ±% of maximum torque |
| Sample period, delay | Encoders, some robots | kHz, µs |
| Encoder noise | Nowhere | — |
| How velocity is produced | Nowhere | Unit only |
| Bit count of collaborative robot joint encoders | Not published | — |

## 4. Findings

- **No simulator really represents encoders:** None has counts, velocity from differences, motor-side measurement, or backlash. This is where Hakoniwa can make a difference.
- **For torque, the interface must define "what is measured":** A joint torque sensor (the force transmitted through the joint) and an estimate from current (the actuator force plus friction) are different things.
- **Many items have no numbers in datasheets:** No model states the noise or how velocity is produced. These are made selectable in the configuration, with no error by default.

## 5. What to adopt in the interface (proposal)

| Item | Proposal | Reason |
|---|---|---|
| Unit of one interface | One per joint (angle, velocity, torque) | Model and gear ratio differ per joint. The same configuration file can be used for any number of joints |
| Truth | The host passes the joint angle and velocity, the actuator force, and the torque transmitted through the joint | A torque sensor and an estimate from current measure different things |
| Resolution | Counts per revolution (can also be written in bits). Also write the measuring side (joint side or motor side) and the gear ratio | Matches how datasheets are written. On the motor side, the joint angle resolution becomes finer by the gear ratio |
| Backlash, deflection | Only when measuring on the motor side. Backlash as a width (play), deflection as a linear stiffness | Gearbox catalog values can be used directly. A piecewise-linear stiffness is first approximated by one straight line |
| Absolute or incremental | Incremental outputs values with the initial angle as 0. Values that wrap per revolution can also be selected | Behavior of real servos |
| Velocity | Truth, or the difference of rounded angles (window length selectable). Can be rounded to the velocity unit (rpm/count) | Many real devices use differences. None reproduces it |
| Torque | Torque sensor (force transmitted through the joint) or estimate from current (actuator force + friction). Accuracy (%), bias, noise, resolution, range | Select the difference in what is measured explicitly in the configuration |
| Delay, period | Same as the existing sensors | Same idea as MuJoCo `delay` |
| Noise | σ in counts. Default 0 | Not in datasheets, so it is a value the user tunes |
| Unit notation | Angles in rad, also `deg`, `arcmin`, `arcsec`; velocity also in `rpm` | Datasheet values can be copied directly |
