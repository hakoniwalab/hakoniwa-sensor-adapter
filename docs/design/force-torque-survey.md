# Force-torque sensors: comparison of simulators and real devices (design survey)

This is a survey for defining the interface for 6-axis force-torque sensors (forces Fx, Fy, Fz and moments Tx, Ty, Tz).
We read simulator source code and documentation, and real device datasheets, and compared what is measured and how errors are represented.

Versions and sources surveyed:

- Gazebo: gz-sensors 10.2.0 (`ForceTorqueSensor`, `GaussianNoiseModel`), the gz-sim ForceTorque system, SDFormat 1.12 `<force_torque>`
- MuJoCo: 3.17.0 `force` and `torque` sensors (`engine_sensor.c`, XML documentation)
- mujoco_ros2_control: `MujocoSystemInterface` and `hardware_interface.rst` on main
- Isaac Sim 5.1 (joint force sensor, contact sensor), Webots `TouchSensor` (force-3d), PyBullet `enableJointForceTorqueSensor`: documentation and main source code
- Real devices: ATI (Mini45, Axia80, Net F/T), Robotiq FT 300-S, OnRobot HEX-E QC / HEX-H QC, Bota SensONE, Universal Robots e-Series end-effector force sensing, KUKA LBR iiwa estimation from joint torques

## 1. Decisions made first

- Units are SI (N, N·m). Datasheet values (% FS, mN, mN·m, per hour) are converted where the configuration is read.
- The interface does not depend on ROS. The data type resembles ROS `geometry_msgs/WrenchStamped`, with matching names and meanings.
- Time, random seed, `reset`, and log columns follow the same rules as the existing sensor interfaces ([imu-adapter.md](imu-adapter.md)).

## 2. Simulators

| | Gazebo (gz-sim and gz-sensors) | MuJoCo (`force`, `torque`) | mujoco_ros2_control | Isaac Sim | Webots (`TouchSensor` force-3d) | PyBullet |
|---|---|---|---|---|---|---|
| Truth | The force transmitted by the joint (physics engine value), moved to the sensor pose. The sensor is placed on a joint | The force that the site's body transmits to its parent through the joint (`cfrc_int`) | Pairs MuJoCo `force` and `torque` | The 6-axis force entering the joint | The sum of external contact forces hitting the sensor's collision shape | Joint reaction force |
| Axes and direction | Axes selectable from child, parent, or sensor. Direction is child to parent (default; the value is negated) or parent to child | Site axes. Child to parent direction | Negates all 6 MuJoCo values (parent to child). Not documented | Not documented | Sensor axes. Force only, no moment | Default axes could not be confirmed |
| Weight of what is attached beyond | Included. No tare | Included (also includes contact and externally applied forces). No tare | Included. No tare | Included | Not included (external contact only) | Included |
| Noise | Gaussian per axis. Fixed bias, time-varying bias (1st-order Gauss-Markov), quantization | `noise` can be written, but `mj_step` does not use it | Adds `noise` at every read (Gaussian or uniform). The seed cannot be set, so values differ every run | Not documented | Noise magnitude in the 3rd column of the lookup table | None |
| Saturation | None | `cutoff` clips each axis | Same as MuJoCo | None for joint forces | Nonlinearity and saturation via the lookup table | None |
| Filter, delay | None | History and interpolation with `nsample`, `delay`, `interval` | None | None | None | None |
| Period | `update_rate` | Every step, or `interval` | Control period | Physics step (not documented) | Period given when enabling | Every step |
| Output | `gz.msgs.Wrench` | Groups of 3 in `sensordata` | 6 state interfaces `force.x` ... `torque.z` | Array per link | 3 values | 6-tuple per joint |

Observations:

- **Gazebo moment reference point:** SDFormat says "about the joint origin", but gz-sim moves it to be about the sensor origin. In that formula, the joint frame position and the force in sensor axes are mixed, so the value may be wrong when the sensor is rotated relative to the joint (not confirmed by a test).
- **Default directions disagree:** In Gazebo, the SDFormat default (child to parent, negated) and the C++ description (parent to child) are opposite. mujoco_ros2_control silently negates the MuJoCo values.
- **Shared random stream:** Gazebo lets you set the seed (the `gz::math::Rand` seed, the gz-sim server configuration). However, there is only one random stream per process, so values change when the number of sensors or the update order changes. The sign of the fixed bias flips with 50% probability. mujoco_ros2_control has a random generator per sensor, but the seed cannot be set, so values differ every run.
- **MuJoCo constraints:** A force-torque sensor needs its own body (connected without a joint) between the parent and the tool. Some forces are not yet collected into `cfrc_int`, such as actuators that act through sites or bodies, gravity compensation, and fluid forces.
- **Small errors:** The SDFormat description of Ty says "force about the Y axis", and the gz-sensors header gives the moment unit as "newtons". The default Webots lookup table multiplies values by 10.

## 3. Real devices

| | ATI Mini45 | ATI Axia80 | Robotiq FT 300-S | OnRobot HEX-E QC | Bota SensONE (T15) | UR e-Series (end effector) |
|---|---|---|---|---|---|---|
| Measurable range (Fxy / Fz / Txy / Tz) | 145 / 290 N, 5 / 5 N·m, and others; 3 calibrations | 500 / 900 N, 20 N·m, and others; 2 calibrations | ±300 N, ±30 N·m | 200 / 200 N, 10 / 6.5 N·m | 500 / 1200 N, 15 / 12 N·m | UR5e: 50 N, 10 N·m |
| Overload | 5.7 to 25.3 times the range | 5 to 12.5 times | 500% | 500% | Absolute values (2500 / 4500 N, and others) | — |
| Noise, resolution | Resolution 1/16 N and others (value excluding 4 counts of noise) | — | Noise 0.1 N, 0.005 N·m (1σ, 1 s) | Noise 0.035 / 0.15 N, 0.002 / 0.001 N·m (1σ, 1 s). Noise-free resolution 0.2 / 0.8 N | Noise-free resolution 100 / 70 mN (6σ peak-to-peak width at 100 Hz) | Precision 3.5 N, 0.2 N·m |
| Accuracy | Not published | Better than 2% FS (per-unit values in the calibration certificate) | Not published | Nonlinearity 2% FS, hysteresis 2%, crosstalk 5% | 2%, nonlinearity 2% | Accuracy 4.0 N, 0.3 N·m |
| Drift | — | "Normal" (no number). 30 min warm-up and frequent tare | Temperature compensation 15 to 35 °C | No number. Automatic calibration every 2 to 3 months | 100 / 500 mN, 3 mN·m per hour | — |
| Output period, delay | Net F/T: up to 7000 Hz, delay 500 µs, low-pass 5 to 838 Hz | — | 100 Hz | — | Up to 3800 Hz | — |
| Tare, tool weight | Tare also removes the tool weight | Same | Tool weight and center of gravity compensation, and value reset | — | Built-in IMU usable for gravity and inertia compensation | — |

Sources of the numbers (retrieved 2026-11-11):

| Model | Source |
|---|---|
| ATI Mini45 | Product page https://www.ati-ia.com/products/ft/ft_models.aspx?id=Mini45 |
| ATI Axia80 | Product page https://www.ati-ia.com/products/ft/ft_models.aspx?id=Axia80-M20 , manual 9620-05-B https://www.ati-ia.com/app_content/Documents/9620-05-B-Axia80.pdf |
| ATI Net F/T | https://ati-ia.com/products/ft/ft_SystemInterfaces.aspx |
| Robotiq FT 300-S | Product page https://robotiq.com/products/ft-300-force-torque-sensor . The noise values come from a search result excerpt of the product sheet (an image PDF) and were not confirmed in the PDF itself |
| OnRobot HEX-E QC / HEX-H QC | Datasheet v1.5 https://onrobot.com/storage/datasheets/hex/datasheet_hex-e_h_qc_v1.5_en.pdf |
| Bota SensONE Gen A | Datasheet https://ventionavatars.s3.amazonaws.com/uploads/part_document/file/1983/SensONE%2BGen%2BA%2BSeries%2BDatasheet.pdf |
| UR e-Series | November 2023 collective datasheet https://www.universal-robots.com/media/1829346/11_2023_collective_data-sheet.pdf |
| KUKA LBR iiwa | Product brochure (joint torque accuracy), independent measurement https://www.frontiersin.org/articles/10.3389/frobt.2022.892916/full |

Collaborative robots that estimate the end-effector force from joint torques (KUKA LBR iiwa) publish only the per-joint torque accuracy (±2% of the maximum torque). An independent measurement found an error of about 2 N for loads of 0 to 20 N, and very large errors near singularities.

### Items that appear in datasheets

| Item | Where it appears | How it is written |
|---|---|---|
| Measurable range | All | Per axis group (Fxy, Fz, Txy, Tz). One device can have several calibrations. The range narrows when forces act on several axes at once (OnRobot, Bota, ATI) |
| Overload | All dedicated sensors | % or a factor, or an absolute value |
| Noise, resolution | All dedicated sensors, but the definitions vary | 1σ over 1 second (Robotiq, OnRobot), 6σ peak-to-peak width at 100 Hz (Bota), value excluding noise counts (ATI) |
| Accuracy | Most | % FS, absolute value (UR), % of maximum torque (KUKA) |
| Output period | Most | Hz (100 Hz to 7 kHz). Delay only for ATI |
| Tare and tool weight compensation | All manuals | Described as a required operation. Drift is also assumed to be removed this way |
| Nonlinearity, hysteresis | Some | % FS |
| Crosstalk | OnRobot only | % |
| Drift | Rare | Per hour (Bota). No value per °C anywhere |
| Deflection | Some | mm and degrees at the rated load |

## 4. Findings

- **No simulator has tare:** None of the simulators has it. On real devices, both the tool weight and drift are assumed to be removed by tare.
- **Conventions for direction, axes, and reference point are unclear:** The direction is reversed in some implementations, and documentation and implementation differ in others.
- **Gazebo has the widest error model, but the random stream is shared:** It has per-axis noise, fixed bias, time-varying bias, and quantization, and the seed can be set. However, there is one random stream per process, so adding sensors or changing the update order changes the values even with the same seed.
- **Crosstalk, overload, sample period and delay, drift rate:** These appear in datasheets but are hardly handled by simulators.
- **Noise definitions must be converted:** They are written in different ways: 1σ, 6σ peak-to-peak width, counts.
- **Estimation from joint torques behaves differently from an end-effector sensor:** The error changes strongly with the posture (the condition of the Jacobian).

## 5. What to adopt in the interface (proposal)

| Item | Proposal | Reason |
|---|---|---|
| Truth | The host passes the 6-axis force at the sensor location in sensor axes. The interface defines the direction (which part applies the force to which) and the moment reference point (the sensor origin) | Implementations differ, so the interface defines one convention |
| Tool weight | The measured value includes the tool weight and inertia. Subtraction is done outside the sensor (controller or host) | Same as real sensors. Removing it inside the sensor would give values different from real devices |
| Tare | Included in the interface as a sensor operation. Stores and subtracts the average output over a set time. Drift is not removed and keeps progressing | No simulator has it, but real devices use it every day. Storing a single value leaves its noise as a fixed offset |
| Configuration form | Written per axis group (Fxy, Fz, Txy, Tz). Values carry a unit suffix (% FS, absolute value) so they can be copied directly from the datasheet | Matches how datasheets are written |
| Noise | σ per axis. The datasheet definition (1σ, 6σ peak-to-peak width) and the output period and bandwidth at which it was measured can be written, and are converted to the adapter's period and bandwidth | Definitions vary, and the same σ is different noise at a different period and bandwidth |
| Accuracy, crosstalk | Represented by a 6×6 calibration matrix, multiplied by the dimensionless values obtained by dividing each axis by its measurable range. Crosstalk and scale error are drawn per unit from the seed at creation (within the accuracy range) | Reproduces per-unit differences, and gives the same unit every run. The datasheet % FS can be used directly |
| Nonlinearity | A stage separate from the matrix (a curve per axis) | Replacing nonlinearity with a linear scale error gives a different error |
| Drift | Rate per hour (random walk or 1st-order Gauss-Markov) | Can represent both the Bota notation and the Gazebo time-varying bias |
| Resolution, saturation, overload | Quantize with the resolution. Outside the range, set the saturation flag. Overload is checked on the physical load before errors are added, with a separate flag | Datasheet values can be used directly. Overload is about how the device breaks, so it depends on the applied load, not on the measured value |
| Period, delay, filter | Sample period, delay, and low-pass (cutoff frequency) are configurable | As with ATI, period, delay, and filter are important properties of real devices |
| Temperature, hysteresis, offset after overload | Switched on and off via the capability table. Not required at first | Datasheets give almost no numbers |
| Seed | Same rule as the existing sensors (created from the seed, vehicle, and sensor name) | Reproducible as defined, however many units are placed and however many times it is rerun |
| Joint torque | A separate interface from end-effector sensors | The errors behave differently (per-joint values, end-effector error that changes with posture) |
