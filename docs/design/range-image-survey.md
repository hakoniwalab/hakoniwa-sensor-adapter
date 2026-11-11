# Range and image sensors: comparison of Gazebo, Project AirSim, and Hakoniwa (design survey)

This is a survey for defining the interfaces for rangefinders, 2D LiDAR, 3D LiDAR, depth cameras, and RGB cameras.
We read the Gazebo and Project AirSim source code and compared configuration, measurement method, and output. Existing Hakoniwa implementations (hakoniwa-mujoco-robots, hakoniwa-unity-drone) are also listed.

Versions surveyed:

- Gazebo: SDFormat 1.12, gz-sensors 10.2.0, gz-sim (Jetty)
- Project AirSim: commit `92032a6c` (core_sim and the Unreal plugin)

## 1. Decisions made first

- Units are meters (angles in radians). Datasheet values (millimeters, % of distance) are converted where the configuration is read.
- The interfaces do not depend on ROS. The data types resemble ROS `Range`, `LaserScan`, `PointCloud2`, `Image`, and `CameraInfo`, with matching names and meanings.

## 2. Rangefinder (single ray)

| | Gazebo | Project AirSim | Existing Hakoniwa (mujoco-robots ultrasonic) |
|---|---|---|---|
| Dedicated sensor | None. There is an SDF definition for sonar, but no implementation. A LiDAR with one ray is used | `distance-sensor` | `ultrasonic` |
| Measurement method | Same as LiDAR | One ray (forward) against Unreal collision | Casts several rays inside a cone and takes the closest distance after converting to the forward direction |
| Configuration | Same as LiDAR. A GPU LiDAR with one ray divides the angle step by 0 | `min-distance` (used only for MAVLink), `max-distance`, `report-frequency` | Measurable range, accuracy per distance (σ, quantization), cone angle and ray count, ultrasonic or infrared |
| No hit | Same as LiDAR (+inf) | Not defined (stays at the default value) | Maximum of the measurable range |
| Noise | Same as LiDAR | None | σ per distance and quantization |
| Output | Same as LiDAR | Distance (float) and world pose (NED) | `Range` (distance and variance) |
| Observations | The "altimeter" that measures height with pressure is not a rangefinder (it is the difference from the initial height) | **The distance is output in centimeters** (the Unreal unit is not converted) | Unit is meters |

## 3. 2D and 3D LiDAR

| | Gazebo (`lidar`: CPU, `gpu_lidar`: GPU) | Project AirSim (`lidar`) | Existing Hakoniwa |
|---|---|---|---|
| Configuration form | `scan/horizontal` and `scan/vertical` with ray count, resolution, minimum and maximum angle; `range` minimum, maximum, resolution; `noise` | Type (`lidar-type`: cylindrical, GPU, depth, rosette, 2 Livox models), channel count, points per second, rotations per second, vertical and horizontal field of view, laser spacing | 2D (mujoco-robots): datasheet range, accuracy per distance, angle range and resolution, scan frequency. 3D (Unity): same form as AirSim |
| World measured | CPU: physics collision. GPU: visual shapes | Unreal collision, or depth rendering | 2D: MuJoCo collision. 3D: Unity collision |
| Timing within one scan | Everything is measured at one pose. No per-point time | Same pose within one step. Points accumulate across steps. No per-point time | Everything is measured in one instant |
| Vehicle tilt | The scan follows the sensor orientation | Follows the sensor orientation | 2D: **always horizontal** (only yaw is used). 3D: follows the sensor orientation |
| No hit | +inf; closer than the minimum gives −inf (the REP 117 rule). NaN becomes the maximum | Drop the point or insert a set value (configurable) | Maximum distance |
| Noise | Gaussian per ray. Added only to finite values, then clamped to the range. No quantization (`range/resolution` has no effect anywhere). The time-varying bias does not move | None | 2D: σ per distance, % of distance, quantization. 3D: none |
| Intensity | CPU: always 0. GPU: the reflectance assigned to the visuals | Created by a fixed formula from incidence angle and distance (independent of material) | 2D: 0. 3D: always 1 |
| Other per-point information | Ring (vertical layer index) | Segmentation index, laser index, azimuth, elevation, distance | None |
| Output | `LaserScan` (vertical and horizontal grid, with world pose), `PointCloudPacked` (x, y, z, intensity, ring) | Point cloud (NED, converted to sensor axes) and world pose | 2D: `LaserScan`. 3D: `PointCloud2` (FLU), pose PDU, retrieval service |
| Observations | Does not measure when there is no subscriber | The units of azimuth, elevation, and distance differ between hit and no hit (cm and degrees are mixed). The GPU LiDAR does not use the rotation settings | 2D distances are written in millimeters |

## 4. Depth camera

| | Gazebo (`depth_camera`, `rgbd_camera`) | Project AirSim (image types `DepthPlanar`, `DepthPerspective`) | Existing Hakoniwa (mujoco-robots) |
|---|---|---|---|
| Measurement method | Depth rendered on the GPU | Depth rendered by Unreal | Reads depth from MuJoCo's standard rendering and converts it to meters |
| Meaning of depth | Could not confirm whether it is planar distance or distance along the ray | Two kinds: planar distance (from the camera plane) and distance along the ray (from the camera point) | Rendering depth (planar distance) |
| Out of range | Too far gives +inf, too close gives −inf | Sky is +inf; no upper range limit | Clipped at near and far |
| Type | 32-bit float, meters | 16-bit float, meters | 32-bit float, meters |
| Noise | Gaussian per pixel (GPU) | Noise as a screen effect (no seed) | Gaussian |
| Observations | `depth_camera/clip` is not read by the depth camera (only `rgbd_camera` uses it) | With the default settings (compression on), the depth image is empty | Not confirmed whether it runs in a headless environment |

## 5. RGB camera

| | Gazebo (`camera`) | Project AirSim (`Scene`) | Existing Hakoniwa (mujoco-robots) |
|---|---|---|---|
| Configuration | Field of view, image size and format, near and far, distortion (Brown), lens (fisheye and others, intrinsics), noise | Image size, field of view (degrees), exposure, motion blur, gamma, depth of field, chromatic aberration, noise effect | Field of view, image size and format, near and far, noise |
| Output | `Image`, `CameraInfo` (K, distortion, P) | Image (BGR or PNG), CameraInfo (no distortion), world pose | `Image` |
| Optical frame | There is a setting for the optical frame name (z forward), but it is not used anywhere. The data stays in sensor axes (x forward) | No conversion to the optical frame | MJCF camera |
| Observations | Few pixel formats are supported (R8G8B8, L8, L16, Bayer) | If the field of view is not written, the CameraInfo focal length becomes NaN | |

## 6. Findings common to all three

- **No per-point time:** All of them measure one scan or one image at one pose and one time. None reproduces the distortion of a LiDAR that measures while rotating.
- **Weak support for reproducible values with a given seed:** Gazebo has one random generator per process. Project AirSim range and image sensors have no noise at all.
- **The existing Hakoniwa implementation matches rangefinder datasheet accuracy best:** The cone model, accuracy per distance, and quantization are in neither Gazebo nor Project AirSim.
- **Unit and axis conventions are unclear:** The Project AirSim rangefinder outputs centimeters, and the Gazebo camera does not use its optical frame setting.

## 7. What to adopt in the interfaces (proposal)

| Item | Proposal | Reason |
|---|---|---|
| World measured | Physics collision (same as the Gazebo CPU LiDAR) | Matches the Hakoniwa rule: cheap visuals, collision shapes at real-object size |
| Configuration form | Base it on the Gazebo `scan`, `range`, and `noise` form. Provide a mapping table to the AirSim names (channel count, points per second, rotation rate, field of view). Datasheet accuracy per distance (the mujoco-robots form) can also be written | Both SDF users and AirSim users can bring their settings as they are |
| No hit | Default to the REP 117 rule (too far +inf, too close −inf). Inserting the maximum distance can also be selected | Matches Gazebo and ROS conventions |
| Resolution, quantization | Actually applied (not effective in Gazebo) | Datasheet values can be used directly |
| Vehicle tilt | Use the full sensor orientation | Drones fly tilted |
| Per-point time | Supported. Distortion from measuring while rotating can be switched on and off in the capability table | None has it. It matters for fast-flying drones |
| Seed | Same rule as the existing 5 sensors (created from the seed, vehicle, and sensor name) | Reproducible as defined, however many units are placed and however many times it is rerun |
| Intensity, per-point information | Intensity is declared in the capability table (0 is acceptable at first). The index of the hit object can be included | Usable for segmentation and as ground truth for learning |
| Meaning of depth | Select planar distance or distance along the ray explicitly in the configuration | Project AirSim has two kinds; Gazebo is unclear |
| Camera axes | The interface explicitly defines the optical frame (z forward, x right, y down) and always outputs `CameraInfo` (K, distortion, P) | Both Gazebo and Project AirSim are unclear here |
| Rangefinder | Base it on the cone model (the mujoco-robots form), and distinguish ultrasonic, infrared, and laser | Neither Gazebo nor Project AirSim has a proper one |
