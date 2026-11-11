# Range and image sensor adapter interfaces (design)

This document defines the interfaces for rangefinders, LiDAR (2D and 3D), depth cameras, and RGB cameras.
The survey (comparison of Gazebo, Project AirSim, and Hakoniwa) is [range-image-survey.md](range-image-survey.md). The interfaces follow the same approach as the 5 sensor interfaces ([imu-adapter.md](imu-adapter.md), [mag-baro-gps-adapter.md](mag-baro-gps-adapter.md)).

## 1. Purpose and scope

- The interfaces are split by use.

  | Interface | Use (drone) | ROS type it resembles |
  |---|---|---|
  | `IRangefinderSensorAdapter` | Height above ground, landing, terrain following, nearby obstacles | `sensor_msgs/Range` |
  | `ILidarSensorAdapter` | 2D: horizontal obstacles, wall following. 3D: obstacle avoidance, mapping | `sensor_msgs/LaserScan`, `sensor_msgs/PointCloud2` |
  | `IDepthCameraSensorAdapter` | Obstacles ahead, shapes at short range | `sensor_msgs/Image` (32FC1), `sensor_msgs/CameraInfo` |
  | `IRgbCameraSensorAdapter` | Viewing, object detection, landing on a marker | `sensor_msgs/Image` (rgb8), `sensor_msgs/CameraInfo` |

- 2D and 3D LiDAR use the same interface. With one vertical layer, it is 2D.
- The interfaces do not depend on ROS. The data types resemble ROS types, with matching field names and meanings. Conversion to ROS or Hakoniwa PDUs is done outside the interface (by the host).
- Units are meters, and angles are radians.

## 2. Host and adapter responsibilities

The difference from the 5 sensors is that **the truth is "the world itself"**. Both distances and images are determined by the objects in the world. So the host owns the part that measures the world (casting rays, rendering), and the adapter calls it.

```text
Host: sensor pose (physics) ─────────────────┐
Host: world measurement (rays, rendering) ◀┐ │
                                           │ ▼
Adapter: sample period, scan pattern → request rays or rendering → range, noise, resolution, no-hit handling → output
                                                                                                                │
Host:                                                                                                           ▼
        conversion to ROS and PDUs → autopilot, log, display
```

| Owner | Content | Reason |
|---|---|---|
| Host | Sensor pose (after the mounting pose is applied), casting rays into the world, rendering the world, excluding its own vehicle from hits | The host owns the physics world. The adapter does not need to know the physics engine |
| Adapter | Sample period, scan pattern (angles, layers, rotation), range, noise, resolution, no-hit handling, lens, camera intrinsics | These are the sensor properties themselves. They are written with datasheet values |
| Host | Converting the output to ROS or PDUs | Outside the interface |

## 3. Decisions (principles)

### 3.1 The host passes the part that measures the world

At creation, the host passes the "part that measures the world" to the adapter.

- **`IRayCaster`** (casts rays): receives a batch of rays written in sensor axes (origins and directions), and casts them into the world from the sensor pose at the current step. For each ray, it returns whether it hit, the distance, the index of the hit object, and the normal of the hit surface (in sensor axes).
- **`IImageRenderer`** (renders): a function. Each request carries the camera pose of the step, the camera (image size, intrinsics K, near and far), the outputs wanted (RGB, depth, segmentation) and destinations the caller owns. The renderer writes the images there. Several requests (the cameras of a rig, such as a stereo pair) are drawn in one call at the same world state. The call blocks and may be made from any thread; handing the work to the thread that owns the graphics context is the renderer's business.

Hakoniwa Drone PRO implements rays with MuJoCo `mj_multiRay` and rendering with MuJoCo rendering. It measures the physics collision world (this matches the Hakoniwa rule: cheap visuals, collision shapes at real-object size).

The adapter declares which one it needs in the capability table (`requires_ray_caster`, `requires_renderer`). If the host does not have it, creation is refused.

### 3.2 Truth: sensor pose and time

| Value | Frame | Unit |
|---|---|---|
| Time | — | ns |
| Sensor position | World NWU | m |
| Sensor orientation (rotation from world NWU to sensor axes) | Quaternion | — |

- The sensor axes are FLU (x forward, y left, z up). For range sensors, the x direction is the front.
- The pose is passed so that it can be attached to the output, and so that the adapter can also output point clouds in world coordinates. The host casts the rays, so the adapter does not need to use the pose for computation.

### 3.3 Time and scanning (per-point time)

- As with the 5 sensors, `step` is called every physics step (`sim_time_ns` = 0, dt, 2dt, ...).
- **A LiDAR that measures while rotating casts only the rays it measures in that step.** For example, a LiDAR rotating at 10 Hz run at a 1 ms step measures 1/100 of a revolution per step. When a full revolution is complete, the output is set to `updated = true`. This way, distortion while the vehicle moves appears naturally. Neither Gazebo nor Project AirSim has this property.
- Each point carries its time offset from the start of the scan (`time_offset`).
- A mode that measures everything in one instant (`scan_mode = instant`) can also be selected. It measures in the same way as Gazebo and Project AirSim, and is used for comparisons or to reduce cost.
- A camera renders one image at the step of the capture time (distortion from exposure time is not handled).

### 3.4 No hit and out of range

- The default is the ROS REP 117 rule. Too far or no hit gives +inf. Closer than the minimum gives −inf.
- A mode that "inserts the maximum of the range" can also be selected in the configuration.
- Noise is added only to values within the range, and the result is then clamped to the range. When clamped, `kSensorFlagSaturated` is set.

### 3.5 Noise and resolution

- Distance noise can be written per distance range. There are two ways to write it:
  - σ (m)
  - % of distance (such as the datasheet "3 % of the measured distance")
- Resolution (quantization) is actually applied. Gazebo `range/resolution` has no effect anywhere, but in this interface it does.
- Depth camera noise can also be written per distance, in the same way.
- RGB noise starts with per-pixel Gaussian noise (mean and σ).

### 3.6 Meaning of depth and camera axes

- The depth camera configuration explicitly selects which one it outputs:
  - `planar`: distance from the camera plane (z depth)
  - `perspective`: distance along the ray from the camera point
- The image axes are the camera optical frame (x right, y down, z forward). Rows go from top to bottom, and columns from left to right. The rotation from the sensor axes (FLU) to the optical frame is fixed by the interface, and is output together with `CameraInfo`.
- `CameraInfo` is always output. It contains K (3×3), the distortion model and coefficients, R, P (3×4), and the image size.

### 3.7 Configuration

- As with the 5 sensors, the adapter reads only configuration in its own format. The host does not pass its own configuration.
- The Hakoniwa adapter configuration is SDF-like JSON. Its structure follows the Gazebo `scan`, `range`, and `noise`, and datasheet accuracy per distance (the hakoniwa-mujoco-robots form) can also be written.
- The mapping to the AirSim names is as follows (the tool that reads the configuration converts them).

  | AirSim | Hakoniwa |
  |---|---|
  | `number-of-channels` | `scan.vertical.samples` |
  | `points-per-second` ÷ `horizontal-rotation-frequency` ÷ channel count | `scan.horizontal.samples` |
  | `horizontal-rotation-frequency` | `update_rate` |
  | `horizontal-fov-start-deg`, `horizontal-fov-end-deg` | `scan.horizontal.min_angle`, `max_angle` (radians) |
  | `vertical-fov-lower-deg`, `vertical-fov-upper-deg` | `scan.vertical.min_angle`, `max_angle` (radians) |
  | `range` | `range.max` |
  | Rangefinder `min-distance`, `max-distance` | `range.min`, `range.max` |
  | Camera `width`, `height`, `fov-degrees` | `image.width`, `image.height`, `horizontal_fov` (radians) |
  | `DepthPlanar`, `DepthPerspective` | `depth.type = planar`, `perspective` |

- Datasheet values (millimeters, %) are converted to meters where the configuration is read.

Configuration example (3D LiDAR):

```json
{
  "type": "lidar",
  "update_rate": 10,
  "scan_mode": "rotating",
  "scan": {
    "horizontal": { "samples": 1800, "min_angle": -3.14159265, "max_angle": 3.14159265 },
    "vertical":   { "samples": 16,   "min_angle": -0.26179939, "max_angle": 0.26179939 }
  },
  "range": { "min": 0.1, "max": 100.0, "resolution": 0.002 },
  "accuracy": [
    { "range": { "min": 0.1, "max": 1.0 },   "stddev": 0.03 },
    { "range": { "min": 1.0, "max": 100.0 }, "percent": 0.3 }
  ],
  "no_return": "inf"
}
```

### 3.8 Random numbers and reset

- Same rules as the 5 sensors. Random streams are created from the seed, the vehicle name and index, and the sensor name.
- `reset` returns the in-progress scan state, the accumulated points, and the random streams to their state at creation.

### 3.9 Output size

- Point clouds and images are large, so they are returned as references to buffers owned by the adapter. A reference is valid until the next `step`. The host copies the data in that time.
- The maximum size is fixed at creation (from the scan pattern and the image size). It does not grow during `step`.

## 4. Interface shape

There are 4 headers (`include/hakoniwa/sensor_adapter/`).

| Header | Content |
|---|---|
| `range_image_sensor_adapter.hpp` | Common: `Ray`, `RayHit`, `IRayCaster`, `RenderRequest`, `RenderStatus`, `IImageRenderer`, `RangeImageAdapterSetup`, `SensorPoseTruth`, `CameraInfo` |
| `rangefinder_sensor_adapter.hpp` | Rangefinder: `RangeSample`, capability table, `IRangefinderSensorAdapter` |
| `lidar_sensor_adapter.hpp` | LiDAR: `LidarPoint`, `LidarScan`, capability table, `ILidarSensorAdapter` |
| `camera_sensor_adapter.hpp` | Depth and RGB: `DepthImage`, `RgbImage`, capability table, `IDepthCameraSensorAdapter`, `IRgbCameraSensorAdapter` |

The rotation from the sensor axes to the optical frame is `v_optical = R(q) v_sensor` with q = (0.5, 0.5, −0.5, 0.5) (forward → +z, left → −x, up → −y).

### 4.1 The part that measures the world (implemented by the host)

```cpp
struct Ray {
    Vector3 origin;     // sensor axes [m] (usually 0; offset for a cone or laser spacing)
    Vector3 direction;  // sensor axes, unit vector
};
struct RayHit {
    bool hit{false};
    double distance{0.0};   // [m]
    std::int32_t object_id{-1};
    Vector3 normal{};       // normal of the hit surface, sensor axes
};
class IRayCaster {
public:
    virtual ~IRayCaster() = default;
    // Cast rays up to max_distance from the sensor pose at the current step. Rays do not hit the own vehicle.
    virtual bool cast(const Ray* rays, std::size_t count, double max_distance, RayHit* hits,
                      std::string& error) = 0;
};

enum RenderOutput : std::uint32_t { kRenderRgb = 1, kRenderDepth = 2, kRenderSegmentation = 4 };
struct ImageBuffer { void* data{nullptr}; std::size_t capacity{0}; };  // owned by the caller
struct RenderCamera {
    std::uint32_t width{0}, height{0};
    double fx{0.0}, fy{0.0}, cx{0.0}, cy{0.0};  // K, pixel centres at integers
    double near_clip{0.0}, far_clip{0.0};        // [m]
};
struct RenderRequest {
    std::string camera_name;     // for errors and logs
    SensorPoseTruth pose{};      // camera pose of this step; the renderer keeps no mount
    RenderCamera camera{};
    std::uint32_t outputs{0};    // RenderOutput bits
    ImageBuffer rgb;             // rgb8, rows from top to bottom
    ImageBuffer depth;           // float32 planar depth [m], +inf out of range
    ImageBuffer segmentation;    // int32 object id, -1 where nothing is drawn
};
struct RenderStatus { bool ok{false}; std::string error; };
struct ImageRendererCapabilities { std::uint32_t outputs{0}; bool lens_distortion{false}; bool non_square_pixels{false}; };
class IImageRenderer {
public:
    virtual ~IImageRenderer() = default;
    // Draw count requests in one go at the same world state; blocks until done.
    // A destination smaller than the image is an error for that request (nothing written).
    virtual bool render(const RenderRequest* requests, std::size_t count, RenderStatus* statuses) = 0;
    virtual ImageRendererCapabilities capabilities() const = 0;
};
```

### 4.2 What is passed at creation

```cpp
struct RangeImageAdapterSetup : SensorAdapterSetup {
    IRayCaster* ray_caster{nullptr};       // nullptr if the host does not have one
    IImageRenderer* renderer{nullptr};     // same as above
};
```

### 4.3 Truth

```cpp
struct SensorPoseTruth {
    std::uint64_t sim_time_ns{0};
    Vector3 position_world{};                 // NWU [m]
    Quaternion orientation_world_to_sensor{};
};
```

### 4.4 Output

```cpp
// Rangefinder (resembles sensor_msgs/Range)
enum class RadiationType : std::uint8_t { kUltrasound = 0, kInfrared = 1, kLaser = 2 };
struct RangeSample {
    bool updated{false};
    std::uint64_t sample_time_ns{0};
    RadiationType radiation_type{RadiationType::kLaser};
    double field_of_view{0.0};   // [rad]
    double min_range{0.0}, max_range{0.0};  // [m]
    double range{0.0};           // [m], REP 117 (±inf)
    double variance{0.0};        // [m²] (when known from the configuration; 0 otherwise)
    std::uint32_t flags{0};
};

// LiDAR (resembles sensor_msgs/LaserScan and PointCloud2)
struct LidarPoint {
    float x, y, z;         // sensor axes [m]
    float intensity;       // declared in the capability table; 0 if not supported
    std::uint16_t ring;    // vertical layer index
    float time_offset;     // time from the start of the scan [s]
    std::int32_t object_id;
};
struct LidarScan {
    bool updated{false};
    std::uint64_t sample_time_ns{0};    // start of the scan
    SensorPoseTruth pose_at_end{};      // sensor pose when the scan ended
    // LaserScan form (row = vertical layer, column = horizontal). 2D has 1 vertical layer
    double angle_min{0.0}, angle_max{0.0}, angle_increment{0.0};
    double vertical_angle_min{0.0}, vertical_angle_max{0.0}, vertical_angle_increment{0.0};
    std::uint32_t horizontal_count{0}, vertical_count{0};
    double time_increment{0.0}, scan_time{0.0};  // [s]
    double range_min{0.0}, range_max{0.0};       // [m]
    const float* ranges{nullptr};       // vertical_count × horizontal_count, REP 117
    const float* intensities{nullptr};  // same layout
    // PointCloud2 form (hit points, and optionally no-hit points)
    const LidarPoint* points{nullptr};
    std::uint32_t point_count{0};
    std::uint32_t flags{0};
};

// Camera (resembles sensor_msgs/CameraInfo)
struct CameraInfo {
    std::uint32_t width{0}, height{0};
    std::string distortion_model;   // "plumb_bob" and others. Empty for no distortion
    std::vector<double> d;          // distortion coefficients
    std::array<double, 9> k{};      // 3×3
    std::array<double, 9> r{};      // 3×3
    std::array<double, 12> p{};     // 3×4
    Quaternion sensor_to_optical{0.5, 0.5, -0.5, 0.5}; // from sensor axes (FLU) to the optical frame (x right, y down, z forward)
};
enum class DepthType : std::uint8_t { kPlanar = 0, kPerspective = 1 };
struct DepthImage {
    bool updated{false};
    std::uint64_t sample_time_ns{0};
    DepthType depth_type{DepthType::kPlanar};
    std::uint32_t width{0}, height{0};
    const float* data{nullptr};     // [m], rows from top to bottom, REP 117
    CameraInfo info{};
    std::uint32_t flags{0};
};
struct RgbImage {
    bool updated{false};
    std::uint64_t sample_time_ns{0};
    std::uint32_t width{0}, height{0};
    std::uint32_t step{0};          // bytes per row
    const std::uint8_t* data{nullptr};  // rgb8, rows from top to bottom
    CameraInfo info{};
    std::uint32_t flags{0};
};
```

### 4.5 Adapter interfaces

```cpp
class IRangefinderSensorAdapter : public ISensorAdapter {
public:
    virtual bool step(const SensorPoseTruth& truth, RangeSample& out, std::string& error) = 0;
    virtual RangefinderAdapterCapabilities capabilities() const = 0;
};
class ILidarSensorAdapter : public ISensorAdapter {
public:
    virtual bool step(const SensorPoseTruth& truth, LidarScan& out, std::string& error) = 0;
    virtual LidarAdapterCapabilities capabilities() const = 0;
};
class IDepthCameraSensorAdapter : public ISensorAdapter {
public:
    virtual bool step(const SensorPoseTruth& truth, DepthImage& out, std::string& error) = 0;
    virtual DepthCameraAdapterCapabilities capabilities() const = 0;
};
class IRgbCameraSensorAdapter : public ISensorAdapter {
public:
    virtual bool step(const SensorPoseTruth& truth, RgbImage& out, std::string& error) = 0;
    virtual RgbCameraAdapterCapabilities capabilities() const = 0;
};
```

As with the 5 sensors, the creation functions are placed on the adapter implementation side (for example `create_mujoco_lidar_sensor_adapter(const RangeImageAdapterSetup&, std::string& error)`).

## 5. Capability table

| Capability | Rangefinder | LiDAR | Depth | RGB | Meaning |
|---|---|---|---|---|---|
| `requires_ray_caster` | ○ | ○ | ○ | | Needs the ray casting part |
| `requires_renderer` | | | ○ | ○ | Needs the rendering part (depth uses either rays or rendering) |
| `noise` | ○ | ○ | ○ | ○ | Noise |
| `range_dependent_accuracy` | ○ | ○ | ○ | | Accuracy per distance range (σ and %) |
| `resolution` | ○ | ○ | ○ | | Resolution |
| `cone` | ○ | | | | Measures with several rays inside a cone |
| `rotating_scan` | | ○ | | | Measures while rotating (per-point time, distortion from motion) |
| `intensity` | | ○ | | | Intensity |
| `object_id` | | ○ | ○ | | Index of the hit object |
| `lens_distortion` | | | ○ | ○ | Lens distortion |
| `deterministic_with_seed` | ○ | ○ | ○ | ○ | Same values with the same seed and the same input |
| `thread_safe_across_instances` | ○ | ○ | ○ | ○ | Different instances may run on different threads |

## 6. What the host (Hakoniwa Drone PRO) does

- Implement `IRayCaster` with `mj_multiRay`. Rays must not hit the vehicle's own objects (by body exclusion or by geom group). The object index is a stable index derived from the MuJoCo geom.
- Implement `IImageRenderer` with MuJoCo rendering. First use the standard rendering (OpenGL). When Filament is included in the MuJoCo distribution, swap it in behind the same interface.
- Convert the output to the existing contracts:
  - 3D LiDAR: `lidar_points` (PointCloud2), `lidar_pos` (pose), `DroneService/LiDARScan`
  - Rangefinder: `Range`
  - 2D LiDAR: `LaserScan`
  - Camera: `Image`, `CameraInfo`
- The build order is rangefinder → 2D LiDAR → 3D LiDAR → depth camera → RGB camera. For each, verify by measuring the output (place objects of known shape and check that distances, angles, and point positions match the truth).
