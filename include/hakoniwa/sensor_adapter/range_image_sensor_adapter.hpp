#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "hakoniwa/sensor_adapter/sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter {

/**
 * Shared part of the range and image sensors (rangefinder, LiDAR, depth camera,
 * RGB camera).
 *
 * Their truth is the world itself, so the host measures the world and the
 * adapter calls it: the host passes a ray caster and/or an image renderer when
 * it creates the adapter. The adapter does not know the physics engine.
 *
 * Units are metres and radians. Ranges follow REP 117 by default: +inf when
 * nothing is hit within the maximum range, -inf when closer than the minimum.
 */

/// One ray in the sensor axes (FLU). The host casts it from the sensor pose of the current step.
struct Ray {
    /// Start point [m], sensor axes. Usually 0; non-zero for cones or laser spacing.
    Vector3 origin{};
    /// Unit direction, sensor axes.
    Vector3 direction{1.0, 0.0, 0.0};
};

/// What one ray hit.
struct RayHit {
    bool hit{false};
    /// Distance from the ray origin along the direction [m]. Valid when hit.
    double distance{0.0};
    /// Stable id of the object that was hit (chosen by the host), -1 when none.
    std::int32_t object_id{-1};
    /// Surface normal at the hit, sensor axes. Valid when hit.
    Vector3 normal{};
};

/**
 * @brief Casts rays into the host's world (for example MuJoCo's mj_multiRay).
 *
 * Rays start at the sensor pose of the current physics step. The vehicle that
 * carries the sensor is never hit.
 */
class IRayCaster {
public:
    virtual ~IRayCaster() = default;
    /// Casts count rays up to max_distance [m] and writes count hits.
    virtual bool cast(const Ray* rays, std::size_t count, double max_distance, RayHit* hits, std::string& error) = 0;
};

/// What the adapter asks the renderer to draw.
struct RenderRequest {
    std::uint32_t width{0};
    std::uint32_t height{0};
    /// Horizontal field of view of a pinhole camera [rad].
    double horizontal_fov{0.0};
    /// Clip planes [m].
    double near_clip{0.0};
    double far_clip{0.0};
    bool rgb{false};
    bool depth{false};
};

/// What the renderer drew. Valid until the next render call.
struct RenderResult {
    /// rgb8, rows from top to bottom, width * 3 bytes per row. nullptr when not requested.
    const std::uint8_t* rgb{nullptr};
    /// Planar depth (distance from the camera plane) [m], rows from top to bottom;
    /// +inf beyond far_clip or where nothing is drawn. nullptr when not requested.
    const float* depth{nullptr};
};

/**
 * @brief Draws the host's world from the sensor pose of the current physics step.
 *
 * The camera looks along the sensor x axis, with the sensor z axis up in the image.
 */
class IImageRenderer {
public:
    virtual ~IImageRenderer() = default;
    virtual bool render(const RenderRequest& request, RenderResult& result, std::string& error) = 0;
};

/// Setup of a range or image sensor adapter.
struct RangeImageAdapterSetup : SensorAdapterSetup {
    /// The host's ray caster, or nullptr when the host has none.
    IRayCaster* ray_caster{nullptr};
    /// The host's renderer, or nullptr when the host has none.
    IImageRenderer* renderer{nullptr};
};

/// Truth for one physics step: the sensor pose (after the mounting pose is applied).
struct SensorPoseTruth {
    /// Simulation time of this step [ns].
    std::uint64_t sim_time_ns{0};
    /// Sensor position [m], world NWU.
    Vector3 position_world{};
    /// Rotation from the world (NWU) to the sensor axes (FLU).
    Quaternion orientation_world_to_sensor{};
};

/**
 * @brief Camera model, shaped like sensor_msgs/CameraInfo.
 *
 * Images are in the optical frame: x right, y down, z forward (the sensor x
 * axis). Rows go from top to bottom, columns from left to right.
 */
struct CameraInfo {
    std::uint32_t width{0};
    std::uint32_t height{0};
    /// "plumb_bob" (k1, k2, p1, p2, k3), "equidistant", ... ; empty without distortion.
    std::string distortion_model;
    std::vector<double> d;
    /// Intrinsic matrix, row-major 3x3.
    std::array<double, 9> k{};
    /// Rectification rotation, row-major 3x3.
    std::array<double, 9> r{};
    /// Projection matrix, row-major 3x4.
    std::array<double, 12> p{};
    /// Rotation from the sensor axes (FLU) to the optical frame, fixed by this
    /// interface: v_optical = R(q) v_sensor, so forward -> +z, left -> -x, up -> -y.
    Quaternion sensor_to_optical{0.5, 0.5, -0.5, 0.5};
};

}  // namespace hakoniwa::sensor_adapter
