#pragma once

#include <cstdint>
#include <string>

#include "hakoniwa/sensor_adapter/range_image_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter {

/// Setup of a depth camera adapter (sensor_name is "depth_camera" by default).
struct DepthCameraAdapterSetup : RangeImageAdapterSetup {
    DepthCameraAdapterSetup() { sensor_name = "depth_camera"; }
};

/// Setup of an RGB camera adapter (sensor_name is "rgb_camera" by default).
struct RgbCameraAdapterSetup : RangeImageAdapterSetup {
    RgbCameraAdapterSetup() { sensor_name = "rgb_camera"; }
};

/// What a depth value means.
enum class DepthType : std::uint8_t {
    /// Distance from the camera plane (z in the optical frame).
    kPlanar = 0,
    /// Distance from the camera centre along the pixel's ray.
    kPerspective = 1,
};

/**
 * @brief One depth image, shaped like sensor_msgs/Image (32FC1, metres) with CameraInfo.
 *
 * data belongs to the adapter and stays valid until its next step(). Held between frames.
 */
struct DepthImage {
    bool updated{false};
    std::uint64_t sample_time_ns{0};
    DepthType depth_type{DepthType::kPlanar};
    std::uint32_t width{0};
    std::uint32_t height{0};
    /// width * height depths [m], rows from top to bottom; +inf / -inf outside the range (REP 117).
    const float* data{nullptr};
    CameraInfo info{};
    /// SensorFlag bits.
    std::uint32_t flags{0};
};

/**
 * @brief One colour image, shaped like sensor_msgs/Image (rgb8) with CameraInfo.
 *
 * data belongs to the adapter and stays valid until its next step(). Held between frames.
 */
struct RgbImage {
    bool updated{false};
    std::uint64_t sample_time_ns{0};
    std::uint32_t width{0};
    std::uint32_t height{0};
    /// Bytes per row.
    std::uint32_t step{0};
    /// rgb8, rows from top to bottom.
    const std::uint8_t* data{nullptr};
    CameraInfo info{};
    /// SensorFlag bits.
    std::uint32_t flags{0};
};

/// What a depth camera adapter implements. A feature it lacks is declared false, never faked.
struct DepthCameraAdapterCapabilities {
    bool requires_ray_caster{false};
    bool requires_renderer{false};
    bool noise{false};
    /// Noise given per distance interval (sigma or percent of distance).
    bool range_dependent_accuracy{false};
    bool resolution{false};
    bool planar{false};
    bool perspective{false};
    bool lens_distortion{false};
    bool object_id{false};
    bool deterministic_with_seed{false};
    bool thread_safe_across_instances{false};
};

/// What an RGB camera adapter implements. A feature it lacks is declared false, never faked.
struct RgbCameraAdapterCapabilities {
    bool requires_ray_caster{false};
    bool requires_renderer{false};
    bool noise{false};
    bool lens_distortion{false};
    bool deterministic_with_seed{false};
    bool thread_safe_across_instances{false};
};

/// One depth camera of one vehicle.
class IDepthCameraSensorAdapter : public ISensorAdapter {
public:
    virtual bool step(const SensorPoseTruth& truth, DepthImage& out, std::string& error) = 0;

    virtual DepthCameraAdapterCapabilities capabilities() const = 0;
};

/// One RGB camera of one vehicle.
class IRgbCameraSensorAdapter : public ISensorAdapter {
public:
    virtual bool step(const SensorPoseTruth& truth, RgbImage& out, std::string& error) = 0;

    virtual RgbCameraAdapterCapabilities capabilities() const = 0;
};

}  // namespace hakoniwa::sensor_adapter
