#pragma once

#include <cstdint>
#include <string>

#include "hakoniwa/sensor_adapter/range_image_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter {

/// Setup of a LiDAR adapter, 2D or 3D (sensor_name is "lidar" by default).
struct LidarAdapterSetup : RangeImageAdapterSetup {
    LidarAdapterSetup() { sensor_name = "lidar"; }
};

/// One returned point, shaped like a sensor_msgs/PointCloud2 point.
struct LidarPoint {
    /// Position [m], sensor axes at the time the point was measured.
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
    /// 0 unless capabilities().intensity.
    float intensity{0.0F};
    /// Vertical row (0 = lowest).
    std::uint16_t ring{0};
    /// Time since the start of the scan [s].
    float time_offset{0.0F};
    /// Object id from the ray caster, -1 when unknown.
    std::int32_t object_id{-1};
};

/**
 * @brief One complete scan, shaped like sensor_msgs/LaserScan plus PointCloud2.
 *
 * A 2D LiDAR has vertical_count 1. Grids are row-major: one row per vertical
 * sample (lowest first), one column per horizontal sample in angle order.
 * The arrays belong to the adapter and stay valid until its next step().
 * Held between scans (updated false).
 */
struct LidarScan {
    bool updated{false};
    /// Time of the first ray of the scan [ns].
    std::uint64_t sample_time_ns{0};
    /// Sensor pose when the scan finished.
    SensorPoseTruth pose_at_end{};
    /// Horizontal angles about the sensor z axis [rad] (0 = forward, positive to the left).
    double angle_min{0.0};
    double angle_max{0.0};
    double angle_increment{0.0};
    /// Vertical angles from the sensor xy plane [rad] (positive up).
    double vertical_angle_min{0.0};
    double vertical_angle_max{0.0};
    double vertical_angle_increment{0.0};
    std::uint32_t horizontal_count{0};
    std::uint32_t vertical_count{0};
    /// Time between two rays and the duration of the scan [s].
    double time_increment{0.0};
    double scan_time{0.0};
    /// Measurable range [m].
    double range_min{0.0};
    double range_max{0.0};
    /// vertical_count * horizontal_count ranges [m], REP 117 (+inf / -inf outside the range).
    const float* ranges{nullptr};
    /// Same layout as ranges; all 0 unless capabilities().intensity.
    const float* intensities{nullptr};
    /// Returned points (finite ranges only).
    const LidarPoint* points{nullptr};
    std::uint32_t point_count{0};
    /// SensorFlag bits (saturated: at least one range was clipped).
    std::uint32_t flags{0};
};

/// What the adapter implements. A feature it lacks is declared false, never faked.
struct LidarAdapterCapabilities {
    bool requires_ray_caster{false};
    bool requires_renderer{false};
    bool noise{false};
    /// Noise given per distance interval (sigma or percent of distance).
    bool range_dependent_accuracy{false};
    bool resolution{false};
    /// The scan is spread over the steps of one rotation (per-point time, motion distortion).
    bool rotating_scan{false};
    bool intensity{false};
    bool object_id{false};
    bool deterministic_with_seed{false};
    bool thread_safe_across_instances{false};
};

/// One LiDAR (2D or 3D) of one vehicle.
class ILidarSensorAdapter : public ISensorAdapter {
public:
    virtual bool step(const SensorPoseTruth& truth, LidarScan& out, std::string& error) = 0;

    virtual LidarAdapterCapabilities capabilities() const = 0;
};

}  // namespace hakoniwa::sensor_adapter
