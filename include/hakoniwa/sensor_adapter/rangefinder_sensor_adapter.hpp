#pragma once

#include <cstdint>
#include <string>

#include "hakoniwa/sensor_adapter/range_image_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter {

/// Setup of a rangefinder adapter (sensor_name is "rangefinder" by default).
struct RangefinderAdapterSetup : RangeImageAdapterSetup {
    RangefinderAdapterSetup() { sensor_name = "rangefinder"; }
};

/// What the rangefinder emits (shaped like sensor_msgs/Range).
enum class RadiationType : std::uint8_t {
    kUltrasound = 0,
    kInfrared = 1,
    kLaser = 2,
};

/// One measured range, shaped like sensor_msgs/Range. Held between measurements.
struct RangeSample {
    bool updated{false};
    std::uint64_t sample_time_ns{0};
    RadiationType radiation_type{RadiationType::kLaser};
    /// Full cone angle [rad]; 0 for a single beam.
    double field_of_view{0.0};
    /// Measurable range [m].
    double min_range{0.0};
    double max_range{0.0};
    /// Measured range along the sensor x axis [m]; +inf / -inf outside the range (REP 117).
    double range{0.0};
    /// Variance of range [m^2] when the adapter knows it, else 0.
    double variance{0.0};
    /// SensorFlag bits.
    std::uint32_t flags{0};
};

/// What the adapter implements. A feature it lacks is declared false, never faked.
struct RangefinderAdapterCapabilities {
    bool requires_ray_caster{false};
    bool requires_renderer{false};
    bool noise{false};
    /// Noise given per distance interval (sigma or percent of distance).
    bool range_dependent_accuracy{false};
    bool resolution{false};
    /// Several rays inside a cone, the nearest projected hit wins.
    bool cone{false};
    bool deterministic_with_seed{false};
    bool thread_safe_across_instances{false};
};

/// One rangefinder of one vehicle.
class IRangefinderSensorAdapter : public ISensorAdapter {
public:
    virtual bool step(const SensorPoseTruth& truth, RangeSample& out, std::string& error) = 0;

    virtual RangefinderAdapterCapabilities capabilities() const = 0;
};

}  // namespace hakoniwa::sensor_adapter
