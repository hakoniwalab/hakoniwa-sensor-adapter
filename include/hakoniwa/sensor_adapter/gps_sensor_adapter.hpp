#pragma once

#include <cstdint>
#include <string>

#include "hakoniwa/sensor_adapter/sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter {

/// Geodetic position on WGS-84 (altitude above the ellipsoid, no geoid model).
struct GeodeticPosition {
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_m{0.0};
};

/// Setup of a GPS adapter (sensor_name is "gps" by default).
struct GpsAdapterSetup : SensorAdapterSetup {
    GpsAdapterSetup() { sensor_name = "gps"; }
    /// Origin of the local NWU frame (the simulation location).
    GeodeticPosition origin{};
};

/**
 * @brief True values at the antenna position for one physics step.
 *
 * The host has already applied the antenna position (mounting position).
 * It gives the antenna both in the local NWU frame and as a geodetic position
 * converted from it, so an adapter may apply its errors in either.
 */
struct GpsTruth {
    /// Simulation time of this step [ns].
    std::uint64_t sim_time_ns{0};
    /// Antenna position [m] in the local NWU frame from GpsAdapterSetup::origin.
    Vector3 position_nwu{};
    /// Antenna velocity [m/s], NWU.
    Vector3 velocity_nwu{};
    /// The same antenna position as a geodetic position.
    GeodeticPosition position{};
};

/// One measured GPS fix.
struct GpsSample {
    bool updated{false};
    std::uint64_t sample_time_ns{0};
    /// Measured position and velocity. Held between measurements.
    GeodeticPosition position{};
    /// Measured velocity [m/s], NWU.
    Vector3 velocity_nwu{};
    /// Horizontal speed [m/s] and course over ground [deg, 0..360 from north].
    double ground_speed_mps{0.0};
    double course_deg{0.0};
    /// False when the speed is too low for a course.
    bool course_valid{false};
    /// Accuracy estimates the receiver reports [m], [m], [m/s].
    double horizontal_accuracy_m{0.0};
    double vertical_accuracy_m{0.0};
    double speed_accuracy_mps{0.0};
    std::uint32_t satellites_visible{0};
    /// 0: no fix, 2: 2D, 3: 3D.
    std::uint32_t fix_type{0};
    /// SensorFlag bits.
    std::uint32_t flags{0};
};

/// What the adapter implements. A feature it lacks is declared false, never faked.
struct GpsAdapterCapabilities {
    bool delay{false};
    bool position_noise{false};
    bool velocity_noise{false};
    bool drift{false};
    /// Fills ground_speed_mps, course_deg and course_valid.
    bool course{false};
    /// Fills the accuracy estimates, satellites_visible and fix_type.
    bool receiver_status{false};
    bool deterministic_with_seed{false};
    bool thread_safe_across_instances{false};
};

/// One GPS receiver of one vehicle.
class IGpsSensorAdapter : public ISensorAdapter {
public:
    virtual bool step(const GpsTruth& truth, GpsSample& out, std::string& error) = 0;

    virtual GpsAdapterCapabilities capabilities() const = 0;
};

}  // namespace hakoniwa::sensor_adapter
