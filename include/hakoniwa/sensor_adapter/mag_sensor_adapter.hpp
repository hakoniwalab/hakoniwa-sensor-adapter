#pragma once

#include <cstdint>
#include <string>

#include "hakoniwa/sensor_adapter/sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter {

/// Setup of a magnetometer adapter (sensor_name is "mag" by default).
struct MagAdapterSetup : SensorAdapterSetup {
    MagAdapterSetup() { sensor_name = "mag"; }
};

/**
 * @brief True magnetic field for one physics step.
 *
 * The host takes the earth field from the simulation location and has already
 * rotated it into the sensor axes with the mounting orientation.
 */
struct MagTruth {
    /// Simulation time of this step [ns].
    std::uint64_t sim_time_ns{0};
    /// Magnetic field [nT], sensor axes.
    Vector3 magnetic_field{};
    /// Magnetic field [nT] in the world (NWU), and the rotation from the world
    /// to the sensor axes. For adapters that rotate the world field themselves.
    Vector3 magnetic_field_world{};
    Quaternion orientation_world_to_sensor{};
};

/// One measured magnetic field.
struct MagSample {
    bool updated{false};
    std::uint64_t sample_time_ns{0};
    /// Measured field [nT], sensor axes. Held between measurements.
    Vector3 value{};
    /// SensorFlag bits.
    std::uint32_t flags{0};
};

/// What the adapter implements. A feature it lacks is declared false, never faked.
struct MagAdapterCapabilities {
    bool delay{false};
    bool noise{false};
    bool hard_iron{false};
    bool soft_iron{false};
    bool drift{false};
    bool resolution{false};
    bool range_clamp{false};
    bool deterministic_with_seed{false};
    bool thread_safe_across_instances{false};
};

/// One magnetometer of one vehicle.
class IMagSensorAdapter : public ISensorAdapter {
public:
    virtual bool step(const MagTruth& truth, MagSample& out, std::string& error) = 0;

    virtual MagAdapterCapabilities capabilities() const = 0;
};

}  // namespace hakoniwa::sensor_adapter
