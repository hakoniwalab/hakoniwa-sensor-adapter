#pragma once

#include <cstdint>
#include <string>

#include "hakoniwa/sensor_adapter/sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter {

/// Setup of a barometer adapter (sensor_name is "baro" by default).
struct BaroAdapterSetup : SensorAdapterSetup {
    BaroAdapterSetup() { sensor_name = "baro"; }
};

/**
 * @brief True values for one physics step.
 *
 * The host gives the height of the sensor position and the weather at sea
 * level. The atmosphere model (pressure and temperature from height) belongs
 * to the adapter, like the other parts of the sensor model.
 */
struct BaroTruth {
    /// Simulation time of this step [ns].
    std::uint64_t sim_time_ns{0};
    /// Height of the sensor position above mean sea level [m] (geometric).
    double altitude_m{0.0};
    /// Weather at sea level.
    double sea_level_pressure_pa{101325.0};
    double sea_level_temperature_c{15.0};
};

/// One measured air pressure.
struct BaroSample {
    bool updated{false};
    std::uint64_t sample_time_ns{0};
    /// Measured static pressure [Pa]. Held between measurements.
    double pressure_pa{0.0};
    /// Temperature at the sensor [deg C].
    double temperature_c{0.0};
    /// Altitude derived from the measured pressure [m] (pressure altitude).
    double pressure_altitude_m{0.0};
    /// SensorFlag bits.
    std::uint32_t flags{0};
};

/// What the adapter implements. A feature it lacks is declared false, never faked.
struct BaroAdapterCapabilities {
    /// Uses the sea-level weather of BaroTruth (otherwise a fixed standard atmosphere).
    bool sea_level_weather{false};
    /// Gives temperature_c and pressure_altitude_m (otherwise they are 0).
    bool temperature{false};
    bool pressure_altitude{false};
    bool delay{false};
    bool noise{false};
    bool offset{false};
    bool drift{false};
    bool resolution{false};
    bool range_clamp{false};
    bool deterministic_with_seed{false};
    bool thread_safe_across_instances{false};
};

/// One barometer of one vehicle.
class IBaroSensorAdapter : public ISensorAdapter {
public:
    virtual bool step(const BaroTruth& truth, BaroSample& out, std::string& error) = 0;

    virtual BaroAdapterCapabilities capabilities() const = 0;
};

}  // namespace hakoniwa::sensor_adapter
