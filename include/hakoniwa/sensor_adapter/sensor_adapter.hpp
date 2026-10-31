#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hakoniwa::sensor_adapter {

/**
 * @brief Three-component vector.
 *
 * Frames follow the Hakoniwa convention: the world is NWU (x North, y West,
 * z Up), the body and the sensor are FLU (x Forward, y Left, z Up).
 */
struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

/// Unit quaternion (w, x, y, z).
struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

/**
 * @brief What the host passes when it creates one sensor adapter instance.
 *
 * The host does not pass its own vehicle configuration. The adapter reads its
 * own configuration file in its own format (for example an SDFormat <sensor>
 * for a Gazebo adapter).
 */
struct SensorAdapterSetup {
    /// The adapter's own configuration file.
    std::string config_path;
    /// Vehicle name and index, and sensor name (logs and random streams).
    std::string vehicle_name;
    std::uint32_t vehicle_index{0};
    std::string sensor_name;
    /// Fixed physics step of the host [ns]. Every step() advances by this.
    std::uint64_t physics_step_ns{1000000};
    /// Random seed. Without a seed the adapter may draw a different sequence each run.
    bool has_random_seed{false};
    std::uint64_t random_seed{0};
};

/// Bits of a sample's flags, shared by every sensor. Bits not defined here are 0.
enum SensorFlag : std::uint32_t {
    /// At least one component of this sample was clipped at the measurement range.
    kSensorFlagSaturated = 1u << 0,
};

/// One log column: name and unit, chosen by the adapter.
struct LogColumn {
    std::string name;
    std::string unit;
};

/**
 * @brief What every sensor adapter has, whatever it measures.
 *
 * Rules for every sensor adapter:
 * - one instance is one sensor of one vehicle, called from one thread;
 * - the sensor's step() is called once per physics step with
 *   sim_time_ns = 0, dt, 2 dt, ...; a time that does not advance by
 *   physics_step_ns is an error;
 * - a value is measured on the boundaries of the adapter's own period and
 *   held between them;
 * - failures return false with a message; no exception crosses the interface.
 */
class ISensorAdapter {
public:
    virtual ~ISensorAdapter() = default;

    /// Returns every internal state (delay history, sampling phase, drift,
    /// offsets drawn at start, random streams) to its state at creation;
    /// the next step() starts at time 0.
    virtual bool reset(std::string& error) = 0;

    /// Log columns. Fixed for the life of the instance; may be empty.
    virtual const std::vector<LogColumn>& log_columns() const = 0;

    /// Values of the log columns after the last step(), in column order.
    /// Reading does not change the adapter state.
    virtual void log_values(std::vector<double>& values) const = 0;
};

}  // namespace hakoniwa::sensor_adapter
