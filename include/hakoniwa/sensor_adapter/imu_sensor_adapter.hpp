#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hakoniwa::sensor_adapter {

/**
 * @brief Three-component vector.
 *
 * Frames follow the Hakoniwa convention: the world is NED (North-East-Down),
 * the body and the sensor are FRD (Forward-Right-Down).
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
 * @brief What the host passes when it creates one IMU adapter instance.
 *
 * The host does not pass its own vehicle configuration. The adapter reads its
 * own configuration file in its own format (for example an SDFormat
 * <sensor type="imu"> for a Gazebo adapter).
 */
struct ImuAdapterSetup {
    /// The adapter's own configuration file.
    std::string config_path;
    /// Vehicle name and index, and sensor name (logs and random streams).
    std::string vehicle_name;
    std::uint32_t vehicle_index{0};
    std::string sensor_name{"imu"};
    /// Fixed physics step of the host [ns]. Every step() advances by this.
    std::uint64_t physics_step_ns{1000000};
    /// Random seed. Without a seed the adapter may draw a different sequence each run.
    bool has_random_seed{false};
    std::uint64_t random_seed{0};
};

/**
 * @brief True values at the sensor position, in the sensor axes, for one physics step.
 *
 * The host has already applied the mounting pose (position, then orientation)
 * and any rotor vibration. The adapter models the sensor errors only.
 */
struct ImuTruth {
    /// Simulation time of this step [ns]. Starts at 0 and grows by physics_step_ns.
    std::uint64_t sim_time_ns{0};
    /// Angular velocity [rad/s], sensor axes.
    Vector3 angular_velocity{};
    /// Specific force [m/s^2], sensor axes: what an ideal accelerometer measures
    /// (kinematic acceleration minus gravity; reads -g along the sensor z at rest, level).
    Vector3 specific_force{};
    /// Rotation from the world (NED) to the sensor axes. For adapters that take
    /// the kinematic acceleration instead of the specific force.
    Quaternion orientation_world_to_sensor{};
    /// Gravity [m/s^2] in the world (NED), e.g. {0, 0, 9.80665}.
    Vector3 gravity_world{0.0, 0.0, 9.80665};
};

/// Bits of ImuChannel::flags. Bits not defined here are 0.
enum ImuFlag : std::uint32_t {
    /// At least one axis of this sample was clipped at the measurement range.
    kImuFlagSaturated = 1u << 0,
};

/// One measured channel (gyro or accelerometer).
struct ImuChannel {
    /// True when the adapter measured a new value in this step.
    bool updated{false};
    /// Simulation time at which the value was measured [ns].
    std::uint64_t sample_time_ns{0};
    /// Measured value, sensor axes. Held between measurements.
    Vector3 value{};
    /// ImuFlag bits.
    std::uint32_t flags{0};
};

/// Output of one step. The gyro and the accelerometer may have their own timing.
struct ImuSample {
    ImuChannel gyro{};   ///< [rad/s]
    ImuChannel accel{};  ///< specific force [m/s^2]
};

/// Accelerometer calibration values the host writes to the autopilot.
struct ImuCalibration {
    /// Zero-g offset [m/s^2], sensor axes.
    Vector3 accel_zero_g_offset{};
    /// Scale factor per axis (1 means no error).
    Vector3 accel_scale{1.0, 1.0, 1.0};
};

/// What the adapter implements. A feature it lacks is declared false, never faked.
struct ImuAdapterCapabilities {
    bool filter{false};
    bool delay{false};
    bool separate_gyro_accel_timing{false};
    bool noise{false};
    bool bias{false};
    bool drift{false};
    bool resolution{false};
    bool range_clamp{false};
    bool calibration_values{false};
    /// Same seed, same inputs and same call order give the same values.
    bool deterministic_with_seed{false};
    /// Different instances may be called from different threads at the same time.
    bool thread_safe_across_instances{false};
};

/// One log column: name and unit, chosen by the adapter.
struct LogColumn {
    std::string name;
    std::string unit;
};

/**
 * @brief One IMU (gyro and accelerometer) of one vehicle.
 *
 * Rules:
 * - one instance is called from one thread;
 * - step() is called once per physics step with sim_time_ns = 0, dt, 2 dt, ...;
 *   a time that does not advance by physics_step_ns is an error;
 * - reset() returns every internal state (delay history, sampling phase,
 *   drift, offsets drawn at start, random streams) to its state at creation.
 */
class IImuSensorAdapter {
public:
    virtual ~IImuSensorAdapter() = default;

    /// Advances one physics step. Returns false and sets error on failure.
    virtual bool step(const ImuTruth& truth, ImuSample& out, std::string& error) = 0;

    /// Returns the instance to its state at creation; the next step() starts at time 0.
    virtual bool reset(std::string& error) = 0;

    virtual ImuAdapterCapabilities capabilities() const = 0;

    /// Calibration values in use since the last reset. Returns false when
    /// capabilities().calibration_values is false.
    virtual bool calibration(ImuCalibration& out) const = 0;

    /// Log columns. Fixed for the life of the instance; may be empty.
    virtual const std::vector<LogColumn>& log_columns() const = 0;

    /// Values of the log columns after the last step(), in column order.
    /// Reading does not change the adapter state.
    virtual void log_values(std::vector<double>& values) const = 0;
};

}  // namespace hakoniwa::sensor_adapter
