#pragma once

#include <cstdint>
#include <string>

#include "hakoniwa/sensor_adapter/sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter {

/// Setup of an IMU adapter (sensor_name is "imu" by default).
struct ImuAdapterSetup : SensorAdapterSetup {
    ImuAdapterSetup() { sensor_name = "imu"; }
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
    /// (kinematic acceleration minus gravity; reads +g along the sensor z at rest, level).
    Vector3 specific_force{};
    /// Rotation from the world (NWU) to the sensor axes. For adapters that take
    /// the kinematic acceleration instead of the specific force.
    Quaternion orientation_world_to_sensor{};
    /// Gravity [m/s^2] in the world (NWU), e.g. {0, 0, -9.80665}.
    Vector3 gravity_world{0.0, 0.0, -9.80665};
};

/// Bits of ImuChannel::flags (the shared SensorFlag bits).
enum ImuFlag : std::uint32_t {
    kImuFlagSaturated = kSensorFlagSaturated,
};

/// One measured channel (gyro or accelerometer).
struct ImuChannel {
    /// True when the adapter measured a new value in this step.
    bool updated{false};
    /// Simulation time at which the value was measured [ns].
    std::uint64_t sample_time_ns{0};
    /// Measured value, sensor axes. Held between measurements.
    Vector3 value{};
    /// SensorFlag bits.
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

/// One IMU (gyro and accelerometer) of one vehicle.
class IImuSensorAdapter : public ISensorAdapter {
public:
    /// Advances one physics step. Returns false and sets error on failure.
    virtual bool step(const ImuTruth& truth, ImuSample& out, std::string& error) = 0;

    virtual ImuAdapterCapabilities capabilities() const = 0;

    /// Calibration values in use since the last reset. Returns false when
    /// capabilities().calibration_values is false.
    virtual bool calibration(ImuCalibration& out) const = 0;
};

}  // namespace hakoniwa::sensor_adapter
