#pragma once

#include <cstdint>
#include <string>

#include "hakoniwa/sensor_adapter/sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter {

/// Setup of a force-torque adapter (sensor_name is "force_torque" by default).
struct ForceTorqueAdapterSetup : SensorAdapterSetup {
    ForceTorqueAdapterSetup() { sensor_name = "force_torque"; }
};

/// Force [N] and torque [N m], sensor axes (FLU).
struct Wrench {
    Vector3 force{};
    Vector3 torque{};
};

/**
 * @brief True wrench at the sensor for one physics step (docs/design/force-torque-adapter.md 3.1).
 *
 * The wrench is what the tool side (the bodies beyond the sensor) exerts on the
 * robot side, in the sensor axes, with the torque about the sensor origin. It
 * includes contact forces and the tool's weight and inertia. A tool hanging at
 * rest gives a force equal to its weight, pointing along gravity.
 */
struct ForceTorqueTruth {
    /// Simulation time of this step [ns]. Starts at 0 and grows by physics_step_ns.
    std::uint64_t sim_time_ns{0};
    /// True wrench, sensor axes, torque about the sensor origin.
    Wrench wrench{};
    /// Rotation from the world (NWU) to the sensor axes. For tool-weight
    /// compensation outside the sensor and for adapters that use gravity.
    Quaternion orientation_world_to_sensor{};
    /// Gravity [m/s^2] in the world (NWU), e.g. {0, 0, -9.80665}.
    Vector3 gravity_world{0.0, 0.0, -9.80665};
};

/// Bits of ForceTorqueSample::flags: the shared SensorFlag bits and the force-torque bits.
enum ForceTorqueFlag : std::uint32_t {
    /// At least one axis of this sample was clipped at the measurement range.
    kForceTorqueFlagSaturated = kSensorFlagSaturated,
    /// Since the previous sample, the physical load on at least one axis
    /// exceeded the overload limit (judged on the truth, before any error).
    kForceTorqueFlagOverloaded = 1u << 8,
};

/// Output of one step.
struct ForceTorqueSample {
    /// True when the adapter measured a new value in this step.
    bool updated{false};
    /// Simulation time at which the value was measured [ns].
    std::uint64_t sample_time_ns{0};
    /// Measured wrench, sensor axes, after tare. Held between measurements.
    Wrench wrench{};
    /// ForceTorqueFlag bits of the held value.
    std::uint32_t flags{0};
};

/// What the adapter implements. A feature it lacks is declared false, never faked.
struct ForceTorqueAdapterCapabilities {
    bool delay{false};
    bool lowpass{false};
    bool nonlinearity{false};
    /// Scale error and crosstalk as a 6x6 matrix on range-normalised values.
    bool calibration_matrix{false};
    bool bias{false};
    bool drift{false};
    bool noise{false};
    bool resolution{false};
    /// Clips at the measurement range and sets kForceTorqueFlagSaturated.
    bool range_clamp{false};
    /// Sets kForceTorqueFlagOverloaded.
    bool overload{false};
    /// Keeps a zero offset after an overload.
    bool overload_offset{false};
    /// Temperature dependence (the host passes a temperature).
    bool temperature{false};
    bool hysteresis{false};
    /// tare() and clear_tare() work.
    bool tare{false};
    /// Same seed, same inputs and same call order give the same values.
    bool deterministic_with_seed{false};
    /// Different instances may be called from different threads at the same time.
    bool thread_safe_across_instances{false};
};

/// One 6-axis force-torque sensor of one vehicle.
class IForceTorqueSensorAdapter : public ISensorAdapter {
public:
    /// Advances one physics step. Returns false and sets error on failure.
    virtual bool step(const ForceTorqueTruth& truth, ForceTorqueSample& out, std::string& error) = 0;

    /// Remembers the mean of the samples of the last tare window (before tare and
    /// resolution) and subtracts it from later samples. Drift keeps running.
    /// Returns false when no sample has been measured since creation or reset,
    /// or when capabilities().tare is false.
    virtual bool tare(std::string& error) = 0;

    /// Forgets the remembered tare. reset() also forgets it.
    virtual void clear_tare() = 0;

    virtual ForceTorqueAdapterCapabilities capabilities() const = 0;
};

}  // namespace hakoniwa::sensor_adapter
