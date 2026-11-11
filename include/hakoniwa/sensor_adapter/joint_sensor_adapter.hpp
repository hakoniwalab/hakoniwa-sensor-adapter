#pragma once

#include <cstdint>
#include <string>

#include "hakoniwa/sensor_adapter/sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter {

/// Setup of a joint sensor adapter (sensor_name is "joint" by default; use the joint name).
struct JointSensorAdapterSetup : SensorAdapterSetup {
    JointSensorAdapterSetup() { sensor_name = "joint"; }
};

/**
 * @brief True values of one revolute joint for one physics step (docs/design/joint-sensor-adapter.md 3.1).
 *
 * The physics has no gearbox: position and velocity are the joint (link) side values.
 * actuator_torque is the sum of the actuators driving the joint; transmitted_torque is the
 * torque the parent side exerts on the child side about the joint axis, which a joint
 * torque sensor measures (equal to actuator_torque when holding still without friction).
 */
struct JointSensorTruth {
    /// Simulation time of this step [ns]. Starts at 0 and grows by physics_step_ns.
    std::uint64_t sim_time_ns{0};
    double position{0.0};            ///< [rad]
    double velocity{0.0};            ///< [rad/s]
    double actuator_torque{0.0};     ///< [N m]
    double transmitted_torque{0.0};  ///< [N m]
    /// False when the host cannot compute transmitted_torque and passes actuator_torque instead.
    bool has_transmitted_torque{true};
};

/// Output of one step: one joint of a JointState.
struct JointSensorSample {
    /// True when the adapter measured a new value in this step.
    bool updated{false};
    /// Simulation time at which the value was measured [ns].
    std::uint64_t sample_time_ns{0};
    double position{0.0};  ///< [rad], joint side
    double velocity{0.0};  ///< [rad/s], joint side
    double effort{0.0};    ///< [N m]; meaningful when has_effort
    bool has_effort{false};
    /// Encoder count at the encoder shaft (after wrap); 0 without an encoder model.
    std::int64_t encoder_count{0};
    /// SensorFlag bits of the held value (saturated: the torque was clipped at its range).
    std::uint32_t flags{0};
};

/// What the adapter implements. A feature it lacks is declared false, never faked.
struct JointSensorAdapterCapabilities {
    bool delay{false};
    bool encoder_counts{false};
    bool motor_side{false};
    bool backlash{false};
    bool stiffness{false};
    bool offset{false};
    bool cyclic_error{false};
    bool noise{false};
    bool incremental{false};
    bool wrap{false};
    bool velocity_from_difference{false};
    bool velocity_lsb{false};
    bool torque_sensor{false};
    bool torque_from_current{false};
    bool torque_errors{false};
    /// Same seed, same inputs and same call order give the same values.
    bool deterministic_with_seed{false};
    /// Different instances may be called from different threads at the same time.
    bool thread_safe_across_instances{false};
};

/// The sensors of one revolute joint (encoder and torque).
class IJointSensorAdapter : public ISensorAdapter {
public:
    /// Advances one physics step. Returns false and sets error on failure.
    virtual bool step(const JointSensorTruth& truth, JointSensorSample& out, std::string& error) = 0;
    virtual JointSensorAdapterCapabilities capabilities() const = 0;
};

}  // namespace hakoniwa::sensor_adapter
