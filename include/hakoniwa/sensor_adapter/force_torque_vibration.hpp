#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hakoniwa/sensor_adapter/force_torque_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter {

/**
 * @brief Setup of a force-torque vibration model (docs/spec/force-torque-vibration.md FTV-010).
 *
 * The fields of SensorAdapterSetup: the model's configuration file, the vehicle and sensor names (random
 * streams, FTV-610), the physics step and the seed. sensor_name is that of the force-torque sensor the
 * vibration is added in front of.
 */
struct ForceTorqueVibrationSetup : SensorAdapterSetup {
    ForceTorqueVibrationSetup() { sensor_name = "force_torque"; }
};

/**
 * @brief Vibration of a moving arm, added by the host to the true wrench in front of a force-torque
 * adapter (docs/design/force-torque-vibration.md, docs/spec/force-torque-vibration.md).
 *
 * The model does not know the adapter and the adapter does not know the model:
 *
 * @code
 * Wrench vibration;
 * if (!model->step(t_ns, joint_velocities, vibration, error)) { ... }
 * truth.wrench = add(physics_wrench, vibration);
 * adapter->step(truth, sample, error);
 * @endcode
 */
class IForceTorqueVibration {
public:
    virtual ~IForceTorqueVibration() = default;

    /// One physics step (FTV-020, FTV-030): joint velocities [rad/s, joint side] in the order of joint_names().
    /// Returns the wrench to add [N, N m, sensor axes], or false with error and no change of state.
    virtual bool step(std::uint64_t sim_time_ns, const std::vector<double>& joint_velocity, Wrench& out,
                      std::string& error) = 0;

    /// One physics step with the joint loads (FTV-022): the torque each joint transmits [N m, joint side], in the
    /// order of joint_names(). Needed by a model with a load gain (FTV-170); other models ignore the loads.
    virtual bool step(std::uint64_t sim_time_ns, const std::vector<double>& joint_velocity,
                      const std::vector<double>& joint_load, Wrench& out, std::string& error)
    {
        (void)joint_load;
        return step(sim_time_ns, joint_velocity, out, error);
    }

    /// True when the model needs the joint loads (FTV-024); the host refuses the model if it cannot provide them.
    virtual bool needs_joint_load() const { return false; }

    /// Back to the state at creation (FTV-040).
    virtual bool reset(std::string& error) = 0;

    /// The configured joints, in the order step() expects their velocities.
    virtual std::vector<std::string> joint_names() const = 0;
};

/// Creates one model. Returns nullptr and sets error on failure (FTV-010, FTV-510).
using ForceTorqueVibrationFactory = std::unique_ptr<IForceTorqueVibration> (*)(
    const ForceTorqueVibrationSetup& setup, std::string& error);

/// Version of HostModelImplementation. A host refuses a descriptor with another version.
/// 2: IForceTorqueVibration gained the step with joint loads and needs_joint_load() (FTV-022, FTV-024).
constexpr std::uint32_t kHostModelImplementationVersion = 2;

/**
 * @brief The host-side models of one implementation (FTV-050).
 *
 * Separate from SensorAdapterImplementation, which is not changed: an implementation without host models
 * simply has no host-model entry point. A model the implementation does not provide is nullptr.
 */
struct HostModelImplementation {
    std::uint32_t version{kHostModelImplementationVersion};
    /// Implementation name: a C identifier, the same as in the entry point name.
    const char* name{nullptr};

    ForceTorqueVibrationFactory create_force_torque_vibration{nullptr};
};

}  // namespace hakoniwa::sensor_adapter

/**
 * @brief Entry point of an implementation's host models: hako_sensor_host_model_<name>().
 *
 * @code
 * // in the implementation library
 * HAKO_SENSOR_HOST_MODEL_DEFINE(hakoniwa)
 * {
 *     static const hakoniwa::sensor_adapter::HostModelImplementation implementation = [] {
 *         hakoniwa::sensor_adapter::HostModelImplementation value;
 *         value.name = "hakoniwa";
 *         value.create_force_torque_vibration = &create_force_torque_vibration;
 *         return value;
 *     }();
 *     return &implementation;
 * }
 *
 * // in the host
 * HAKO_SENSOR_HOST_MODEL_DECLARE(hakoniwa);
 * const auto* models = HAKO_SENSOR_HOST_MODEL_ENTRY(hakoniwa)();
 * @endcode
 */
#define HAKO_SENSOR_HOST_MODEL_ENTRY(name) hako_sensor_host_model_##name

#define HAKO_SENSOR_HOST_MODEL_DECLARE(name) \
    extern "C" const ::hakoniwa::sensor_adapter::HostModelImplementation* HAKO_SENSOR_HOST_MODEL_ENTRY(name)()

#define HAKO_SENSOR_HOST_MODEL_DEFINE(name) \
    HAKO_SENSOR_HOST_MODEL_DECLARE(name); \
    extern "C" const ::hakoniwa::sensor_adapter::HostModelImplementation* HAKO_SENSOR_HOST_MODEL_ENTRY(name)()
